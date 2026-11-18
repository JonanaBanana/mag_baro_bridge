"""Publish PX4 magnetometer and barometer data from MAVLink HIGHRES_IMU to ROS 2.

PX4 fills HIGHRES_IMU from vehicle_magnetometer (calibrated, body frame) and
vehicle_air_data, and sets bits in `fields_updated` only when that sensor has
produced a new sample. This node publishes each sensor only on fresh data.

The MAVLink decoding is done here with the standard library (no pymavlink):
only HIGHRES_IMU (msgid 105) is parsed, and its CRC is checked.

Packets are received on a background thread that blocks in recv(), so the node
only wakes up when PX4 sends data. (A high-rate rclpy timer polling the socket
cost ~50% of a core on the VOXL2 with Foxy.)
"""
import socket
import struct
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import FluidPressure, MagneticField, Temperature
from std_msgs.msg import Float64

# --- MAVLink HIGHRES_IMU definition (common.xml) ---------------------------
HIGHRES_IMU_ID = 105
HIGHRES_IMU_CRC_EXTRA = 93
# Wire order: uint64 time_usec, 13 floats (xacc..zacc, xgyro..zgyro,
# xmag..zmag, abs_pressure, diff_pressure, pressure_alt, temperature),
# uint16 fields_updated, uint8 id (extension field, MAVLink 2 only).
HIGHRES_IMU = struct.Struct('<Q13fHB')  # 63 bytes

MAVLINK1_STX = 0xFE
MAVLINK2_STX = 0xFD
MAVLINK2_SIGNED = 0x01

# HIGHRES_IMU_UPDATED_FLAGS
MAG_BITS = (1 << 6) | (1 << 7) | (1 << 8)
ABS_PRESSURE_BIT = 1 << 9
PRESSURE_ALT_BIT = 1 << 11
TEMPERATURE_BIT = 1 << 12

GAUSS_TO_TESLA = 1e-4
HPA_TO_PA = 100.0


def x25_crc(data, crc=0xFFFF):
    """MAVLink checksum (CRC-16/MCRF4XX), same as crc_accumulate() in C."""
    for b in data:
        tmp = (b ^ crc) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def parse_highres_imu(buf):
    """Decode every valid HIGHRES_IMU in a UDP datagram.

    PX4 may pack several MAVLink packets into one datagram, so walk all of it.
    Returns (messages, crc_errors, {(msgid, mavlink_version), ...} seen).
    """
    out = []
    seen_ids = set()
    crc_errors = 0
    i, n = 0, len(buf)
    while i < n:
        stx = buf[i]
        if stx == MAVLINK2_STX:
            hdr = 10
            if i + hdr > n:
                break
            plen = buf[i + 1]
            sig = 13 if buf[i + 2] & MAVLINK2_SIGNED else 0
            msgid = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
        elif stx == MAVLINK1_STX:
            hdr = 6
            if i + hdr > n:
                break
            plen = buf[i + 1]
            sig = 0
            msgid = buf[i + 5]
        else:
            i += 1
            continue

        end = i + hdr + plen + 2  # header + payload + CRC
        if end + sig > n:
            break

        seen_ids.add((msgid, 2 if stx == MAVLINK2_STX else 1))
        if msgid == HIGHRES_IMU_ID:
            crc = x25_crc(buf[i + 1:i + hdr + plen])
            crc = x25_crc((HIGHRES_IMU_CRC_EXTRA,), crc)
            if crc != (buf[end - 2] | (buf[end - 1] << 8)):
                crc_errors += 1
                i += 1  # not a real packet start; resync
                continue
            # MAVLink 2 strips trailing zero bytes, MAVLink 1 has no extension.
            payload = bytes(buf[i + hdr:i + hdr + plen]).ljust(HIGHRES_IMU.size, b'\0')
            out.append(HIGHRES_IMU.unpack(payload[:HIGHRES_IMU.size]))

        i = end + sig
    return out, crc_errors, seen_ids


class MagBaroBridge(Node):

    def __init__(self):
        super().__init__('mag_baro_bridge')

        self.declare_parameter('bind_address', '127.0.0.1')
        self.declare_parameter('udp_port', 14571)
        self.declare_parameter('frame_id', 'base_link')
        # PX4 body frame is FRD; ROS convention (REP-103) is FLU.
        self.declare_parameter('frd_to_flu', True)
        self.declare_parameter('timeout_s', 2.0)

        addr = self.get_parameter('bind_address').value
        port = int(self.get_parameter('udp_port').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.frd_to_flu = self.get_parameter('frd_to_flu').value
        self.timeout_ns = int(self.get_parameter('timeout_s').value * 1e9)

        qos = qos_profile_sensor_data
        self.mag_pub = self.create_publisher(MagneticField, 'mag', qos)
        self.press_pub = self.create_publisher(FluidPressure, 'baro/pressure', qos)
        self.temp_pub = self.create_publisher(Temperature, 'baro/temperature', qos)
        self.alt_pub = self.create_publisher(Float64, 'baro/altitude', qos)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((addr, port))
        # Blocking recv with a timeout: wakes on data, or periodically to check
        # the no-data warning and shutdown
        self.sock.settimeout(0.5)
        self.get_logger().info(f'Listening for MAVLink on udp {addr}:{port}')

        self.last_rx_ns = self.get_clock().now().nanoseconds
        self.last_warn_ns = 0
        self.got_first = False
        self.crc_errors = 0
        self.rx_datagrams = 0
        self.seen_ids = set()

        self._stop = threading.Event()
        self._rx_thread = threading.Thread(target=self._rx_loop, name='mavlink_rx', daemon=True)
        self._rx_thread.start()

    def stop(self):
        self._stop.set()
        self._rx_thread.join(timeout=2.0)
        self.sock.close()

    def _rx_loop(self):
        while not self._stop.is_set() and rclpy.ok():
            try:
                data = self.sock.recv(4096)
            except socket.timeout:
                self._check_timeout()
                continue
            except InterruptedError:
                continue
            except OSError:
                break  # socket closed during shutdown
            self.rx_datagrams += 1
            msgs, errs, ids = parse_highres_imu(data)
            self.seen_ids |= ids
            if errs:
                self.crc_errors += errs
                self.get_logger().warn(f'MAVLink CRC errors: {self.crc_errors} total')
            for m in msgs:
                self.on_highres_imu(m)
            self._check_timeout()

    def _check_timeout(self):
        now = self.get_clock().now().nanoseconds
        if now - self.last_rx_ns > self.timeout_ns and now - self.last_warn_ns > 5e9:
            self.last_warn_ns = now
            if self.rx_datagrams == 0:
                detail = 'no UDP packets at all on this port (PX4 is not sending here)'
            else:
                ids = ', '.join(f'{i} (v{v})' for i, v in sorted(self.seen_ids))
                detail = (f'{self.rx_datagrams} UDP packets received, '
                          f'MAVLink msgids seen: {ids or "none"} '
                          f'(105 = HIGHRES_IMU, 0 = HEARTBEAT)')
            self.get_logger().warn(f'No HIGHRES_IMU received: {detail}')

    # NB: must not be called `handle` - that name is an rclpy Node property.
    def on_highres_imu(self, f):
        (_time_usec,
         _xacc, _yacc, _zacc, _xgyro, _ygyro, _zgyro,
         xmag, ymag, zmag,
         abs_pressure, _diff_pressure, pressure_alt, temperature,
         flags, _imu_id) = f

        stamp = self.get_clock().now()
        self.last_rx_ns = stamp.nanoseconds
        if not self.got_first:
            self.got_first = True
            self.get_logger().info('Receiving HIGHRES_IMU from PX4')
        header_stamp = stamp.to_msg()

        if flags & MAG_BITS:
            m = MagneticField()
            m.header.stamp = header_stamp
            m.header.frame_id = self.frame_id
            x, y, z = xmag, ymag, zmag
            if self.frd_to_flu:
                y, z = -y, -z
            m.magnetic_field.x = x * GAUSS_TO_TESLA
            m.magnetic_field.y = y * GAUSS_TO_TESLA
            m.magnetic_field.z = z * GAUSS_TO_TESLA
            # covariance left all-zero = unknown
            self.mag_pub.publish(m)

        if flags & ABS_PRESSURE_BIT:
            p = FluidPressure()
            p.header.stamp = header_stamp
            p.header.frame_id = self.frame_id
            p.fluid_pressure = abs_pressure * HPA_TO_PA
            self.press_pub.publish(p)

        if flags & TEMPERATURE_BIT:
            t = Temperature()
            t.header.stamp = header_stamp
            t.header.frame_id = self.frame_id
            t.temperature = float(temperature)
            self.temp_pub.publish(t)

        if flags & PRESSURE_ALT_BIT:
            self.alt_pub.publish(Float64(data=float(pressure_alt)))


def main(args=None):
    rclpy.init(args=args)
    node = MagBaroBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
