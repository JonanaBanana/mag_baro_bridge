# mag_baro_bridge

Publishes PX4's calibrated magnetometer and barometer data on a VOXL 2 as ROS 2 topics.

```
PX4 (vehicle_magnetometer, vehicle_air_data)
  └─ mavlink instance, udp 14570 → 127.0.0.1:14571, HIGHRES_IMU
       └─ mag_baro_node (decodes HIGHRES_IMU) → /px4/mag, /px4/baro/*
```

## Topics (default namespace `px4`)

| Topic | Type | Notes |
|---|---|---|
| `/px4/mag` | `sensor_msgs/MagneticField` | Tesla, FLU body frame (set `frd_to_flu:=false` for PX4 FRD) |
| `/px4/baro/pressure` | `sensor_msgs/FluidPressure` | Pa |
| `/px4/baro/temperature` | `sensor_msgs/Temperature` | °C, barometer temperature (board-heated, ~40 °C) |
| `/px4/baro/altitude` | `std_msgs/Float64` | m, standard-atmosphere pressure altitude |

- QoS is `sensor_data` (best effort). Subscribers must use a compatible QoS, e.g.
  `ros2 topic echo /px4/mag --qos-reliability best_effort`.
- Each topic is published only when PX4 marks that sensor as updated in `fields_updated`,
  so no repeated values.
- Header stamps are the ROS time at reception.
- Mag values are calibrated and in body frame (same data as `vehicle_magnetometer`).

### Rates (Starling 2 Max)

| | Driver (`sensor_*`) | Published by PX4 (`vehicle_*`) | Limited by |
|---|---|---|---|
| Magnetometer (QMC5883L) | ~48 Hz | ~14 Hz | PX4 param `SENS_MAG_RATE` = 15 |
| Barometer (ICP-10100) | ~8 Hz | ~8 Hz | baro driver |

## Dependencies

None beyond ROS 2 (Foxy): the node decodes `HIGHRES_IMU` itself, with CRC check, using only
the Python standard library. No pymavlink needed. Both MAVLink 1 and 2 are accepted (PX4 sends
MAVLink 1 on this link, since the node never sends anything).

## Setup on the VOXL

### 1. Make PX4 start the extra mavlink instance

`voxl-px4-start` runs the commands in `EXTRA_STEPS` (from `/etc/modalai/voxl-px4.conf`) at
the end of startup. **Only the first array element actually reaches the start script** (bash
arrays can't be passed to a child process), and `;`/`&&` don't work inside an entry. So both
commands live in [scripts/px4-mag-baro-mavlink.sh](scripts/px4-mag-baro-mavlink.sh), and that
script is the single entry:

```bash
cp scripts/px4-mag-baro-mavlink.sh /data/px4-mag-baro-mavlink.sh
chmod +x /data/px4-mag-baro-mavlink.sh
```

```bash
# /etc/modalai/voxl-px4.conf
EXTRA_STEPS=( "/data/px4-mag-baro-mavlink.sh" )
```

Put any further PX4 startup commands in that script too. Ports 14556-14559 are already used
by the stock mavlink instances.

Apply with `systemctl restart voxl-px4`. To test without restarting PX4, run the commands
from the script by hand.

### 2. Build

```bash
cd /ros2_ws && colcon build --packages-select mag_baro_bridge
source install/setup.bash
ros2 launch mag_baro_bridge mag_baro_bridge.launch.py   # manual test
```

### 3. Start at boot

The systemd service follows the other custom VOXL services in
`AerialTN/voxl_data/services/`:
[voxl-custom-mag-baro.service](../AerialTN/voxl_data/services/voxl-custom-mag-baro.service) and
its wrapper [voxl-custom-mag-baro.sh](../AerialTN/voxl_data/services/voxl-custom-mag-baro.sh).

```bash
cd /ros2_ws/src/AerialTN/voxl_data/services
cp voxl-custom-mag-baro.sh /usr/local/bin/ && chmod +x /usr/local/bin/voxl-custom-mag-baro.sh
cp voxl-custom-mag-baro.service /etc/systemd/system/
systemctl daemon-reload
systemctl enable --now voxl-custom-mag-baro
journalctl -u voxl-custom-mag-baro -f
```

The service starts after `voxl-px4` (ordering only). If PX4 restarts, the node keeps running
and resumes when data comes back.

## Parameters

| Parameter | Default | |
|---|---|---|
| `udp_port` | `14571` | Port PX4 sends to (`-o` in the mavlink start command) |
| `bind_address` | `127.0.0.1` | |
| `frame_id` | `base_link` | |
| `frd_to_flu` | `true` | Convert PX4 FRD body frame to ROS FLU |
| `timeout_s` | `2.0` | Warn if no HIGHRES_IMU for this long |

Launch arguments: `udp_port`, `frame_id`, `namespace` (default `px4`).

## Troubleshooting

The node's log says what is wrong:

| Log message | Meaning / fix |
|---|---|
| `Receiving HIGHRES_IMU from PX4` | Working. |
| `No HIGHRES_IMU received: no UDP packets at all` | The mavlink instance isn't running or isn't sending to 127.0.0.1:14571. Check the start command has `-t 127.0.0.1`. |
| `No HIGHRES_IMU received: N UDP packets ... msgids seen: 0, 1, 24, ...` | Instance runs but the HIGHRES_IMU stream is off: run `px4-mavlink stream -u 14570 -s HIGHRES_IMU -r 50`. If that fixes it, the startup script didn't run (see step 1). |
| `MAVLink CRC errors` | Packets arrive but don't decode. Shouldn't happen. Check nothing else is sending to 14571. |

Useful checks:

- `px4-mavlink status` and `px4-uorb top` print to the PX4 log, not the terminal. Watch
  `journalctl -u voxl-px4 -f` while running them.
- `px4-listener vehicle_magnetometer` / `px4-listener vehicle_air_data` show the PX4-side
  values to compare against.
- `journalctl -u voxl-px4 -b | grep -A3 "EXTRA STEPS"` shows which startup steps PX4 actually
  received.
