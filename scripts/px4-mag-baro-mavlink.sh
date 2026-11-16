#!/bin/bash
# Start a dedicated PX4 mavlink instance on localhost for mag_baro_bridge
# and enable the HIGHRES_IMU stream on it.
#
# Called from /etc/modalai/voxl-px4.conf as the ONLY EXTRA_STEPS entry:
#   EXTRA_STEPS=( "/data/px4-mag-baro-mavlink.sh" )
# (voxl-px4 only passes the first EXTRA_STEPS element to voxl-px4-start.)

px4-mavlink start -u 14570 -o 14571 -t 127.0.0.1 -r 20000 -m minimal

# give the new instance a moment to come up before configuring streams
sleep 1

px4-mavlink stream -u 14570 -s HIGHRES_IMU -r 50
