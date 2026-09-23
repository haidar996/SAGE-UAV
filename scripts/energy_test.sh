#!/bin/bash
S=/tmp/sage_logs
ps -eo pid,args | grep -F "sage_energy_monitor" | grep -v grep | grep -v "bash -c" | grep -v energy_test | awk '{print $1}' | xargs -r kill -9
sleep 1
source /opt/ros/humble/setup.bash; source ~/sage_ws/install/setup.bash
setsid nohup ros2 run sage_px4_interface sage_energy_monitor --ros-args -p reserve_fraction:=${1:-0.75} > $S/energy.log 2>&1 < /dev/null &
