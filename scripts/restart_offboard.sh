#!/bin/bash
ps -eo pid,args | grep "[l]ib/sage_px4_interface/offboard" | grep -v "bash -c" | awk '{print $1}' | xargs -r kill
ps -eo pid,args | grep "[r]os2 run sage_px4_interface offboard" | grep -v "bash -c" | awk '{print $1}' | xargs -r kill
sleep 2
source /opt/ros/humble/setup.bash; source ~/sage_ws/install/setup.bash
setsid nohup ros2 run sage_px4_interface offboard_position_node > /tmp/sage_logs/offboard.log 2>&1 < /dev/null &
