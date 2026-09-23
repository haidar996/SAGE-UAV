#!/bin/bash
S=/tmp/sage_logs
ps -eo pid,args | grep -E "[s]age_viewpoint_planner|[s]age_mission_manager" | grep -v "bash -c" | awk '{print $1}' | xargs -r kill
sleep 2
source /opt/ros/humble/setup.bash; source ~/sage_ws/install/setup.bash
setsid nohup ros2 run sage_px4_interface sage_mission_manager > $S/mission2.log 2>&1 < /dev/null &
sleep 2
setsid nohup ros2 run sage_px4_interface sage_viewpoint_planner > $S/planner2.log 2>&1 < /dev/null &
