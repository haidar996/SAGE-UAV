#!/bin/bash
S=/tmp/sage_logs
source /opt/ros/humble/setup.bash; source ~/sage_ws/install/setup.bash
setsid nohup ros2 run sage_px4_interface sage_mission_manager > $S/mission.log 2>&1 < /dev/null &
sleep 2
setsid nohup ros2 run sage_px4_interface sage_viewpoint_planner > $S/planner.log 2>&1 < /dev/null &
