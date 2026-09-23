#!/bin/bash
S=/tmp/sage_logs
# Idempotent: stop any running copies first.
for n in sage_mission_parser sage_mission_manager sage_viewpoint_planner; do
  ps -eo pid,args | grep "[l]ib/sage_px4_interface/$n" | grep -v "bash -c" | awk '{print $1}' | xargs -r kill
  ps -eo pid,args | grep "[r]os2 run sage_px4_interface $n" | grep -v "bash -c" | awk '{print $1}' | xargs -r kill
done
sleep 1
source /opt/ros/humble/setup.bash; source ~/sage_ws/install/setup.bash
setsid nohup ros2 run sage_px4_interface sage_mission_parser > $S/parser.log 2>&1 < /dev/null &
setsid nohup ros2 run sage_px4_interface sage_mission_manager > $S/mission.log 2>&1 < /dev/null &
sleep 2
setsid nohup ros2 run sage_px4_interface sage_viewpoint_planner > $S/planner.log 2>&1 < /dev/null &
