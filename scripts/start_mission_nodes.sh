#!/bin/bash
S=/tmp/sage_logs
# Idempotent: stop any running copies first.
for n in sage_mission_parser sage_mission_manager sage_viewpoint_planner; do
  ps -eo pid,args | grep "[l]ib/sage_px4_interface/$n" | grep -v "bash -c" | awk '{print $1}' | xargs -r kill
  ps -eo pid,args | grep "[r]os2 run sage_px4_interface $n" | grep -v "bash -c" | awk '{print $1}' | xargs -r kill
done
sleep 1
source /opt/ros/humble/setup.bash; source ~/sage_ws/install/setup.bash
# Per-world settings (search area, known obstacles): config/<world>.env
CONF=~/SAGE-UAV/config/${SAGE_WORLD:-sage_test}.env
[ -f "$CONF" ] && source $CONF
PLAN_ARGS=""; MGR_ARGS=""
[ -n "$SAGE_AREA" ] && PLAN_ARGS="$PLAN_ARGS -p area:=$SAGE_AREA"
[ -n "$SAGE_OBSTACLES" ] && PLAN_ARGS="$PLAN_ARGS -p obstacles:=$SAGE_OBSTACLES" && MGR_ARGS="-p obstacles:=$SAGE_OBSTACLES"
[ -n "$SAGE_PLANNER_ARGS" ] && PLAN_ARGS="$PLAN_ARGS $SAGE_PLANNER_ARGS"
setsid nohup ros2 run sage_px4_interface sage_mission_parser > $S/parser.log 2>&1 < /dev/null &
setsid nohup ros2 run sage_px4_interface sage_mission_manager ${MGR_ARGS:+--ros-args $MGR_ARGS} > $S/mission.log 2>&1 < /dev/null &
sleep 2
setsid nohup ros2 run sage_px4_interface sage_viewpoint_planner ${PLAN_ARGS:+--ros-args $PLAN_ARGS} > $S/planner.log 2>&1 < /dev/null &
