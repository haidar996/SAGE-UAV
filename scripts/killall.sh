#!/bin/bash
for pat in sage_mission_parser sage_energy_monitor sage_viewpoint_planner sage_mission_manager offboard_position_nod sage_semantic_world sage_target_localizer yolo_detector_launcher parameter_bridge MicroXRCEAgent "bin/px4 -d" "gz sim"; do
  ps -eo pid,args | grep -F "$pat" | grep -v grep | grep -v "bash -c" | grep -v killall | awk '{print $1}' | xargs -r kill -9
done
sleep 2
ps -eo pid,args | grep -E "sage_|px4|gz sim|MicroXRCE|parameter_bridge|yolo" | grep -v grep | grep -v "bash -c" | grep -v killall
echo remaining-check-done
