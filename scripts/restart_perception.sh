#!/bin/bash
# Restart YOLO + target localizer (e.g. after code changes). SAGE_WORLD, ATT env.
S=/tmp/sage_logs
export SAGE_WORLD=${SAGE_WORLD:-sage_test}
for pat in "lib/sage_px4_interface/sage_target_localizer" "yolo_detector_launcher"; do
  ps -eo pid,args | grep -F "$pat" | grep -v grep | grep -v "bash -c" | grep -v restart_perception | awk '{print $1}' | xargs -r kill
done
sleep 2
source /opt/ros/humble/setup.bash; source ~/sage_ws/install/setup.bash
setsid nohup ~/sage_vision_env/bin/python ~/sage_ws/install/sage_px4_interface/lib/sage_px4_interface/yolo_detector_launcher.py > $S/yolo.log 2>&1 < /dev/null &
setsid nohup ros2 run sage_px4_interface sage_target_localizer --ros-args -p attitude_source:=${ATT:-gz_truth} > $S/localizer.log 2>&1 < /dev/null &
