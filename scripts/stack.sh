#!/bin/bash
# Full SAGE stack restart. usage: stack.sh [drain_s]
S=/tmp/sage_logs
DRAIN=${1:-600}
export SAGE_WORLD=${SAGE_WORLD:-sage_test}
for pat in "sage_energy_monitor" "sage_viewpoint_planner" "sage_mission_manager" "offboard_position_nod" "sage_semantic_world" "sage_target_localizer" "yolo_detector_launcher" "parameter_bridge" "MicroXRCEAgent" "bin/px4 -d" "gz sim"; do
  ps -eo pid,args | grep -F "$pat" | grep -v grep | grep -v "bash -c" | grep -v "stack.sh" | awk '{print $1}' | xargs -r kill -9
done
sleep 3
source /opt/ros/humble/setup.bash; source ~/sage_ws/install/setup.bash
run(){ n=$1; shift; setsid nohup "$@" > $S/$n.log 2>&1 < /dev/null & }
B=~/PX4-Autopilot/build/px4_sitl_default/rootfs
cd $B && HEADLESS=1 PX4_GZ_WORLD=$SAGE_WORLD PX4_SYS_AUTOSTART=4019 PX4_SIM_MODEL=gz_x500_mono_cam run px4 ../bin/px4 -d ../etc
cd ~/Micro-XRCE-DDS-Agent/build && run xrce ./MicroXRCEAgent udp4 -p 8888
until grep -q "Startup script returned" $S/px4.log 2>/dev/null; do sleep 2; done
cd $B; ../bin/px4-param set NAV_DLL_ACT 0 >/dev/null; ../bin/px4-param set SIM_BAT_DRAIN $DRAIN >/dev/null; ../bin/px4-param set SIM_BAT_MIN_PCT 0 >/dev/null
until grep -q "Ready for takeoff" $S/px4.log 2>/dev/null; do sleep 2; done
run offboard ros2 run sage_px4_interface offboard_position_node
# Arm BEFORE loading the CPU with perception (load starves the sim IMU).
for i in $(seq 1 40); do
  a=$(timeout 5 ros2 topic echo /fmu/out/vehicle_status_v1 --once 2>/dev/null | grep -E "^arming_state" | awk '{print $2}')
  [ "$a" = "2" ] && break; sleep 3
done
echo "arming_state=$a"
sleep 10
P=/world/$SAGE_WORLD/model/x500_mono_cam_0/link/camera_link/sensor/imager
run bridge_img ros2 run ros_gz_bridge parameter_bridge "$P/image@sensor_msgs/msg/Image[gz.msgs.Image"
run bridge_info ros2 run ros_gz_bridge parameter_bridge "$P/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
run yolo ~/sage_vision_env/bin/python ~/sage_ws/install/sage_px4_interface/lib/sage_px4_interface/yolo_detector_launcher.py
# SITL: PX4 yaw estimate is ~28 deg off here, so the localizer uses Gazebo
# ground-truth attitude by default (ATT=px4 to use the PX4 estimate).
run bridge_pose ros2 run ros_gz_bridge parameter_bridge "/world/$SAGE_WORLD/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"
run localizer ros2 run sage_px4_interface sage_target_localizer --ros-args -p attitude_source:=${ATT:-gz_truth}
run world ros2 run sage_px4_interface sage_semantic_world_model
run energy ros2 run sage_px4_interface sage_energy_monitor
echo stack-up
