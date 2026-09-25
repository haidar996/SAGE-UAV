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
# Fresh PX4 parameters every boot: a saved parameter file accumulated a bad
# learned magnetometer offset (CAL_MAG0_ZOFF 0.26 G on a 0.49 G field) that made
# the EKF flag the compass as disturbed (yaw never aligned, random heading
# error, preflight failures).
rm -f $B/parameters.bson $B/parameters_backup.bson
cd $B && HEADLESS=1 PX4_GZ_WORLD=$SAGE_WORLD PX4_SYS_AUTOSTART=4019 PX4_SIM_MODEL=gz_x500_mono_cam run px4 ../bin/px4 -d ../etc
cd ~/Micro-XRCE-DDS-Agent/build && run xrce ./MicroXRCEAgent udp4 -p 8888
wait_for(){ pat=$1; limit=$2; for i in $(seq 1 $limit); do grep -q "$pat" $S/px4.log 2>/dev/null && return 0; sleep 2; done; return 1; }
wait_for "Startup script returned" 60 || { echo "STARTUP_TIMEOUT"; exit 2; }
cd $B; ../bin/px4-param set NAV_DLL_ACT 0 >/dev/null; ../bin/px4-param set MPC_XY_VEL_MAX 3.5 >/dev/null; for kv in COM_ARM_MAG_STR:0 COM_ARM_MAG_ANG:180 COM_ARM_IMU_ACC:2 COM_ARM_IMU_GYR:1; do ../bin/px4-param set ${kv%%:*} ${kv##*:} >/dev/null; done; ../bin/px4-param set MPC_TILTMAX_AIR 30 >/dev/null; ../bin/px4-param set COM_OF_LOSS_T 2 >/dev/null; ../bin/px4-param set COM_OBL_RC_ACT 5 >/dev/null; ../bin/px4-param set SIM_BAT_DRAIN $DRAIN >/dev/null; ../bin/px4-param set SIM_BAT_MIN_PCT 0 >/dev/null
wait_for "Ready for takeoff" 60 || { echo "PREFLIGHT_NOT_READY: $(grep -E 'Preflight Fail' $S/px4.log | tail -1)"; exit 3; }
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
# Localizer attitude source: px4 (default, the PX4 estimate) or gz_truth
# (Gazebo ground truth, for evaluation): ATT=gz_truth.
run bridge_pose ros2 run ros_gz_bridge parameter_bridge "/world/$SAGE_WORLD/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"
run localizer ros2 run sage_px4_interface sage_target_localizer --ros-args -p attitude_source:=${ATT:-px4}
run world ros2 run sage_px4_interface sage_semantic_world_model
run energy ros2 run sage_px4_interface sage_energy_monitor
echo stack-up
