# sage_px4_interface

ROS 2 (Humble) Python package of SAGE-UAV: perception, 3D localization, semantic world model, active planner, mission manager, energy monitor and
PX4 offboard control. Architecture, topics and parameters: [`docs/ARCHITECTURE.md`](../../docs/ARCHITECTURE.md).

```bash
colcon build --packages-select sage_px4_interface      # in a workspace that also contains px4_msgs v1.16.2
ros2 run sage_px4_interface sage_viewpoint_planner --ros-args -p area:="[-12.0,12.0,-12.0,12.0]"
```
Executables: `sage_mission_parser`, `sage_target_localizer`, `sage_semantic_world_model`, `sage_viewpoint_planner`, `sage_mission_manager`,
`sage_energy_monitor`, `offboard_position_node`, `camera_test`. The YOLO node is started through `yolo_detector_launcher.py` in the vision environment.
Unit tests: `test/`.
