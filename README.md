# SAGE-UAV

**Semantic AI-Guided Exploration and Active Search for Autonomous UAVs**

An autonomous UAV that interprets a high-level mission ("Find all people and report their locations"), searches an environment, detects and localizes targets with computer vision, keeps a semantic world model, picks informative viewpoints, and searches in an energy- and safety-aware way. ROS 2 Humble + PX4 SITL + Gazebo.

## Pipeline

```
mission text -> mission parser (Claude LLM or rules) -> /sage/mission/spec
camera -> YOLO -> 3D target localizer -> semantic world model
      -> viewpoint planner (active vision, energy-aware) -> mission manager (safety checks)
      -> offboard node -> PX4        energy monitor -> abort / return home / land
mission completion -> /sage/mission/report (JSON: what was found, where)
```

## Layout

- `src/sage_px4_interface/` ROS 2 package (all nodes, unit tests in `test/`)
- `worlds/sage_test.sdf` Gazebo world (copy into `PX4-Autopilot/Tools/simulation/gz/worlds/`)
- `scripts/` helper scripts (`stack.sh` full restart, `start_mission_nodes.sh`, ...)
- `docs/` roadmap (`steps.md`), progress log (`progress.md`), specification
- `results/` recorded data and analysis

## Run (summary)

1. PX4 SITL: model `gz_x500_mono_cam`, world `sage_test`; Micro XRCE-DDS agent on UDP 8888.
2. `colcon build --packages-select sage_px4_interface` in a workspace that also contains `px4_msgs`.
3. `scripts/stack.sh` brings up the stack; `scripts/start_mission_nodes.sh` starts parser, mission manager and planner.
4. Send a mission:
   `ros2 topic pub --once /sage/mission/command std_msgs/msg/String "{data: 'Find all people'}"`

## Mission understanding (optional LLM)

The parser uses the Claude API when `ANTHROPIC_API_KEY` is set in the environment, and falls back to a rule-based parser otherwise. The key is never stored in the repository. Set `SAGE_LLM_MODEL` to override the model.

## Status

See `docs/progress.md`. Simulation only; the perception stack detects `person` (COCO class 0).
