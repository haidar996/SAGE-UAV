# Project specification

## Purpose
SAGE-UAV (Semantic AI-Guided Exploration & Active Search) is an autonomy stack that lets a quadrotor carry out a search mission described in
plain language, for example *"Find all people in this area and report their locations"*. The system converts the sentence into a validated
mission, searches the area, detects and localizes targets in 3D, confirms each one with repeated observations, manages its own battery and
safety, reports what it found, and returns home and lands.

## Goals
1. **Mission-driven operation.** One natural-language command starts a complete autonomous mission.
2. **Trustworthy findings.** A target is only reported after live evidence accumulates from several viewpoints, and every report carries a position
   and a confidence.
3. **Safe by construction.** Nothing the planner produces reaches the flight controller without passing an independent safety layer.
4. **Measurable.** Every mission is scored automatically against ground truth, so changes can be judged with numbers.
5. **Reproducible.** Fixed worlds, fixed ground truth, pinned software versions, one-command trial runner.

## Scope
| in scope | out of scope (for now) |
|---|---|
| PX4 SITL + Gazebo quadrotor (x500, monocular camera) | real-hardware flight |
| detection and localization of the `person` class | other object classes and attributes such as colour (parsed, reported as unsupported) |
| static and walking people, known static obstacles | sensing of unknown obstacles |
| single-UAV area search with return-home and landing | multi-UAV coordination |

## Functional requirements
| id | requirement | implemented by |
|---|---|---|
| F1 | accept a free-text mission and produce a structured spec; reject unsupported missions with a reason | `sage_mission_parser` (+ optional LLM front end) |
| F2 | detect people in the onboard camera stream | `yolo_detector` |
| F3 | estimate each person's ground position from a detection | `sage_target_localizer` |
| F4 | keep persistent tracks, motion state and merge duplicates | `sage_semantic_world_model` |
| F5 | sweep the search area and re-observe candidates until confirmed | `sage_viewpoint_planner` |
| F6 | validate every viewpoint before flight (obstacles, step size, altitude) | `sage_mission_manager` |
| F7 | monitor battery, compute return cost, trigger return-home | `sage_energy_monitor` |
| F8 | arm, fly offboard setpoints, land and disarm | `offboard_position_node` |
| F9 | publish a JSON report at mission end | `sage_viewpoint_planner` |

## Non-functional requirements
- Runs on one 4-core CPU without GPU (YOLO capped at 5 Hz; simulation kept near real time).
- Every node is a ROS 2 Humble Python node with parameters for all thresholds.
- Pure decision logic (mission parsing, obstacle routing, identity resolution) is unit-tested without a simulator.

## Success criteria (evaluation)
Recall, precision and mean position error against ground truth, mission time, and mission completion rate, measured over repeated runs in each
world (see [`EVALUATION.md`](EVALUATION.md)).
