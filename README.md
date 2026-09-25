<div align="center">

# 🛰️ SAGE-UAV

### Semantic AI-Guided Exploration & Active Search for autonomous drones

**Type one sentence. The drone finds the people, tells you where they are, and lands itself.**

![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white)
![PX4](https://img.shields.io/badge/PX4-SITL-1d76d2)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange)
![Python](https://img.shields.io/badge/Python-3.10-3776AB?logo=python&logoColor=white)
![YOLO](https://img.shields.io/badge/Perception-YOLO-00b4d8)
![Status](https://img.shields.io/badge/status-simulation%20research-success)
![License](https://img.shields.io/badge/license-MIT-lightgrey)

<img src="docs/media/demo_preview.gif" alt="SAGE-UAV demo: onboard camera and the view from above the drone" width="100%">

<sub>Left: onboard camera with YOLO detections and the live mission map. Right: Gazebo view from a camera mounted above the drone.
Same flight, 2x real speed. Full videos: <a href="demo/sage_uav_camera_view.mp4">camera view</a> · <a href="demo/sage_uav_drone_top_view.mp4">drone top view</a>.</sub>

</div>

---

## ✨ What it does

```text
"Find all people in this area and report their locations"
        │
        ▼
 understand ─▶ search ─▶ detect ─▶ localize in 3D ─▶ remember ─▶ verify ─▶ report ─▶ return ─▶ land
```

SAGE-UAV is a complete autonomy stack for a quadrotor, built and evaluated in simulation (ROS 2 Humble, PX4 SITL, Gazebo):

| stage | what happens |
|---|---|
| **Mission understanding** | Free text becomes a validated JSON spec. A rule-based parser works offline; an optional Claude front end can fill the same fields. Unsupported requests (for example "red vehicles") are rejected with a reason. |
| **Perception** | Onboard camera + YOLO person detector (5 Hz). |
| **3D localization** | Image ray + time-aligned PX4 pose and attitude give a ground position, about **0.3 m** median error. |
| **Semantic memory** | Tracks with motion state and persistent IDs; fragmented and duplicate tracks are merged. |
| **Active search** | Serpentine coverage sweep with a 360° scan at every waypoint; a candidate is re-observed until it accumulates enough live evidence to count as a person. |
| **Safety** | Every viewpoint is validated (obstacle map, step limit, altitude); an energy monitor triggers return-home; a UAV-lost guard ends the mission if the drone leaves the area; PX4 holds on offboard loss. |
| **Report** | JSON with every person found, position, confidence, and mission time. |

<div align="center">
<img src="docs/media/architecture.png" alt="SAGE-UAV architecture" width="92%">
</div>

<details>
<summary><b>Example report</b></summary>

```json
{
  "mission": "Find all people in this area and report their locations",
  "status": "area_covered",
  "count": 4,
  "found": [
    {"id": 1,  "x": 0.1,  "y": 4.8,  "confidence": 0.81},
    {"id": 3,  "x": -8.4, "y": 9.5,  "confidence": 0.79},
    {"id": 6,  "x": 4.0,  "y": 1.4,  "confidence": 0.77},
    {"id": 10, "x": 9.9,  "y": -9.0, "confidence": 0.84}
  ],
  "duration_s": 706.0
}
```
</details>

---

## 🎥 Demo

<div align="center">
<img src="docs/media/collage.png" alt="Frames from the demo flight" width="92%">
<br><sub>Search, first detection, and verified people from the demo flight (4 of 4 found, no false reports, 706 s).</sub>
</div>

The scene ([`worlds/sage_rescue.sdf`](worlds/)) is a small town: roads, houses, trees, cars, a helipad, three standing people and one person walking a 12 m path.

---

## 📊 Results (simulation, scored automatically against ground truth)

| world | runs | people found | precision | mean position error | mission time (median) |
|---|---|---|---|---|---|
| `sage_sar`: 3 standing people | 5 | **100%** | **100%** | **0.30 m** | 242 s |
| `sage_rescue`: 3 standing + 1 walking, rich scene | 7 | 89% | 89% | 0.46 m | 630 s |
| `sage_hard`: 3 standing + 2 walking, obstacles | 4 | 100% | 91% | 0.43 m | 895 s |

<div align="center"><img src="docs/media/results.png" alt="Results overview" width="92%"></div>

Standing people are located reliably; a person **walking** through the scene is found in most runs and occasionally reported at two points on its path.
Per-run tables, protocol and notes: [`docs/results.md`](docs/results.md), [`docs/EVALUATION.md`](docs/EVALUATION.md).

---

## 🧪 Highlights

- **Measured, not assumed.** [`scripts/run_trials.sh`](scripts/run_trials.sh) repeats a full mission, saves every log, and
  [`scripts/score_trial.py`](scripts/score_trial.py) matches reports to ground truth.
- **Independent safety layer.** Every planner viewpoint is checked by a separate node before it reaches PX4; energy, obstacle and UAV-lost guards are
  built in.
- **Reproducible.** Pinned versions, PX4 patches, setup scripts and the YOLO weights are in [`environment/`](environment/README.md); 29 unit tests run
  without a simulator.
- **Filmed without slowing the simulator.** A light logger records the flight; both demo videos are rendered afterwards
  ([`scripts/record_raw.py`](scripts/record_raw.py), [`scripts/render_demo.py`](scripts/render_demo.py)).

---

## 🚀 Quick start

Requirements: Ubuntu 22.04, ROS 2 Humble, PX4-Autopilot v1.16.0 (SITL, `gz_x500_mono_cam`), Gazebo Harmonic, Micro XRCE-DDS Agent v2.4.3, `px4_msgs` v1.16.2,
and a Python environment with `ultralytics` for YOLO. **Exact versions, patches, package lists and step-by-step setup: [`environment/`](environment/README.md).**

```bash
# 1. build the ROS 2 package (in a workspace that also contains px4_msgs)
colcon build --packages-select sage_px4_interface

# 2. install the worlds and the drone model with the extra camera link
cp worlds/*.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/
cp models/x500_mono_cam/model.sdf ~/PX4-Autopilot/Tools/simulation/gz/models/x500_mono_cam/

# 3. start the simulation stack (health-gated, retries automatically) and the mission nodes
SAGE_WORLD=sage_rescue scripts/stack_verified.sh 1800
SAGE_WORLD=sage_rescue scripts/start_mission_nodes.sh

# 4. give it a mission
ros2 topic pub --once /sage/mission/command std_msgs/msg/String \
  "{data: 'Find all people in this area and report their locations'}"
```

Repeat and score: `scripts/run_trials.sh 5 sage_rescue`. Film a flight: `SAGE_RECORD=1 scripts/run_trials.sh 1 sage_rescue`, then
`python3 scripts/render_demo.py results/raw/<run> --planner-log results/logs/<run>/planner.log`.

---

## 🗂️ Repository map

| path | contents |
|---|---|
| `src/sage_px4_interface/` | the ROS 2 package: nodes, mission parser, planner, tests |
| `worlds/`, `config/`, `models/` | Gazebo worlds and their generator, per-world truth/area/obstacle files, the drone model, YOLO weights |
| `scripts/` | stack start, trial runner, scoring, video recorder/renderer, figure generation |
| `results/` | trial tables, saved logs, figures |
| `demo/` | the two demo videos, pictures, post text ([details](demo/README.md)) |
| `environment/` | pinned versions, PX4 patches, workspace + vision-env setup, package lists |
| `docs/` | [index](docs/README.md): specification, architecture, usage, worlds, evaluation, roadmap, design notes, engineering log |

## ℹ️ Scope

Simulation research system (PX4 SITL + Gazebo, `person` class, known static obstacles). Assumptions: [`docs/SCOPE.md`](docs/SCOPE.md).

## 🛣️ Roadmap

Active identity check for walking people · shorter missions · controlled experiments (random vs intelligent search, energy-aware planning) ·
hardware-in-the-loop. Details: [`docs/ROADMAP.md`](docs/ROADMAP.md).

## 📄 License

MIT, see [`LICENSE`](LICENSE).
