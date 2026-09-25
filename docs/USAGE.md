# Usage

Setup of the software stack: [`../environment/README.md`](../environment/README.md).

## Start a mission
```bash
cd ~/SAGE-UAV
SAGE_WORLD=sage_rescue scripts/stack_verified.sh 1800       # PX4 + Gazebo + bridges + perception + world model + energy monitor (health-gated)
SAGE_WORLD=sage_rescue scripts/start_mission_nodes.sh       # parser + mission manager + planner (reads config/sage_rescue.env)
ros2 topic pub --once /sage/mission/command std_msgs/msg/String \
  "{data: 'Find all people in this area and report their locations'}"
```
Progress is printed by the planner (`/tmp/sage_logs/planner.log`): `MISSION ACCEPTED`, `COVERAGE WAYPOINT`, `TARGET VERIFIED`, `MISSION COMPLETE`,
then `RETURN HOME`, `HOME REACHED`, landing and disarm. The report is published on `/sage/mission/report`.

`stack_verified.sh` starts the stack, checks that the UAV hovers near the origin and that the simulation runs at a real-time factor above 0.5,
and retries up to 5 times. Stop everything with `scripts/killall.sh`.

## Mission language
| example | result |
|---|---|
| `Find all people in this area and report their locations` | search the whole area, report every person (`quantity: all`) |
| `Find 2 people and report their location` | stop as soon as 2 are verified (`quantity: 2`) |
| `Find all people and count them` | `output: count` |
| `Find all red vehicles` | parsed, marked **unsupported** (only `person` is detected) and rejected with a reason |

The rule-based parser needs no network. If `ANTHROPIC_API_KEY` is set, a Claude front end fills the same fields first and falls back to the rules on
any error (`use_llm`, `SAGE_LLM_MODEL`). The key is only read from the environment and is never stored.

## Configuration
- **World settings:** `config/<world>.env` (search area, known obstacles, battery drain, extra planner arguments) is read by `start_mission_nodes.sh`.
- **Parameters:** every threshold is a ROS 2 parameter; see the table in [`ARCHITECTURE.md`](ARCHITECTURE.md). Override with
  `ros2 run ... --ros-args -p name:=value`.
- **Environment variables:** `SAGE_WORLD` (world name), `SAGE_YOLO_HZ` (detector rate, default 5), `SAGE_RECORD=1` (light logger + annotated frames
  for filming), `SAGE_GUI=1` (show the Gazebo window; slows the simulation), `ATT=gz_truth` (evaluation-only attitude source).

## Scripts
| script | purpose |
|---|---|
| `stack.sh`, `stack_verified.sh` | full stack start, health-gated start with retries |
| `start_mission_nodes.sh`, `restart_*.sh`, `killall.sh` | mission nodes and process control |
| `run_trials.sh N world` | repeat a full mission N times, save logs to `results/logs/<id>/`, append a scored row to `results/trials_<world>.csv` |
| `score_trial.py` | score one planner log against `config/truth_<world>.json` |
| `record_raw.py`, `render_demo.py`, `finalize_demo.py`, `make_collage.py` | film a flight and render the two demo videos and pictures |
| `make_figures.py` | architecture diagram, results charts, summary table |
| `tools/` | analysis helpers (motion windows, camera frame grab, MP4 fast-start) |

## Tests
```bash
cd src/sage_px4_interface && source /opt/ros/humble/setup.bash
PYTHONPATH=. python3 -m pytest test/test_mission_parser.py test/test_mission_llm.py test/test_obstacle_map.py test/test_identity_resolution.py -q
```
(29 tests; no simulator needed.)
