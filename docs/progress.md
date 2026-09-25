# SAGE-UAV progress log

## 2026-09-23 — restart baseline + 18.3a
- Headless PX4 (x500_mono_cam, world sage_test) + XRCE agent + ROS2 chain restarted from zero.
- Arming was blocked by "No connection to ground control station"; sim param NAV_DLL_ACT set to 0.
- Baseline OK: arming_state=2, nav_state=14, preflight pass, hover z~-2 m (small ~0.3 m XY drift, accel-bias warning).
- Full loop verified: YOLO -> localizer -> world model -> planner -> mission manager -> offboard. Viewpoints 1->4 reached, search cycle wraps.
- 18.3a: planner PERSON OBSERVATION log now `info`, throttled 1 Hz (was `debug`). Measurement only, no decision change.
- Observed image_x range so far: 0.046 .. 0.60 (targets at frame edge do occur). Next: 18.3b analyze samples.

## 18.3b — image-position analysis (318 samples, results/observations_18_3b.csv, scripts/analyze_observations.py)
- img_x: min 0.03, p10 0.13, median 0.56, p90 0.80, max 0.96. img_y: median 0.66 (person sits in lower half; min 0.26, max 0.95).
- 243/318 samples pass confidence>=0.6 and bbox>=1%. Requiring center >=0.10 from every edge keeps 219; >=0.15 keeps 193.
- Edge samples (img_x <0.10 from edge) are NOT lower confidence (0.69 vs 0.70 centre) and have larger bbox: edge position alone does not predict weak detections here.
- Candidate 18.3c criterion: margin from image border (start 0.10), ideally on the bbox extent to catch clipped targets. Not yet implemented.

## 18.3c/d — image-border margin criterion: VALIDATED (2026-09-23)
- Planner: `bbox_margin` = min distance of bbox extents to any image border (fraction). `position_ok` = margin >= 0.10; 4th condition next to fresh/confidence/bbox-size. Logged in all OBSERVATION lines.
- Observed: HOLD at margin 0.166 / 0.231; QUALITY LOST with conf 0.824, bbox 2.9% but margin 0.097 (position_ok=False only); INSUFFICIENT at margin 0.022. Search continued across viewpoints, 0 Mission Manager rejections, no tracebacks.
- Known issue: Mission Manager keeps last_accepted_viewpoint forever. Restarting only the planner makes its first viewpoint a >3 m "jump" -> rejected forever (deadlock). Workaround: restart Mission Manager first, then planner. Proper fix (reset on stale) not done.

## Mission Manager stale-viewpoint fix: VALIDATED (2026-09-23)
- sage_mission_manager.py: if no viewpoint accepted for 3 s (`last_accepted_timeout`), last_accepted_viewpoint is cleared ("jump check reset"). All other checks (frame, altitude, <=8 m from UAV, quaternion) unchanged.
- Test: planner-only restart mid-flight -> 1 stale reset, 0 rejections, viewpoints accepted again (323 -> 554), no tracebacks.
- Remaining limitation: a freshly started planner has no target until one is visible; if the UAV is holding where the person is out of view, nothing happens. Recovery: send UAV to safe hover (restart offboard node -> (0,0,-2)).

## 20a — energy monitor (measurement only) (2026-09-23)
- New node `sage_energy_monitor` (registered in setup.py). Reads /fmu/out/battery_status + local position; logs ENERGY line at 1 Hz: remaining, voltage, smoothed drain rate (30 s window), home distance, return time (1.0 m/s + 0.5 m/s descent), return cost, margin_after_return (reserve 20 %). No effect on flight.
- Finding: sim battery is a stub (SIM_BAT_DRAIN=60 s, SIM_BAT_MIN_PCT=50, current_a=-1); `remaining` is stuck at 50.0 %, so drain rate reads 0. Node itself verified working.
- Next: restart PX4 with realistic drain (e.g. SIM_BAT_DRAIN ~600 s, MIN_PCT 0) to get real data (20b), then analysis, then the abort/return-home rule (20c), then validation (20d).

## 20b — real energy data (2026-09-23)
- Full stack restarted with `stack.sh` (scratchpad): SIM_BAT_DRAIN=600, SIM_BAT_MIN_PCT=0, NAV_DLL_ACT=0 set after PX4 startup.
- Battery now drains: 100 -> ~78.5 % in ~140 s of ground+flight, rate ~0.12-0.14 %/s, voltage 16.08 -> 15.81 V. Rate is nearly identical on the ground and while searching -> sim drain is time-based, not load-based. Energy model for the sim = drain_rate x time (matches monitor).
- Return estimate at 5.5 m from home: ~9 s, ~1.2 % cost; margin_after_return 57 % (reserve 20 %).
- Planner/Mission Manager unaffected (0 rejections).
- Next 20c: when margin_after_return < 0 (or remaining < reserve+cost) -> abort search -> return home -> land.

## 20c/d — energy abort -> return home -> land -> disarm: VALIDATED (2026-09-23)
- sage_energy_monitor: ROS param `reserve_fraction` (default 0.20). Latched flag /sage/energy/return_home (std_msgs/Bool) when remaining - drain_rate*return_time - reserve < 0.
- sage_mission_manager: on flag, ignores planner viewpoints ("RETURN HOME ACTIVE"), flies to home (0,0,-2) in 1.5 m steps from the UAV's actual position, at <=0.4 m publishes /sage/mission/land_request ("HOME REACHED").
- offboard_position_node: on land_request sends NAV_LAND; PX4 land detector never fires in this SITL (landed=false while physically on ground, gz z=-0.01), so after 3 s within 0.15 m of ground it sends force-disarm (param2=21196). Normal disarm is denied ("not landed").
- Test (reserve temporarily 0.75 to trigger in ~2 min): EXHAUSTED at remaining 76.5 % -> return home ~13 s -> LAND -> DISARM. arming_state 2 -> 1. 0 rejections during abort.
- Default reserve stays 0.20 (test value only passed via --ros-args). Scripts: scratchpad stack.sh / energy_test.sh.
- Still to do in Step 20: energy-aware viewpoint choice (score with distance/energy cost), abort mid-search resume policy.

## 20e — energy-aware viewpoint selection: VALIDATED (2026-09-23)
- Energy monitor publishes /sage/energy/status = [remaining, drain_rate (NaN until 5 s of data), reserve].
- Planner param `energy_aware` (default True; False = old fixed order 1->2->3->4 for A/B, Experiment C). After an insufficient observation it marks the viewpoint visited and picks the nearest unvisited affordable one; affordable = remaining - drain_rate*(travel + return_from_viewpoint + 4 s landing) >= reserve. Cycle resets when all 4 visited. Log: ENERGY-AWARE SELECTION | chosen | distance | est_cost_incl_return | unaffordable_skipped.
- Observed: order adapts (3,2,1 then 2,3,4), est cost 1.1-1.8 % incl. return. With reserve set just under remaining: "NO AFFORDABLE VIEWPOINT | holding position" then EXHAUSTED -> RETURN HOME ACTIVE. 0 rejections, 0 tracebacks.
- Caveat: while drain rate is unknown (first ~5 s) cost is n/a and nothing is skipped.
- Caveat: geometry is 4 viewpoints on a 3 m circle, so nearest-first saves little vs fixed order (hops 4.2-6 m). Real gain needs more candidates/regions (Step 17/21).
- Caveat: in this run nearly all evaluations had position_ok=False despite conf 0.6-0.86 and bbox 3-6 % -> HOLD is rare with the 0.10 border margin. Threshold/camera pitch worth revisiting.

## Reliability fixes made along the way
- offboard_position_node retries ARM every 2 s until armed once (never after LAND request).
- stack.sh now arms first and starts perception afterwards: CPU load (YOLO ~165 %, load avg 12) was starving the SITL IMU -> "High Gyro/Accel Bias" preflight failures.

## 18.3e — border margin re-tuned 0.10 -> 0.02 (2026-09-23, built, NOT yet validated live)
- Data (130 samples, conf>=0.6 & bbox>=1 %): 56 samples at margin <= 0.005 (bbox touches border: person's feet clipped at bottom, mean img_y 0.81), only 5 between 0.005-0.03, then continuous. 0.10 kept 39 % of good samples, 0.02 keeps 60 %. 0.02 sits in the gap: rejects clipped views, keeps close-to-edge ones.
- Also added: offboard node retries OFFBOARD request every 2 s (request could be lost if DDS link not up).
- Helper scripts now in ~/SAGE-UAV/scripts (logs go to /tmp/sage_logs): stack.sh [drain_s] (full restart, arm first), restart_both.sh (Mission Manager + planner), restart_offboard.sh, start_planner.sh, energy_test.sh [reserve], killall.sh.
- Known quirk: after a planner restart the offboard node keeps holding the last viewpoint; if the person is out of view nothing is detected -> restart offboard node (goes to (0,0,-2)) then planner.

## 18.3e — margin 0.02 VALIDATED (2026-09-23)
- 170 s energy-aware search run: 12 OBSERVATION SUFFICIENT (HOLD reachable again; with 0.10 it was almost never), 37 INSUFFICIENT, 12 QUALITY LOST. 20/37 insufficient evaluations failed on position_ok alone (conf & bbox fine) — those had bbox_margin ~0.000-0.001 i.e. genuinely clipped, so the rejection is correct. 0 tracebacks, 0 Mission Manager rejections.
- Remaining insight: at viewpoints where the person is clipped at the bottom, the geometry (3 m standoff, 2 m altitude, fixed camera pitch) is the cause; a higher altitude or larger standoff would fix clipping. Candidate tuning for later.

## 19 stage 1 — rule-based mission parser: VALIDATED (2026-09-23)
- New node `sage_mission_parser` (sage_mission_parser.py, registered in setup.py). /sage/mission/command (std_msgs/String, free text) -> /sage/mission/spec (std_msgs/String JSON, transient-local): {text, action, target_class, attribute, quantity ('all'|int), output ('locations'|'count'), valid, supported, reason}. Pure function `parse_mission()` is unit-tested: test/test_mission_parser.py, 7 passed (run: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=.:$PYTHONPATH python3 -m pytest test/test_mission_parser.py; note the ROS env's typeguard pytest plugin is broken, hence the env var).
- Perception only detects `person` (yolo_detector hard-wired to COCO class 0) -> SUPPORTED_CLASSES={'person'}; other classes parse but are marked unsupported with a reason.
- Planner: param `require_mission` (default True). Without an accepted mission it stays idle. New mission resets search state. Logs MISSION ACCEPTED / MISSION REJECTED.
- Live test: idle with known target -> 0 viewpoints accepted; "Find all red vehicles..." -> parser NOT EXECUTABLE + planner MISSION REJECTED (idle stays); "Find all people in this area and report their locations" -> PARSED, MISSION ACCEPTED, search starts (307 viewpoints accepted in 60 s, HOLD/energy-aware selection working). 0 tracebacks.
- Not yet acted upon: `quantity` (all/N -> stop condition), `attribute` (colour, needs perception), `output` (report format). Stored and logged only.
- Scripts: ~/SAGE-UAV/scripts/start_mission_nodes.sh (parser + Mission Manager + planner). Send a mission: ros2 topic pub --once /sage/mission/command std_msgs/msg/String "{data: 'Find all people'}"
- Next: stage 2 = LLM front end producing the same spec (needs API key decision), and acting on quantity/output (mission completion + report).

## 19 stage 1b — mission completion + report: VALIDATED for quantity=N (2026-09-24)
- Code (already in sage_viewpoint_planner.py, written after the parser entry): per-target evidence accumulation (2 s of sufficient observations, resets after 5 s gap) -> TARGET VERIFIED; verified targets are skipped, next nearest unverified one is searched. `complete_mission(reason)` publishes JSON report on the report topic {mission, status, quantity_requested, output, count, found[id,x,y,confidence,viewpoint], duration_s}, then latches return-home. Reasons: quantity_reached, all_tracked_verified, timeout (`mission_timeout_s`, default 300), energy_abort.
- Live test ("Find 1 person and report their location"): MISSION ACCEPTED -> TARGET VERIFIED id=1 at (-0.17, 1.49), conf 0.818 -> MISSION COMPLETE (quantity_reached, 5 s) -> RETURN HOME -> HOME REACHED -> LAND -> DISARM. 0 tracebacks across all nodes.
- Not yet tested: quantity='all' (completes only when every tracked target is verified, needs >1 person in world), timeout path, verified position vs ground truth (person's true world position not compared).
- Minor: "New target acquired" still logged once right after completion (harmless, mission inactive).
- Next: test 'all' with 2+ people (Step 21 world), then stage 2 LLM front end (needs API key decision).

## PROJECT STATE AUDIT (2026-09-24, full scan of files)
Layout (differs from what the log implied):
- ALL code lives in ~/sage_ws/src/sage_px4_interface/sage_px4_interface/ (ROS2 Humble package; px4_msgs v1.16 alongside). ~/SAGE-UAV/{src,config,models,worlds,experiments} are EMPTY, README.md is empty; ~/SAGE-UAV only holds docs/, scripts/, results/. No git repo anywhere (no version history/backup!).
- World file is NOT in the project: ~/PX4-Autopilot/Tools/simulation/gz/worlds/sage_test.sdf (last edited 2026-09-21). Contents: ground plane + ONE static animated actor "person" at gz (4, 0, 1.0). No buildings, vehicles or other people. Model: gz_x500_mono_cam, autostart 4019.
- YOLO weights yolo26n.pt exist in 3 copies (~, ~/sage_ws, package dir); yolo runs in ~/sage_vision_env. bus.jpg is a leftover test image.
- Nodes (setup.py): offboard_position_node, camera_test, sage_target_localizer, sage_semantic_world_model (877 l), sage_viewpoint_planner (1541 l), sage_mission_manager, sage_energy_monitor, sage_mission_parser; yolo_detector via yolo_detector_launcher.py. Tests: only test/test_mission_parser.py (+ default flake8/pep257 stubs).
- Pipeline topics: yolo -> /sage/perception/detections -> localizer -> /sage/perception/target_position -> world model -> /sage/world_model/targets -> planner -> /sage/planning/viewpoint -> mission manager -> /sage/mission/validated_viewpoint -> offboard node -> /fmu/in/*. Also /sage/mission/{command,spec,report,return_home,land_request}, /sage/energy/{status,return_home}.

Roadmap status vs steps.md (its colour markers are stale; steps 11-19 still show yellow/orange/red):
- Steps 0-16: done (env, PX4, offboard nav, camera, YOLO, tracking, 3D localization, semantic map + motion classification/memory).
- Step 17 active search + Step 18 active vision (18.3a-e): done and validated.
- Step 19: stage 1 (rule parser) + completion/report done. Stage 2 (LLM front end) NOT started. Attribute (colour) and non-person classes unsupported.
- Step 20 energy-aware: done (monitor, abort/return/land, energy-aware viewpoints). Sim battery is time-based only.
- Steps 21-25: NOT started (search & rescue world, robustness, experiments A/B/C, results, demo). experiments/ and results/ (only one CSV) are empty of real work.

Open issues found in this audit:
1. LOCALIZATION ACCURACY (see 'Localization check' below; original note follows): person's true position is gz (4,0) = NED ~(0, 4); the verified report said (-0.17, 1.49), ~2.5 m off, and localizer output swings (e.g. (2.1,7.2)->(2.4,6.9) while moving). No ground-truth comparison has ever been done; a "found: locations" report is only as good as this. Needs a ground-truth error study (use gz pose of actor, static UAV at several viewpoints) before Step 21/experiments.
2. World too simple for Steps 21+ (single static person). Need multi-person + distractors (vehicles/houses) to test quantity='all' and false detections.
3. No git / backup; world file lives outside the project. Copy world into ~/SAGE-UAV/worlds and `git init`.
4. Sim quirks still needed: NAV_DLL_ACT=0, force-disarm after LAND (land detector never fires).

## 19 stage 2 — Claude LLM mission parser: BUILT, live API call NOT yet tested (2026-09-24)
- New module sage_mission_llm.py: `llm_parse_mission(text, client)` calls Claude (default model claude-opus-5, override env SAGE_LLM_MODEL; effort low, 15 s timeout, 1 retry) with a fixed system prompt + JSON-schema structured output, and returns the same spec dict as the rule parser (plus `source`: 'llm'|'rules'). `supported` is computed in code from SUPPORTED_CLASSES, never trusted from the model; bad/invalid output raises LLMParseError.
- sage_mission_parser node now tries the LLM first (param `use_llm`, default True) and falls back to the rule parser on any error or if no ANTHROPIC_API_KEY / SDK. Startup log says "Claude LLM with rule-based fallback" or "rule-based only". Command text is passed as the user message only (prompt-injection: LLM can only fill spec fields, all safety checks downstream unchanged).
- SDK: `pip install --user anthropic` (1.8.0, system python3.10 used by ROS). Key must be exported as ANTHROPIC_API_KEY in the shell that launches the parser (start_mission_nodes.sh); never commit it.
- Tests: test/test_mission_llm.py (6, mocked fields, no network) + parser tests = 13 passed. Live: without a key the node starts "rule-based only" and parses "Find all people" (quantity=all) correctly.
- TODO: run with a real key on free-form commands, compare LLM vs rules on a set of phrasings.

## 19 stage 2b — Ollama (local, keyless) backend added (2026-09-24)
- No API key available -> added local backend in sage_mission_llm.py: `ollama_parse_mission` (HTTP to localhost:11434, schema-constrained JSON, temperature 0, keep_alive=0 so the model unloads right after each command and does not load the sim; default model qwen2.5:3b, env SAGE_OLLAMA_MODEL). `make_parser()` picks backend via env SAGE_LLM_BACKEND=claude|ollama|auto (auto: Claude if ANTHROPIC_API_KEY set, else Ollama if server up, else rules). Parser node logs which backend it uses; spec `source` = llm|ollama|rules. Backend is chosen at node startup.
- Tests: 15 mission tests pass (mocked). Ollama itself NOT yet installed on this machine (no GPU, 7 GB RAM, ~9 GB disk free): install + `ollama pull qwen2.5:3b` then live-test.

## Housekeeping done (2026-09-24)
- Package moved into the project: ~/SAGE-UAV/src/sage_px4_interface (git-tracked); ~/sage_ws/src/sage_px4_interface is now a SYMLINK to it, colcon build verified. World copied to ~/SAGE-UAV/worlds/sage_test.sdf (the PX4 copy is still what the sim loads; keep them in sync). README.md written, .gitignore added (*.pt, build dirs, env files), `git init` on branch main + initial commit (local repo identity GitHub noreply identity; NOT pushed). Secret scan of tracked files: clean. Removed leftover bus.jpg (ultralytics sample image). docs/review.md (77 KB chat transcript) and the file `SAGE-UAV` (original plan paste) are tracked: decide whether to publish them.
- Ollama attempt abandoned and fully removed (user will use the Claude API key instead).

## Localization check (2026-09-24, partial)
- Measured: 675 localizer samples over 70 s vs assumed truth NED (0, 4) [actor at gz (4,0)]: mean (N -1.74, E 3.45), std (0.47, 0.29), error median 1.82 m / p90 2.46 / max 2.88. So precision is OK (~0.3-0.5 m) but there is a consistent ~1.8 m offset (mostly lateral/north).
- Ground truth is UNCERTAIN: the actor does not appear in gz pose/info or dynamic_pose/info, so its true position (and feet height) is only the SDF waypoint (4,0,1.0). Offset may be truth error, camera/attitude error, or both.
- Likely contributors found: (1) localizer rotates the ray by HEADING only, ignoring roll/pitch, while the hovering UAV visibly tilts (gz quaternion showed ~15 deg roll/pitch) and wanders +-1.5 m around (0,0,-2); (2) sim runs at ~0.72x real time with load average ~12.8 on 4 cores (YOLO + PX4 + gz on one machine), so image, pose and time are skewed; (3) camera body offset assumed (0.12, 0.03, 0.242) fine.
- Next: replace the actor (or add a static box with a known pose) as a measurable target, log gz drone pose+attitude with each detection, add roll/pitch to the projection, re-measure. Reduce CPU load first (lower YOLO rate / image size).

## Localization fix, part 1 (2026-09-24)
- Localizer now uses the FULL attitude quaternion (/fmu/out/vehicle_attitude, volatile QoS: a transient-local subscription never connected) for the ray rotation and camera offset. Param `use_attitude` (default True; False = old heading-only) for A/B. Built; a clean A/B was NOT obtained yet because of the yaw problem below.
- ROOT CAUSE FOUND for the ~1.8 m lateral offset: PX4's yaw estimate disagrees with Gazebo truth by a constant ~28-33 deg (PX4 heading 90 deg / rpy yaw 90 vs gz yaw 27-29 deg ENU = 61-63 deg NED, while the drone hovers still). Confirmed independently with the image: the static person appears at pixel x~940, exactly where a camera 27 deg off-axis predicts (fx~537, cx 640). Position (x,y,z) agrees with gz to ~0.1 m, only yaw is wrong. At 4 m range 28 deg = ~1.9 m lateral error. Present with perception OFF (not a CPU-load effect); sign of the offset differed between two earlier restarts (+33/-28 deg), so it is random per boot.
- EKF flags: cs_mag true but cs_mag_hdg false, cs_mag_3d false, cs_mag_heading_consistent FALSE. Tried at runtime (no effect on heading, all reverted to defaults): EKF2_MAG_TYPE 1, EKF2_DECL_TYPE 0 + EKF2_MAG_DECL -24.6, EKF2_MAG_CHECK 0, EKF2_HDG_GATE 100. World mag field is identical to PX4 default.sdf.
- Consequence: every position derived from PX4 heading (localizer, and any world-frame reasoning) is rotated by the yaw error about the UAV. Hover position itself is fine.
- Ideas not tried: fresh boot with `EKF2_MAG_TYPE=5`(none, yaw from GPS velocity/flight), check the sim gyro/mag sensors in the x500 model, spawn with a defined yaw, or (SITL-only) take yaw from gz ground truth for evaluation. Measurement helpers: /tmp/sync.py (gz vs PX4 pose) and /tmp/loc_sample.py (localizer error) are scratch files, not in the repo.
- Current stack was restarted WITHOUT perception for this test (stack.sh variant); run scripts/stack.sh for a full start.

## Localization fix, part 2: SITL ground-truth attitude (2026-09-24)
- Tried disabling the compass (EKF2_MAG_TYPE=5): PX4 heading still exactly 90 deg while the true yaw drifted (29 -> 24 deg). Reverted to default (0). PX4 yaw estimator in this SITL is not trustworthy; root cause of that still unknown.
- Localizer param `attitude_source`: 'px4' (default in code, real-world path) or 'gz_truth' (SITL only: attitude from Gazebo pose via ros_gz_bridge /world/sage_test/pose/info TFMessage; ENU/FLU -> NED/FRD conversion unit-checked: 27 deg ENU -> 63 deg NED). scripts/stack.sh now starts the pose bridge and runs the localizer with gz_truth (ATT=px4 to override). Position still from PX4 (agrees with gz to ~0.1 m).
- Result, drone hovering steadily (tilt < 2 deg, 655 samples): mean (N 0.66, E 4.15) vs assumed truth (0, 4), std (0.13, 0.16), error median 0.68 m, p90 0.87 m. Before: ~1.8 m median (px4 yaw). Precision is good; remaining 0.7 m is mostly a north offset (0.66) whose cause is unresolved: actor's true position/feet height are unknown (actor not in gz pose topics), or bbox-anchor/camera-model bias. Under tilt/wobble errors were larger (max 6 m) - samples taken while the UAV tilts or moves should be gated (roll/pitch/rate) - TODO.
- For any published result state clearly that localization uses SITL ground-truth attitude.

## Tilt filter + Step 21 world v1 (2026-09-24)
- Localizer: param `max_tilt_deg` (default 8): skips samples when roll/pitch tilt exceeds it (warns at most every 5 s). No skips triggered in the SAR run (drone hovered steadily), so its effect is not yet demonstrated.
- World name is now configurable: env SAGE_WORLD (default sage_test) used by stack.sh, yolo_detector.py and sage_target_localizer.py. New world worlds/sage_sar.sdf (copied to PX4-Autopilot/Tools/simulation/gz/worlds/): 3 people (static actors) at gz (4,0), (7,-6), (-6,5) = NED (0,4), (-6,7), (5,-6); distractors (boxes): house (-8,-7), red car (10,5), blue truck (-2,-10), shed (2,9). Start: `SAGE_WORLD=sage_sar scripts/stack.sh`.
- FIRST RUN in sage_sar with "Find all people in this area and report their locations": MISSION COMPLETE after 4 s, found=2 at (-2.1,3.3) and (-0.7,0.5) -> WRONG: true people are at (0,4), (-6,7), (5,-6); the 2 "found" are stale/noisy world-model tracks. Return home + land + disarm worked. World model had created ids 1,2,3,4,7,8,9 (duplicates) from noisy localizations and one track was classified MOVING (1.3 m/s) although all people are static.
- Design gaps exposed (must fix before Step 21 counts as done):
  1. quantity='all' completes as soon as every CURRENTLY TRACKED target is verified; there is no area-coverage search, so people outside the camera view are never found. Need a search pattern (lawnmower/viewpoint grid over the search area) and a completion criterion (area covered).
  2. Verification uses world-model tracks created before/at mission start; need to reset the world model (or ignore tracks older than the mission) and require multiple consistent observations.
  3. World model track fragmentation/false MOVING from localization noise while the UAV moves (association gate 2 m, EKF yaw offset, timing skew).
  4. Planner viewpoint logic circles ONE target on a 3 m radius; mission manager limits (<=8 m from UAV, 3 m jump) restrict how far the UAV can travel per viewpoint.

## Step 21: coverage search implemented (2026-09-24) - control flow works, result quality NOT yet good
Implemented in sage_viewpoint_planner.py (backup of the pre-change file was /tmp/planner_backup.py, not kept; see git history):
- Coverage sweep: params `area` [x_min,x_max,y_min,y_max] (default -9,9,-9,9 NED), `coverage_spacing` 6 m (3x3 = 9 waypoints, serpentine, start end nearer the UAV), `scan_hold_s` 3 s x 4 headings (0/90/180/270 deg) per waypoint, `coverage_enabled`. With an active mission and no candidate the planner sweeps; 'all' completes with status `area_covered` when all waypoints are scanned (int quantity still completes on `quantity_reached`). Old `all_tracked_verified` completion removed. mission_timeout_s default 900.
- Candidate handling: a world-model person track becomes a candidate only if inside area+1.5 m and not within 2.5 m of a verified person; the existing 4-viewpoint observation runs; evidence now requires the LIVE localized position (/sage/perception/target_position, fresh) to lie within 2 m of the track, and the verified position is the mean of that evidence (was: the stored track position). Duplicates within 2.5 m are merged; unconfirmed candidates are rejected after `candidate_timeout_s` (45 s) and the sweep resumes. Throttled `CANDIDATE STATUS` log every 4 s.
- planner max_step_distance 2.0 -> 0.7 m (carrot speed ~3.5 m/s); stack.sh sets PX4 MPC_XY_VEL_MAX 3.5 and MPC_TILTMAX_AIR 30. YOLO now capped at 5 Hz (env SAGE_YOLO_HZ) - unthrottled YOLO drove the sim real-time factor to 0.14-0.48 and PX4 estimator failures at startup; with the cap RTF = 1.0.
- sage_sar.sdf distractors are now VISUAL-ONLY (no collision): they were taller than the 2 m flight altitude and the UAV flew into them (PX4 'Attitude failure (roll)' -> flyaway of 150 m). No obstacle avoidance exists; do not add collisions again without it.
- Script fixes: killall.sh also kills stale stack.sh instances (a stale stack.sh from a timed-out command made later starts fail with preflight 'High Gyro Bias'); start_mission_nodes.sh is idempotent (a duplicate mission parser published the mission twice); restart_offboard.sh now restarts the node (it only killed it before).
- Results in sage_sar (true people NED (0,4), (-6,7), (5,-6)): person 1 verified repeatedly at (-0.01,3.49), (-0.24,3.64), (-0.46,3.47), i.e. 0.4-0.7 m error. People 2 and 3 were never verified. A run also verified a PHANTOM at (1.59,-0.77) (conf 0.82, 11 evidence points) where no person exists, and many candidates were rejected after 45 s (tracks near (-4.5,0), (-6.9,-0.2), (-5.6,-3.1) also match no person). 0 of 9 coverage waypoints were reached in the last runs because candidates kept pulling the UAV away; the mission did not finish within ~7 min.
- Open problems (perception/localization, not the search logic): (1) unexplained phantom tracks/verifications - need to check what YOLO detects (distractors? shadows? actor pose) and log annotated frames; (2) localization at long range / other headings is much worse than the 0.7 m measured at the origin; gz_truth attitude may lag during yaw scans; (3) world model fragments tracks (ids up to 60+), ids differ per run; (4) hover wander of several metres at the start of some runs (start (4.1,-1.4)); (5) YOLO confidence 0.3-0.8 flickers, bbox >=1% needs <= ~5 m range.

## Perception debugging (2026-09-24) - ROOT CAUSES FOUND
Tools (scratch, /tmp, not in repo): perc_debug.py (projects the known people/boxes into the image with the Gazebo pose at frame time and matches YOLO detections), loc_err.py (localizer output vs nearest true person), pos_sync.py (PX4 vs Gazebo pose, synchronized).
1. **Localizer bug: mirrored left-right axis** (`body_y = -ray_x`; camera optical x = right = PX4 body y, so it must be `+ray_x`). It only showed up when the person was off-centre. Fixing it: median error to the true person 1.95 m -> 0.23 m (gz_truth attitude, UAV hovering) and 0.70 m with the PX4 attitude (this boot PX4 heading was 8 deg off Gazebo). My earlier "1.8 m offset was PX4 yaw" conclusion was largely wrong: the yaw error is real (random per boot: 6-33 deg) but the big offset was this bug.
2. **gz_truth mode had never worked**: ros_gz_bridge's TFMessage carries empty frame names, so the model was never matched and the localizer silently used PX4 heading. Now uses /world/<w>/dynamic_pose/info (first entry = UAV model) and the pose HISTORY: YOLO stamps each detection with the frame arrival time, the localizer interpolates the Gazebo pose at that time (YOLO adds ~0.3 s latency; the UAV moves/rotates meanwhile). The earlier 0.68 m result was therefore from the PX4-heading path.
3. **YOLO detects the distractors as people**: with the UAV sweeping, of 154 person detections 35 were real people, 47 were on the house/car/truck/shed (conf up to 0.66-0.78, bbox aspect similar to a person: 1.5-3), the rest were mismatches from unsynchronized frames. Bbox aspect ratio does not separate them. These are real false positives (useful for Step 22 robustness); verification must be made more robust (e.g. multi-view consistency / larger confidence / person-height check from the localized range).
4. Camera images tilt strongly while the UAV moves (roll ~20 deg); projection with the synchronized true pose agrees with detections to ~10-20 px (bbox height 249 px vs 239 projected), so the camera model and geometry are right.
5. Startup flake: some boots the UAV was already 12-35 m away before the mission (arming while the EKF is unsettled / High Gyro Bias preflight). TODO: gate mission start on a position check and auto-retry stack start.

## Perception debugging part 2 + FIRST SUCCESSFUL STEP 21 RUN (2026-09-24)
- **Camera/pose time offset**: swept the pose lookup offset against the projection of the known people; error minimum at 0.6 s (median 19 px vs 81 px at 0, 100+ px beyond 0.9 s). Localizer param `pose_delay_s` (default 0.6, gz_truth mode) looks up the Gazebo pose at (detection arrival - 0.6 s). Load dependent (measured with YOLO 5 Hz, RTF 1). With it: 218/226 localizations (96 %) within 2 m of a real person while the UAV moves, median 0.47 m (0.23 m when hovering).
- **Full mission in sage_sar** ("Find all people in this area and report their locations", fresh battery, stack.sh 1200): MISSION COMPLETE status=area_covered, found=3, duration 283 s, all 9 coverage waypoints scanned, 1 unconfirmed candidate rejected, 0 phantom verifications, then HOME REACHED -> LAND -> DISARM, 0 tracebacks.
  | found | true (NED) | error |
  | (0.1, 4.1) | (0, 4) | 0.14 m |
  | (4.9, -5.8) | (5, -6) | 0.22 m |
  | (-6.0, 6.7) | (-6, 7) | 0.30 m |
- Caveats to state in any demo/paper: localization uses SITL ground-truth attitude and pose (attitude_source=gz_truth) with a tuned 0.6 s camera delay; the PX4-estimate path works too (0.70 m median in a boot with 8 deg heading error) but has a random per-boot yaw offset and no delay compensation. Distractors are visual-only (no obstacle avoidance). Static people only. One run, one world: repeat runs needed for statistics (Step 23/24).
- Reproduce: `SAGE_WORLD=sage_sar scripts/stack.sh 1200` -> `scripts/start_mission_nodes.sh` -> publish the mission on /sage/mission/command.
- Next: repeat runs for statistics; startup-health gate + retry in stack.sh; verification robustness vs distractors (Step 22); real-world path (PX4 attitude with delay compensation); Claude/LLM parser live test (needs ANTHROPIC_API_KEY).

## Trial infrastructure + 5-run repeat in sage_sar (2026-09-24)
- New scripts: `stack_verified.sh` (starts the stack, checks the UAV hovers within 4 m of the origin at z -0.8..-3.5 and RTF > 0.5, retries up to 3x; it correctly rejected/retried bad starts), `run_trials.sh N world [mission]` (repeats stack start -> mission -> wait for MISSION COMPLETE -> save logs to results/logs/<id>/ -> append a scored row to results/trials_<world>.csv), `score_trial.py` (matches reported locations to the known people within 1.5 m: tp/fp/fn/mean+max error/duration/rejected candidates), `restart_perception.sh`. PX4 flight logs (~2.6 GB of old .ulg under PX4-Autopilot/build/px4_sitl_default/rootfs/log) were deleted to free disk; each run creates ~30-100 MB more, clean them periodically.
- **Results, 5 runs, mission "Find all people in this area and report their locations", sage_sar (3 people), gz_truth attitude, YOLO 5 Hz:**
  | run | found | TP | FP | FN | mean err | max err | duration | rejected cand. |
  | 1 | 3 | 3 | 0 | 0 | 0.29 | 0.36 | 340 s | 2 |
  | 2 | 2 | 2 | 0 | 1 | 0.34 | 0.45 | 610 s | 8 |
  | 3 | 3 | 3 | 0 | 0 | 0.35 | 0.50 | 395 s | 3 |
  | 4 | 3 | 3 | 0 | 0 | 0.34 | 0.71 | 588 s | 7 |
  | 5 | 3 | 3 | 0 | 0 | 0.23 | 0.32 | 360 s | 3 |
  Summary: recall 14/15 = 93 %, precision 100 % (0 false positives), mean position error ~0.31 m, all runs ended `area_covered`, duration 340-610 s (median 395 s), 2-8 phantom candidates rejected per run (each costs up to 45 s and is the main driver of duration).
- Run 2 missed one person (not yet analysed; logs in results/logs/0924_040647). Ideas: shorter candidate timeout (45 -> 25 s), reject candidates that were only seen from far away, a second sweep pass over unscanned regions when fewer than expected... (the mission has no known count).

## Session log 2026-09-24 (late): root cause of yaw/startup problems, new capabilities
- **ROOT CAUSE of the random PX4 yaw error and the startup failures ("High Gyro Bias", "Strong magnetic interference", "ekf2 missing data", never "Ready for takeoff"): a persisted PX4 parameter file (rootfs/parameters.bson) had accumulated a bad learned magnetometer offset, CAL_MAG0_ZOFF = 0.2577 G on a 0.49 G field.** The EKF flagged the mag as disturbed (cs_mag_field_disturbed true, cs_yaw_align false), so yaw was either unaligned or wrong by 6-33 deg per boot. Fix: stack.sh deletes parameters.bson/parameters_backup.bson before every PX4 start. After the fix: cs_mag_3d true, cs_mag_field_disturbed false, PX4 heading within 2 deg of Gazebo, position within 0.05 m. (My earlier statement that the yaw error was random/unfixable was wrong.)
- **Ground-truth dependence removed**: the localizer now buffers the PX4 pose+attitude and looks up the pose at (frame arrival - 0.6 s) like the gz path, with `px4_height_offset_m` = -0.3 (empirical sweep: 0.11 m median error at 4 m, 0.53 m at 10 m; better than gz_truth mode which had 0.24/0.75 because the bbox anchor is 10 % above the feet). PX4 mode is now the default (stack.sh, restart_perception.sh); gz_truth stays available (ATT=gz_truth) for evaluation.
- Sim-only relaxed arming thresholds in stack.sh (COM_ARM_MAG_STR 0, COM_ARM_MAG_ANG 180, COM_ARM_IMU_ACC 2, COM_ARM_IMU_GYR 1); stack start waits for machine load < 2 and fails fast (60 x 2 s per stage) so stack_verified.sh can retry.
- **New drone/software capabilities**: (1) continuous 360 deg yaw sweep at each coverage waypoint (`scan_mode` rotate|stepped, `scan_yaw_rate_deg` 45; was four held 90 deg headings); (2) map-based obstacle avoidance: `obstacle_map.py` (inflated rectangles, visibility-graph detours, 6 unit tests), planner routes around known obstacles and projects viewpoints/waypoints out of them, Mission Manager rejects viewpoints inside them (params `obstacles`, `obstacle_margin`; KNOWN static map only - no perception of unknown obstacles); (3) camera pitch parameter in the localizer (`camera_pitch_deg`, not yet used on the vehicle); (4) candidate refinement + faster phantom rejection + waypoint lead cap (earlier).
- **New world `sage_hard`** (worlds/make_worlds.py generates it plus config/sage_hard.env and config/truth_<world>.json): 24 x 24 m area, 5 people (3 static, 2 WALKERS at 0.6 m/s along fixed paths), 2 buildings, a wall and 3 trees WITH collision, 3 visual-only distractors. Scoring accepts a walker report within 1.5 m of its path. scripts/start_mission_nodes.sh reads config/<world>.env (search area, obstacle map); run_trials.sh reads SAGE_DRAIN from it.
- First sage_hard trial (before motion-aware dedupe): area_covered, found 5, TP 4, FP 1 (walker verified twice), FN 1 (static person 2.3 m from the walker's verified spot was merged into it), mean err 0.53 m, 632 s. Fixes then made: motion estimate from the evidence window (speed > 0.3 m/s = moving), moving-aware duplicate merge (growth radius 1.2 m/s, cap 8 m), static candidates no longer blocked by a moving person's old position, sweep waypoints projected out of obstacles (114 needless 'inside obstacle' rejections).
- Do not edit scripts/run_trials.sh while a batch runs (bash reads scripts incrementally); batches are run from a copy (/tmp/rt.sh).

## Session 2026-09-24 (night): sage_hard false-positive analysis, threshold change, trials
- Committed sage_hard trial logs (b1ea9cb). No git remote yet: nothing is backed up off this machine. Decide on a remote and whether to publish docs/review.md and the `SAGE-UAV` plan-paste file.
- **sage_hard "false positives" are mostly DUPLICATES of one real person, not phantoms**: walkers (W1 path (4.5,-11)->(4.5,-5); W2 path (-5,5)->(-5,11)) were verified as static at several points along their path (5-6 m apart, outside the 2.5 m merge radius), and static S3 (8,-8) was verified twice 1.5-1.7 m apart because the first reading was flagged MOVING and the static-vs-moving merge rule only merges within 1.5 m.
- Speed from the 11-point / 2 s evidence window is noisy: over 13 verified targets, static <= 0.16 m/s, walkers >= 0.30 m/s in the first analysis, but later runs gave a walker reading of 0.16 and a static reading of 0.49.
- Change made: planner `moving_speed_threshold` 0.45 -> 0.25 (sage_viewpoint_planner.py:292). NOT yet committed.
- Trials with 0.25 (results/trials_sage_hard.csv; old rows copied to trials_sage_hard_v2_thr045.csv):
  | run | TP | FP | FN | mean err | duration |
  | 0924_220557 | 5 | 1 | 0 | 0.39 m | 556 s (area_covered) |
  | 0924_213855 | no_report: hit the runner's 800 s cutoff at waypoint 9/16, 7 verified; by hand 4 correct + 2 duplicates (W1 twice, W2 twice) | | | | >800 s |
  | 0924_215730 | stack_failed (sim real-time factor 0.38 < 0.5 health gate, load 2-3) | | | | |
  Only one clean run, so the 0.25 threshold is NOT validated. The remaining FP in 220557 is S3 verified at (6.43,-8.64) and (7.97,-8.81).
- Sim is slow when other things run (RTF 0.37-0.72): trials are not comparable unless the machine is otherwise idle.
- Script fix (uncommitted): stack_verified.sh now does `mkdir -p /tmp/sage_logs` (a missing dir made the first trial launch stall). Do not use `pkill -f rt.sh` from a tool shell (it kills the calling shell).
- Next: (1) widen the static-vs-moving merge in `same_person` from 1.5 m to 2.5 m (fixes the S3 pattern); (2) decide how to handle walker duplicates 5-6 m apart (time/path-based merge risks merging two real people); (3) run >= 5 trials with the machine idle; (4) commit; (5) then the earlier list: explain the `no_report` run 0924_050428 and the missed person in 0924_040647, shorten missions (phantom rejections), live-test the Claude parser (Individual API key, prepaid credit, SAGE_LLM_MODEL=claude-haiku-4-5-20251001 suggested), Step 22 robustness, Step 23 experiments.

## Session 2026-09-25: static-vs-moving merge widened
- sage_viewpoint_planner.py `same_person`: static candidate vs verified walker now merges within 2.5 m (was 1.5 m), targeting the sage_hard S3 duplicate (verified twice 1.5-1.7 m apart). Built; NOT yet validated by trials.
- Next: >= 5 sage_hard trials with the machine idle (`scripts/run_trials.sh 5 sage_hard`, copy the script to /tmp first), then walker-duplicate handling (5-6 m apart), Claude parser live test, Step 22/23.

## 5-trial batch on sage_hard with 2.5 m merge (2026-09-25): inconclusive
- Results: trial 1 area_covered TP5/FP1/FN0, 714 s. Trials 2, 4, 5: `no_report`, cut off by run_trials.sh's 800 s wait (not by the mission). Trial 3: UAV flew away (~20 m from the candidate, outside the area), skipped by hand and logged as `skipped_flyaway`; PX4 failure reason NOT checked (no flight log for that run).
- Trials 2 and 4 logs: all 5 people verified at correct positions, 0 duplicates, 0 false positives, 0 tracebacks. They had reached waypoint 11/16 and 15/16 at cutoff. 55-65 % of run time is candidate handling (420-540 s), 8-14 rejected candidates per run. Sweep alone is ~12-15 s per waypoint.
- Mission Manager: 57-74 "viewpoint jump too large" rejections per run (3.1-4.3 m), in bursts; effect not traced.
- Change: run_trials.sh wait cutoff raised 800 s -> 1600 s (mission_timeout_s is 900 s, so a report is always produced). candidate_timeout_s is already 25 s in code (older entries say 45 s); not changed.
- Next: rerun the batch; look at why so many phantom candidates arise (world model tracks from distractors / walkers) rather than only shortening timeouts; check the flyaway cause.

## sage_hard batch 2 (1600 s cutoff) + W2 duplicate analysis (2026-09-25)
- Trials: 020852 area_covered TP5/FP1 859 s; 022758 TP5/FP0 895 s; 024917 skipped (UAV flew away 24 m out and sat on the ground; Mission Manager then rejected every viewpoint as "too far", 2133 times - deadlock; cause of the flyaway NOT found, logs in results/logs/0925_024917); 030627 timeout(900 s) TP5/FP0; 032505 area_covered TP5/FP1 707 s. Recall 20/20, 2 FP, mean err 0.27-0.62 m.
- Both FPs are walker W2 (path (-5,5)-(-5,11)) verified twice: 2.7 m apart (trial 1; missed by the 2.5 m rule) and 6.0 m apart (trial 5; second reading speed 0.25, not > threshold). S1, S2, S3, W1 never duplicated.
- sage_sar (easy world) has 0 FP in 12 scored trials; the duplicates, long runs (707-900 s vs 200-610 s) and phantom candidates (7-14 vs 0-8) are sage_hard specific.
- Change: static-vs-walker merge radius 2.5 -> 3.2 m (fixes the trial-1 pattern; capped below 4.0 m because static S2 sits 4.0 m from the W2 path). The 6 m pattern (trial 5) is NOT fixed: a distance rule that large would swallow S2. Needs path/time-aware merging or a better speed estimate.

## 3.2 m merge trial: REVERTED to 2.5 m (2026-09-25)
- Trial 0925_040416 (3.2 m): area_covered, found 5, TP4/FP1/FN1, 875 s. FN = static S3 (8,-8): 6 S3 candidates were merged into verified walker W1 (5.12,-7.34); two at 2.6-2.8 m would have survived at 2.5 m. S3 is ~3.5 m from the W1 path, S2 ~4.0 m from the W2 path, so a larger distance radius cannot work here. FP = W2 verified twice (static reading first at (-4.57,5.12), then MOVING at (-6.47,7.88), 3.4 m apart; the verified-static rule uses 2.5 m).
- Radius back to 2.5 m. Open: walker duplicates need path-aware/time-aware merging or a better speed estimate (walker readings 0.18-1.39 m/s).

## Walker-duplicate analysis: speed cannot identify walkers (2026-09-25, no code change)
- World model motion label vs truth, all 0925_02-04 sage_hard world.logs (per-update, smoothed position within 1.5 m of a true person): MOVING share S1 20 %, S2 39 %, S3 38 %, W1 41 %, W2 65 %. The persistent-motion classifier labels static people MOVING far too often and walkers STATIONARY too often -> not usable for identity. Planner window speed: walkers read 0.18-1.39 m/s, statics 0.03-0.24 (overlap).
- Geometry: S3 is ~3.5 m from the W1 path, S2 ~4.0 m from the W2 path, while W2 duplicates were 2.7-6 m apart -> no distance/speed rule separates them (3.2 m radius lost S3). Walkers turn around with ~1 s pause at the path ends, which further weakens slope-based speed there.
- Conclusion: identity of a possibly-moving person can't be decided from a 2 s speed estimate; needs either active disambiguation (revisit the earlier verified position: person still there => distinct, gone => same walker) or reporting possible duplicates as flagged pairs instead of merging/suppressing.
