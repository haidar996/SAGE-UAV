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
1. LOCALIZATION ACCURACY UNVERIFIED / likely poor: person's true position is gz (4,0) = NED ~(0, 4); the verified report said (-0.17, 1.49), ~2.5 m off, and localizer output swings (e.g. (2.1,7.2)->(2.4,6.9) while moving). No ground-truth comparison has ever been done; a "found: locations" report is only as good as this. Needs a ground-truth error study (use gz pose of actor, static UAV at several viewpoints) before Step 21/experiments.
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
