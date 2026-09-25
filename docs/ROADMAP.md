# Roadmap and status

| # | area | status |
|---|---|---|
| 0-8 | environment, PX4 SITL, offboard control, camera, YOLO detection | done |
| 9-14 | tracking, 3D localization, semantic world model, motion classification, memory | done |
| 15-17 | active search, coverage sweep, candidate handling | done |
| 18 | active vision: viewpoint quality (confidence, box size, image-border margin) | done |
| 19 | mission understanding: rule parser, JSON spec, completion and report; optional LLM front end | done (LLM live test needs an API key) |
| 20 | energy awareness: monitor, return-home, energy-aware viewpoint choice | done |
| 21 | search-and-rescue worlds, coverage search, scoring | done |
| 22 | robustness: guards for lost UAV, offboard loss, start-up retries | mostly done |
| 23 | controlled experiments (random vs intelligent search, fixed vs active viewpoints, distance vs energy-aware planning, memory, tracking, noise levels) | next |
| 24 | results and figures | first version done (`docs/EVALUATION.md`) |
| 25 | demonstration video and write-up | done (`demo/`) |

## Next
1. **Identity check for walking people:** revisit an earlier verified position to decide whether a new detection is the same person (design and
   tested decision logic exist: `docs/design/identity_check.md`, `identity_resolution.py`).
2. **Shorter missions:** cut the time spent on tracks that never confirm; log the per-tick evidence signal to design a safe early exit.
3. **Experiments A-F** with 10 runs per configuration.
4. **Hardware-in-the-loop** and a real camera stream.

The original planning document is kept as [`archive/original_plan.md`](archive/original_plan.md); the engineering diary is
[`development_log.md`](development_log.md).
