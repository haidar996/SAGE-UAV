# Changelog

## 0.1.0 - 2026-09-25
First public release.
- Full mission pipeline: parser, YOLO perception, 3D localization, semantic world model, active planner, mission manager, energy monitor, offboard control.
- Worlds `sage_sar`, `sage_rescue`, `sage_hard`, `sage_hard_long` with ground truth and automatic scoring.
- Trial runner, figures, two-view demo videos rendered from a light in-flight log.
- Guards: obstacle map and viewpoint validation, energy return-home, UAV-lost guard, PX4 offboard-loss hold.
- Environment folder with pinned versions, PX4 patches and setup scripts.
