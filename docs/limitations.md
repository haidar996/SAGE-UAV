# Limitations and simulation-only shortcuts (state 2026-09-25)

State these in any report, paper or demo.

## Simulation-only
- PX4 SITL + Gazebo only; nothing flown on hardware. Camera is the simulated x500_mono_cam.
- Arming/health: `scripts/stack.sh` relaxes PX4 arming checks (COM_ARM_MAG_STR 0, COM_ARM_MAG_ANG 180,
  COM_ARM_IMU_ACC 2, COM_ARM_IMU_GYR 1), sets NAV_DLL_ACT=0 (no GCS link in SITL), deletes the saved
  parameter file every boot (a learned bad mag offset broke yaw alignment), caps MPC velocity/tilt.
- Landing: the PX4 land detector never fires in this SITL, so after NAV_LAND the offboard node force-disarms
  (param2=21196) once within 0.15 m of the ground for 3 s. Not acceptable on a real vehicle.
- Battery is a stub (SIM_BAT_DRAIN, time-based, independent of load): the energy-aware logic is validated
  against a time-based drain only.
- YOLO runs capped at 5 Hz (SAGE_YOLO_HZ); the sim only runs at real-time factor ~1 with that cap and an idle machine.
- Localization: PX4 pose + attitude with a tuned 0.6 s camera delay (`pose_delay_s`) and a -0.3 m height offset
  (`px4_height_offset_m`), both fitted in this sim/load; they do not transfer to other hardware or load.
  A `gz_truth` mode exists for evaluation only.
- Worlds: `sage_sar` (3 static people), `sage_hard` / `sage_hard_long` (3 static + 2 back-and-forth walkers at
  0.6 m/s, 6 m / 6-12 m paths). People are animated actors; only class `person` is detected (COCO class 0).
  Distractors are visual-only. Results are for these worlds, not general.

## Known behavioural limits
- Obstacle avoidance uses a KNOWN static map (`obstacles` parameter); unknown obstacles are not sensed.
- Walkers can be reported twice (identity is not resolved; speed/motion state cannot separate walkers from
  static people, see progress.md and design_identity_check.md). Recall is prioritised over precision.
- Missions take 700-900+ s in sage_hard; about 28 % of it goes on candidate tracks that are never confirmed.
- One sim-freeze followed by a UAV flyaway (0925_024917) was seen in ~12 recent hard-world runs, cause unknown.
- Small samples: 4-5 scored runs per configuration.
- Attributes (colour) and non-person classes are unsupported by perception (parser accepts them but marks them
  unsupported). LLM parsing needs an API key; rule-based parsing is the default fallback.
