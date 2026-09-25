# Scope and assumptions

SAGE-UAV is a simulation research system. This page lists the assumptions to keep in mind when quoting its results.

**Simulation setup**
- PX4 SITL and Gazebo only; the camera is the simulated x500 mono camera; people are animated actors.
- PX4 arming checks are relaxed and the persisted parameter file is reset at every start (`scripts/stack.sh`); the land detector does not fire in this
  SITL, so the offboard node force-disarms after touch-down. The battery is a time-based simulation model.
- YOLO runs at 5 Hz on CPU so that the simulation stays near real time.

**Perception and localization**
- Only the `person` class is detected; attributes such as colour are parsed but reported as unsupported.
- Localization uses PX4 pose and attitude looked up 0.6 s before frame arrival and a fixed height offset; both were tuned for this setup.

**Navigation**
- Obstacle avoidance uses a known static map; unknown obstacles are not sensed.

**Moving people**
- A walking person is sometimes reported twice, because speed estimated over a couple of seconds cannot separate a walker from a standing person. Recall is
  prioritised over precision. A designed fix (active re-observation) is in [`design/identity_check.md`](design/identity_check.md).

**Statistics**
- Each configuration has 4-7 scored runs; treat the numbers as indicative.
