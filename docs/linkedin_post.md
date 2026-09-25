# LinkedIn post - SAGE-UAV

Attach the two videos of the SAME flight: `demo/sage_uav_camera_view.mp4` (onboard camera + YOLO + map) and `demo/sage_uav_gazebo_overhead.mp4` (Gazebo overhead view), each 5.3 min at 2x real speed (4 of 4 people found, no duplicate). Then the pictures in this order:
1. `results/figures/linkedin_card.png`  2. `demo/pictures/collage_2x2.png` (and `demo/pictures/overhead/top_verified_4.png`)  3. `results/figures/architecture.png`  4. `results/figures/results_overview.png`

---

**I built a drone that understands "find all people in this area", flies the search on its own, and tells you exactly where they are.**

SAGE-UAV (Semantic AI-Guided Exploration & Active Search) is an autonomous search-and-rescue quadrotor, built in simulation with ROS 2, PX4 and Gazebo.

You type one sentence. The drone then:
- turns it into a validated mission,
- sweeps the search area with an onboard camera and YOLO,
- localizes every person in 3D (about 0.3 m average error),
- keeps a semantic memory and re-observes a candidate until it is really a person,
- flies around obstacles, watches its own battery,
- and returns home and lands, with a report of who was found and where.

**The numbers (simulation)**
- Rescue scene with buildings, trees, cars and one person walking: 21 of 24 people found over 6 flights (0.46 m average error). Standing people are found reliably; the walking person is the hard part, and in 3 flights it was reported twice.
- Simpler scene, 3 standing people: 100% found, 0 false alarms, 0.3 m average error, about 4 minutes.
- Every flight command passes a safety check before it reaches the autopilot.

**What I learned the hard way**
- A single speed estimate cannot tell a walking person from a standing one. My drone sometimes reports the same walker twice. I measured it, wrote the analysis down and kept it in the results instead of hiding it.
- Two "optimizations" I made made the drone faster and quietly worse at finding people. Only checking against ground truth caught it.
- Boring engineering wins: retries, health checks, logging every run, and a guard that ends the mission if the drone is ever lost.

**What is next:** an active "go back and look again" step to resolve walker identity, real hardware tests, and richer scenes.

Everything is simulation, with documented shortcuts, and the code, logs and honest limitations are in the repo.

#robotics #drones #ROS2 #PX4 #Gazebo #computervision #YOLO #autonomy #searchandrescue #AI #engineering

---
Short version (for a comment or a repost):
An autonomous drone that takes "find all people in this area", searches, detects and localizes them in 3D (0.3 m), verifies each one, and lands by itself. ROS 2 + PX4 + Gazebo + YOLO. Simulation only, limitations documented. Video below.
