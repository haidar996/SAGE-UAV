# Environment: everything needed to rebuild the setup

The project runs on three big third-party trees that are **not copied** into this repository (they are gigabytes and have their own
repositories). Everything that is specific to SAGE-UAV is captured here or elsewhere in the repo:

| what | where it lives | how this repo captures it |
|---|---|---|
| ROS 2 package `sage_px4_interface` | `src/sage_px4_interface/` | the code itself |
| ROS workspace `~/sage_ws` | your machine | `environment/setup_workspace.sh` + `environment/sage_ws.repos` (px4_msgs v1.16.2) |
| PX4-Autopilot (SITL) | `~/PX4-Autopilot` | pinned to **v1.16.0** (`6ea3539`); our changes are only the Gazebo worlds (`worlds/`) and one drone model file (`models/x500_mono_cam/model.sdf`, diff in `px4_patches/`); installer `environment/patch_px4.sh`. PX4 parameters are set at runtime by `scripts/stack.sh` |
| Gazebo | system | Gazebo Sim **8.15.0** (Harmonic) |
| Micro XRCE-DDS Agent | `~/Micro-XRCE-DDS-Agent` | pinned to **v2.4.3** (`7362281`), UDP 8888 |
| Vision Python env `~/sage_vision_env` | your machine | `environment/requirements-vision.txt` (193 pinned packages, Python 3.10.12, CPU-only PyTorch 2.5.1 + ultralytics 8.4.157) |
| YOLO weights | `models/yolo/yolo26n.pt` | included (5.5 MB, sha256 prefix `9b09cc8b`; Ultralytics weights are AGPL-3.0) |
| ROS/Gazebo apt packages | system | `environment/apt-ros-packages.txt` |

Tested on Ubuntu 22.04.5, Python 3.10.12, 4 CPU cores, 7 GB RAM, no GPU. The simulation, YOLO and PX4 share one CPU, so YOLO is capped at 5 Hz
(`SAGE_YOLO_HZ`) and the stack start is health-gated (`scripts/stack_verified.sh`).

## From scratch

```bash
# 1. ROS 2 Humble + Gazebo Harmonic bridge packages (see apt-ros-packages.txt for the exact list)
sudo apt install ros-humble-desktop ros-humble-ros-gzharmonic ros-humble-vision-msgs ros-humble-cv-bridge \
     ros-humble-image-transport-plugins ros-humble-ros-gz-image python3-colcon-common-extensions python3-vcstool

# 2. PX4 v1.16.0 SITL with Gazebo (x500 with mono camera)
git clone https://github.com/PX4/PX4-Autopilot.git --recursive ~/PX4-Autopilot
cd ~/PX4-Autopilot && git checkout v1.16.0 && git submodule update --init --recursive && make px4_sitl gz_x500_mono_cam   # first build, then Ctrl-C

# 3. this repository's worlds and drone model into PX4
git clone https://github.com/haidar996/SAGE-UAV.git ~/SAGE-UAV
~/SAGE-UAV/environment/patch_px4.sh

# 4. Micro XRCE-DDS Agent v2.4.3
git clone --branch v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git ~/Micro-XRCE-DDS-Agent
mkdir -p ~/Micro-XRCE-DDS-Agent/build && cd ~/Micro-XRCE-DDS-Agent/build && cmake .. && make -j2

# 5. ROS workspace (px4_msgs v1.16.2 + the package, symlinked)
~/SAGE-UAV/environment/setup_workspace.sh

# 6. vision environment (the node's launcher uses this exact interpreter path)
python3 -m venv ~/sage_vision_env
~/sage_vision_env/bin/pip install --extra-index-url https://download.pytorch.org/whl/cpu -r ~/SAGE-UAV/environment/requirements-vision.txt
cp ~/SAGE-UAV/models/yolo/yolo26n.pt ~/sage_ws/          # weights are loaded by file name

# 7. run
cd ~/SAGE-UAV && SAGE_WORLD=sage_rescue scripts/stack_verified.sh 1800 && SAGE_WORLD=sage_rescue scripts/start_mission_nodes.sh
```

## Paths the scripts assume

`~/PX4-Autopilot`, `~/Micro-XRCE-DDS-Agent`, `~/sage_ws`, `~/sage_vision_env`, `~/SAGE-UAV`, and `/tmp/sage_logs` (created automatically).
`src/sage_px4_interface/sage_px4_interface/yolo_detector_launcher.py` starts with `#!/home/haidar/sage_vision_env/bin/python`; edit the first line
if your home directory differs.

## Practical notes

- Delete PX4's persisted `parameters.bson` before each SITL start (`stack.sh` does): a learned bad magnetometer offset broke yaw alignment.
- Extra Gazebo cameras and the Gazebo GUI cost real-time factor; below about 0.5 the drone becomes unreliable. Record with the light logger
  (`SAGE_RECORD=1`) and render offline; keep the GUI off (`SAGE_GUI=1` turns it on).
- The `px4-param` values, arming-check relaxations and the force-disarm after landing are simulation-only shortcuts (`docs/limitations.md`).

## Optional: social-media encoding
`pip install imageio-ffmpeg` (bundles ffmpeg) enables `scripts/tools/encode_social.py`, which encodes videos to the platforms' recommended settings (H.264 High, yuv420p, 30 fps, AAC, faststart).
