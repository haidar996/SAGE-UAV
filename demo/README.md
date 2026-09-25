# Demo

Two videos of the **same flight** (run `0925_180639`, world `sage_rescue`: 4 of 4 people found, no false reports, mean error 0.44 m, 706 s),
each 6 min 07 s at 2x real speed, H.264 (High, yuv420p), 1280 x 720, web-optimised.

| file | shows |
|---|---|
| `sage_uav_camera_view.mp4` | onboard camera with YOLO boxes, live mission map, pipeline stage bar, battery |
| `sage_uav_drone_top_view.mp4` | Gazebo view from a camera 5 m above the drone (north-up, circular window), side map and verified positions |
| `pictures/`, `linkedin_pictures.zip` | collage, key frames, drone-top frames, figures |
| `linkedin_post.md` | post text and the order of the attachments |
| `archive/` | earlier versions |

**How they were made.** During the flight only a light logger (`scripts/record_raw.py`) saves JPEG frames, poses and battery, so the simulator is not
slowed. `scripts/render_demo.py` then renders both videos from that log at exactly 2x speed and `scripts/tools/faststart.py` moves the MP4 index to the
front so the files stream in browsers.
