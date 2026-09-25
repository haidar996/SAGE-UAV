# Demo

Two videos of the **same flight** (run `0925_180639`, world `sage_rescue`: 4 of 4 people found, no false reports, mean error 0.44 m, 706 s),
each 6 min 07 s at 2x real speed, H.264 (High, yuv420p), 1280 x 720, 30 fps, AAC audio track, index at the start of the file.

| file | shows |
|---|---|
| `sage_uav_camera_view.mp4` | onboard camera with YOLO boxes, live mission map, pipeline stage bar, battery |
| `sage_uav_drone_top_view.mp4` | Gazebo view from a camera 5 m above the drone (north-up, circular window), side map and verified positions |
| `pictures/`, `linkedin_pictures.zip` | collage, key frames, drone-top frames, figures |
| `linkedin/` | `SAGE-UAV-linkedin-video.mp4` (1080p split screen + picture slides, 6:30, made locally, not in git), `SAGE-UAV-carousel.pdf` and `pages/` (picture carousel) |
| `linkedin_post.md` | post text, what LinkedIn accepts, how to post |
| `archive/` | earlier versions |

**How they were made.** During the flight only a light logger (`scripts/record_raw.py`) saves JPEG frames, poses and battery, so the simulator is not
slowed. `scripts/render_demo.py` then renders both videos from that log at exactly 2x speed and `scripts/tools/faststart.py` moves the MP4 index to the
front so the files stream in browsers.

**Social-media version.** `scripts/render_showcase.py` renders one 1920 x 1080 video (camera left, view from above the drone right, HUD below, picture
slides in between); `scripts/tools/encode_social.py` encodes it to the platforms' recommended settings (H.264 High, yuv420p, 30 fps, AAC, faststart;
needs `pip install imageio-ffmpeg`). `scripts/make_carousel.py` builds the PDF carousel.
