# LinkedIn post - SAGE-UAV

## What LinkedIn accepts (checked)
- **Video:** MP4 with H.264 video and AAC audio is the recommended format (30 fps, up to 4096 x 2304, under 10 min on mobile / 15 min on desktop, up to 5 GB).
- **One video per post, and a video cannot be combined with images.** A post is *either* one video, *or* up to 20 images, *or* a PDF document (carousel).
  Sources: [LinkedIn video specs 2026](https://www.yansmedia.com/blog/linkedin-video-specs), [LinkedIn media limits](https://help.postpone.app/platforms/linkedin/media-limits).

## Files (all in `demo/linkedin/` on your machine)
| file | use |
|---|---|
| `SAGE-UAV-linkedin-video.mp4` | **the video to upload**: 1920x1080, 30 fps, H.264 High + AAC, 6:30. It contains both views side by side (onboard camera + Gazebo view from above the drone) and the pictures as slides (architecture, results, frames), so nothing else needs uploading with it. Not stored in git (47 MB); regenerate with `scripts/render_showcase.py` + `scripts/tools/encode_social.py` |
| `SAGE-UAV-carousel.pdf`, `pages/page_1..6.png` | the pictures as a 6-page document (PDF carousel) or as images |
| `../sage_uav_camera_view.mp4`, `../sage_uav_drone_top_view.mp4` | the two separate 720p videos (30 fps, AAC) for other places |

## How to post
1. **Best:** one post with `SAGE-UAV-linkedin-video.mp4` + the text below. Put the repository link in the text.
2. **Pictures:** as a second post (`SAGE-UAV-carousel.pdf` as a document, or `pages/*.png` as a multi-image post), or one image in the first comment.

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
- Rescue scene with buildings, trees, cars and one person walking: 25 of 28 people found over 7 flights (0.46 m average error). Standing people are found reliably; the walking person is the hard part, and in 3 flights it was reported twice.
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
