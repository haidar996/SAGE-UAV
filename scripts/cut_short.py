#!/usr/bin/env python3
"""Make a short time-lapse from a recorded demo: keep the intro/outro cards, take every Nth mission frame.
usage: python3 scripts/cut_short.py in.mp4 out.mp4 [N=3] [intro_frames=80] [outro_frames=120]"""
import sys
import cv2

src, dst = sys.argv[1], sys.argv[2]
n = int(sys.argv[3]) if len(sys.argv) > 3 else 3
intro = int(sys.argv[4]) if len(sys.argv) > 4 else 80
outro = int(sys.argv[5]) if len(sys.argv) > 5 else 120
cap = cv2.VideoCapture(src)
total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
fps = cap.get(cv2.CAP_PROP_FPS)
w, h = int(cap.get(3)), int(cap.get(4))
out = None
for cc in ('avc1', 'H264', 'mp4v'):
    out = cv2.VideoWriter(dst, cv2.VideoWriter_fourcc(*cc), fps, (w, h))
    if out.isOpened():
        break
speed = 4 * n
for i in range(total):
    ok, f = cap.read()
    if not ok:
        break
    if i < intro or i >= total - outro:
        out.write(f)
    elif (i - intro) % n == 0:
        cv2.rectangle(f, (1060, 688), (1280, 720), (30, 25, 22), -1)
        cv2.putText(f, f'{speed}x time-lapse', (1080, 708), cv2.FONT_HERSHEY_DUPLEX, 0.5, (150, 150, 150), 1, cv2.LINE_AA)
        out.write(f)
out.release()
print('written', dst, 'speed', speed, 'x')
