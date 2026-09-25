#!/usr/bin/env python3
"""Live demo recorder: onboard camera (with YOLO boxes) + top-down mission map + status bar -> mp4 + stills.

Run it while a mission is flying (ROS + the SAGE stack up):
    python3 scripts/record_demo.py --world sage_rescue --out results/demo/run1
    python3 scripts/record_demo.py --selftest --out /tmp/selftest      # layout check without ROS
It ends by itself ~25 s after MISSION COMPLETE appears in the planner log (or on Ctrl-C).
Output: <out>/demo.mp4 (time-lapse, --speed x), <out>/stills/*.png, <out>/summary.json
"""
import argparse
import collections
import json
import math
import os
import re
import time

import cv2
import numpy as np

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
W, H = 1280, 720
HDR, PANEL_H, HUD_H = 56, 480, 184
BG = (24, 20, 18)
ACC = (245, 166, 35)      # BGR accent (blue-ish orange)
GREEN = (96, 201, 66)
FONT = cv2.FONT_HERSHEY_DUPLEX
STAGES = ['UNDERSTAND', 'SEARCH', 'DETECT', 'LOCALIZE', 'VERIFY', 'RETURN', 'LAND']


def put(img, text, org, scale=0.6, color=(235, 235, 235), thick=1):
    cv2.putText(img, text, org, FONT, scale, color, thick, cv2.LINE_AA)


def load_world(world):
    truth = json.load(open(os.path.join(ROOT, 'config', f'truth_{world}.json')))['people']
    env = {}
    p = os.path.join(ROOT, 'config', f'{world}.env')
    if os.path.exists(p):
        for line in open(p):
            m = re.match(r'(\w+)="?(.*?)"?$', line.strip())
            if m:
                env[m[1]] = m[2]
    area = json.loads(env.get('SAGE_AREA', '[-12,12,-12,12]'))
    obst = [tuple(float(v) for v in r.split(',')) for r in env.get('SAGE_OBSTACLES', '').split(';') if r]
    return truth, area, obst


class Scene:
    def __init__(self, world, mission, speed):
        self.world, self.mission, self.speed = world, mission, speed
        self.truth, self.area, self.obst = load_world(world)
        self.trail, self.tracks, self.verified, self.visited = [], [], [], []
        self.pose = None            # north, east, down, heading
        self.batt = None
        self.boxes, self.boxes_t = [], 0.0
        self.t0 = time.time()
        self.events = {'accepted': False, 'candidate': False, 'complete': False, 'landed': False,
                       'localizing': 0.0, 'wp': (0, 0)}
        self.report = None
        n0, n1, e0, e1 = self.area
        pad = 2.5
        self.bounds = (n0 - pad, n1 + pad, e0 - pad, e1 + pad)
        self.msize = PANEL_H - 20
        self.mx0 = 640 + (640 - self.msize) // 2
        self.my0 = HDR + 10
        self.scale = self.msize / (self.bounds[1] - self.bounds[0])
        self.base = self.make_base()

    def m2p(self, n, e):
        b = self.bounds
        return (int(self.mx0 + (e - b[2]) * self.scale), int(self.my0 + (b[1] - n) * self.scale))

    def make_base(self):
        img = np.full((H, W, 3), BG, np.uint8)
        cv2.rectangle(img, (0, 0), (W, HDR), (38, 30, 26), -1)
        put(img, 'SAGE-UAV', (18, 37), 0.95, (255, 255, 255), 2)
        put(img, 'Semantic AI-Guided Exploration & Active Search', (190, 36), 0.62, (200, 200, 200))
        put(img, 'ROS 2 | PX4 | Gazebo | YOLO', (W - 330, 36), 0.58, ACC)
        cv2.rectangle(img, (640, HDR), (W, HDR + PANEL_H), (34, 28, 24), -1)
        b = self.bounds
        p0, p1 = self.m2p(b[1], b[2]), self.m2p(b[0], b[3])
        cv2.rectangle(img, p0, p1, (46, 60, 44), -1)
        for v in range(-10, 11, 5):
            a, c = self.m2p(b[1], v), self.m2p(b[0], v)
            cv2.line(img, a, c, (62, 78, 60), 1)
            a, c = self.m2p(v, b[2]), self.m2p(v, b[3])
            cv2.line(img, a, c, (62, 78, 60), 1)
        n0, n1, e0, e1 = self.area
        a, c = self.m2p(n1, e0), self.m2p(n0, e1)
        cv2.rectangle(img, a, c, (150, 200, 150), 1)
        put(img, 'search area', (a[0] + 6, a[1] + 16), 0.42, (150, 200, 150))
        for o in self.obst:
            a, c = self.m2p(o[1], o[2]), self.m2p(o[0], o[3])
            cv2.rectangle(img, a, c, (90, 110, 130), -1)
            cv2.rectangle(img, a, c, (140, 160, 180), 1)
        for p in self.truth:
            pts = p['points']
            if len(pts) == 2:
                a, c = self.m2p(*pts[0]), self.m2p(*pts[1])
                for i in range(0, 20, 2):     # dashed walker path
                    f = i / 20
                    s = (int(a[0] + (c[0] - a[0]) * f), int(a[1] + (c[1] - a[1]) * f))
                    e = (int(a[0] + (c[0] - a[0]) * (f + 0.05)), int(a[1] + (c[1] - a[1]) * (f + 0.05)))
                    cv2.line(img, s, e, (120, 140, 190), 1)
        put(img, 'N', (self.mx0 + self.msize + 6, self.my0 + 14), 0.5, (200, 200, 200))
        put(img, 'top-down, north up', (650, self.my0 + self.msize - 4), 0.4, (170, 170, 170))
        return img

    # ------------------------------------------------------------------ drawing
    def stage(self):
        ev = self.events
        if ev['landed']:
            return 6
        if ev['complete']:
            return 5
        if ev['candidate']:
            return 4
        if time.time() - ev['localizing'] < 2.0:
            return 3
        if time.time() - self.boxes_t < 1.0 and self.boxes:
            return 2
        return 1 if ev['accepted'] else 0

    def compose(self, cam, boxes=()):
        img = self.base.copy()
        # camera panel
        if cam is not None:
            small = cv2.resize(cam, (640, PANEL_H))
            for (cx, cy, bw, bh, conf) in boxes:
                x1, y1, x2, y2 = [int(v / 2) for v in (cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2)]
                cv2.rectangle(small, (x1, y1), (x2, y2), GREEN, 2)
                cv2.rectangle(small, (x1, y1 - 20), (x1 + 118, y1), GREEN, -1)
                put(small, f'person {conf:.2f}', (x1 + 4, y1 - 6), 0.5, (20, 20, 20))
            img[HDR:HDR + PANEL_H, 0:640] = small
        else:
            cv2.rectangle(img, (0, HDR), (640, HDR + PANEL_H), (20, 20, 20), -1)
        cv2.rectangle(img, (0, HDR), (640, HDR + 26), (0, 0, 0), -1)
        put(img, 'ONBOARD CAMERA + YOLO', (10, HDR + 18), 0.5, (255, 255, 255))
        cv2.rectangle(img, (640, HDR), (W, HDR + 26), (0, 0, 0), -1)
        put(img, 'MISSION MAP', (650, HDR + 18), 0.5, (255, 255, 255))
        # map layers
        for w in self.visited:
            cv2.circle(img, self.m2p(*w), 3, (120, 150, 120), -1)
        for i in range(1, len(self.trail)):
            cv2.line(img, self.m2p(*self.trail[i - 1]), self.m2p(*self.trail[i]), (255, 200, 90), 2, cv2.LINE_AA)
        for (n, e) in self.tracks:
            p = self.m2p(n, e)
            cv2.drawMarker(img, p, (60, 220, 240), cv2.MARKER_CROSS, 9, 1)
        for k, (n, e) in enumerate(self.verified, 1):
            p = self.m2p(n, e)
            cv2.circle(img, p, 9, GREEN, -1)
            cv2.circle(img, p, 12, (255, 255, 255), 1)
            put(img, f'P{k}', (p[0] + 14, p[1] - 8), 0.55, (255, 255, 255), 1)
            put(img, f'({n:.1f}, {e:.1f})', (p[0] + 14, p[1] + 8), 0.4, (190, 240, 190))
        if self.pose is not None:
            n, e, _, hd = self.pose
            p = self.m2p(n, e)
            # FOV wedge
            ov = img.copy()
            L = 4.5 * self.scale
            a1, a2 = hd - 0.6, hd + 0.6
            tri = np.array([p, (int(p[0] + L * math.sin(a1)), int(p[1] - L * math.cos(a1))),
                            (int(p[0] + L * math.sin(a2)), int(p[1] - L * math.cos(a2)))])
            cv2.fillPoly(ov, [tri], (255, 220, 120))
            img = cv2.addWeighted(ov, 0.28, img, 0.72, 0)
            tip = (int(p[0] + 11 * math.sin(hd)), int(p[1] - 11 * math.cos(hd)))
            l = (int(p[0] + 8 * math.sin(hd + 2.5)), int(p[1] - 8 * math.cos(hd + 2.5)))
            r = (int(p[0] + 8 * math.sin(hd - 2.5)), int(p[1] - 8 * math.cos(hd - 2.5)))
            cv2.fillPoly(img, [np.array([tip, l, r])], (40, 140, 255))
            cv2.polylines(img, [np.array([tip, l, r])], True, (255, 255, 255), 1, cv2.LINE_AA)
        # HUD
        y0 = HDR + PANEL_H
        cv2.rectangle(img, (0, y0), (W, H), (30, 25, 22), -1)
        cv2.line(img, (0, y0), (W, y0), ACC, 2)
        put(img, 'MISSION', (18, y0 + 28), 0.5, ACC)
        put(img, f'"{self.mission}"', (110, y0 + 28), 0.62, (255, 255, 255))
        st = self.stage()
        x = 18
        for i, name in enumerate(STAGES):
            tw = cv2.getTextSize(name, FONT, 0.5, 1)[0][0]
            col = ACC if i == st else ((90, 110, 90) if i < st else (70, 70, 70))
            cv2.rectangle(img, (x - 6, y0 + 46), (x + tw + 6, y0 + 74), col, -1 if i == st else 1)
            put(img, name, (x, y0 + 66), 0.5, (20, 20, 20) if i == st else (200, 200, 200))
            x += tw + 26
        el = time.time() - self.t0
        sim_s = el * self.speed if False else el
        put(img, f'TIME {int(sim_s // 60):02d}:{int(sim_s % 60):02d}', (18, y0 + 110), 0.7, (255, 255, 255))
        alt = f'{-self.pose[2]:.1f} m' if self.pose else '--'
        put(img, f'ALT {alt}', (200, y0 + 110), 0.7, (255, 255, 255))
        wp = self.events['wp']
        put(img, f'WAYPOINT {wp[0]}/{wp[1]}' if wp[1] else 'WAYPOINT --', (370, y0 + 110), 0.7, (255, 255, 255))
        put(img, f'FOUND {len(self.verified)}', (640, y0 + 110), 0.9, GREEN, 2)
        # battery
        b = self.batt if self.batt is not None else 100.0
        put(img, 'BATTERY', (18, y0 + 150), 0.5, (200, 200, 200))
        cv2.rectangle(img, (110, y0 + 134), (330, y0 + 154), (90, 90, 90), 1)
        cv2.rectangle(img, (112, y0 + 136), (112 + int(216 * max(0, min(100, b)) / 100), y0 + 152),
                      GREEN if b > 30 else (60, 60, 230), -1)
        put(img, f'{b:.0f}%', (342, y0 + 151), 0.55, (255, 255, 255))
        put(img, f'{self.speed:g}x time-lapse', (W - 200, y0 + 170), 0.5, (150, 150, 150))
        return img

    def title_card(self, lines, sub=None):
        img = np.full((H, W, 3), BG, np.uint8)
        cv2.rectangle(img, (0, 0), (W, 8), ACC, -1)
        y = 260
        for i, (t, sc, col) in enumerate(lines):
            tw = cv2.getTextSize(t, FONT, sc, 2)[0][0]
            put(img, t, ((W - tw) // 2, y), sc, col, 2)
            y += int(sc * 60)
        if sub:
            for t in sub:
                tw = cv2.getTextSize(t, FONT, 0.7, 1)[0][0]
                put(img, t, ((W - tw) // 2, y + 20), 0.7, (200, 200, 200))
                y += 34
        return img


def open_writer(path, fps):
    for cc in ('avc1', 'H264', 'mp4v'):
        w = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*cc), fps, (W, H))
        if w.isOpened():
            return w, cc
    raise RuntimeError('no video codec available')


def score(truth, found):
    """Mean error of reported locations to the nearest unmatched true person / path."""
    def dist(p, q):
        pts = q['points']
        if len(pts) == 1:
            return math.hypot(p[0] - pts[0][0], p[1] - pts[0][1])
        (ax, ay), (bx, by) = pts
        dx, dy = bx - ax, by - ay
        t = max(0, min(1, ((p[0] - ax) * dx + (p[1] - ay) * dy) / (dx * dx + dy * dy)))
        return math.hypot(p[0] - ax - t * dx, p[1] - ay - t * dy)
    free, errs = list(range(len(truth))), []
    for f in found:
        best = min(free, key=lambda i: dist(f, truth[i]), default=None)
        if best is not None and dist(f, truth[best]) <= 1.5:
            errs.append(dist(f, truth[best]))
            free.remove(best)
    return len(errs), errs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', default='sage_rescue')
    ap.add_argument('--out', required=True)
    ap.add_argument('--mission', default='Find all people in this area and report their locations')
    ap.add_argument('--speed', type=float, default=4.0, help='time-lapse factor (one frame per YOLO result = 5 Hz, played at 20 fps)')
    ap.add_argument('--tail-s', type=float, default=25.0)
    ap.add_argument('--planner-log', default='/tmp/sage_logs/planner.log')
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args()
    os.makedirs(os.path.join(a.out, 'stills'), exist_ok=True)
    sc = Scene(a.world, a.mission, a.speed)
    writer, codec = open_writer(os.path.join(a.out, 'demo.mp4'), 20)
    still_n = {'k': 0}
    last_frame = [None]

    def still(name, img):
        cv2.imwrite(os.path.join(a.out, 'stills', name + '.png'), img)

    def emit(img, n=1):
        for _ in range(n):
            writer.write(img)

    emit(sc.title_card([('SAGE-UAV', 2.4, (255, 255, 255)),
                        ('Semantic AI-Guided Exploration & Active Search', 0.95, (210, 210, 210))],
                       ['Autonomous search-and-rescue drone  |  ROS 2 - PX4 - Gazebo - YOLO',
                        f'Mission: "{a.mission}"']), 20 * 4)

    if a.selftest:
        sc.events.update(accepted=True, wp=(6, 9))
        sc.pose = (2.0, -3.0, -2.0, 0.8)
        sc.trail = [(-9 + i * 0.4, -9 + i * 0.3) for i in range(30)] + [(2.0, -3.0)]
        sc.tracks = [(0.4, 4.6), (-8.7, 9.3)]
        sc.verified = [(0.1, 4.1)]
        sc.batt = 87.0
        sc.boxes, sc.boxes_t = [(500, 560, 60, 150, 0.83)], time.time()
        cam = np.full((960, 1280, 3), (120, 160, 110), np.uint8)
        cv2.rectangle(cam, (0, 0), (1280, 380), (200, 170, 130), -1)
        frame = sc.compose(cam, sc.boxes)
        still('selftest', frame)
        emit(frame, 48)
        emit(sc.title_card([('MISSION COMPLETE', 1.6, GREEN)], ['3 / 3 people found']), 48)
        writer.release()
        print('selftest written to', a.out, 'codec', codec)
        return

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import Float32MultiArray
    from vision_msgs.msg import Detection2DArray, Detection3DArray
    from px4_msgs.msg import VehicleLocalPosition

    class Rec(Node):
        def __init__(self):
            super().__init__('sage_demo_recorder')
            q = qos_profile_sensor_data
            p = f'/world/{a.world}/model/x500_mono_cam_0/link/camera_link/sensor/imager/image'
            self.create_subscription(Image, '/sage/perception/annotated', self.on_annotated, q)
            self.create_subscription(Detection2DArray, '/sage/perception/detections', self.on_det, q)
            self.create_subscription(Detection3DArray, '/sage/world_model/targets', self.on_tracks, q)
            self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.on_pos, q)
            self.create_subscription(Float32MultiArray, '/sage/energy/status', self.on_energy, q)
            self.ring = collections.OrderedDict()
            self.last_frame = 0.0
            self.log_pos = 0
            self.t_done = None
            self.first_det_saved = False
            self.frames = 0

        def on_pos(self, m):
            if not (m.xy_valid and m.z_valid):
                return
            sc.pose = (float(m.x), float(m.y), float(m.z), float(m.heading))
            if not sc.trail or math.hypot(sc.trail[-1][0] - m.x, sc.trail[-1][1] - m.y) > 0.25:
                sc.trail.append((float(m.x), float(m.y)))
            if sc.events['complete'] and m.z > -0.3:
                sc.events['landed'] = True

        def on_energy(self, m):
            if len(m.data) >= 1:
                b = float(m.data[0])
                sc.batt = b * 100.0 if b <= 1.0 else b

        def on_det(self, m):
            boxes = [(d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.size_x, d.bbox.size_y,
                      d.results[0].hypothesis.score if d.results else 0.0) for d in m.detections]
            sc.boxes, sc.boxes_t = boxes, time.time()
            self.poll_log()
            return

        def on_annotated(self, m):
            """Frames arrive from yolo_detector with the boxes already drawn on the analysed image."""
            self.poll_log()
            arr = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)[:, :, :3]
            frame = sc.compose(cv2.resize(arr, (1280, 960)), ())
            writer.write(frame)
            self.frames += 1
            if sc.boxes and not self.first_det_saved:
                still('first_detection', frame)
                self.first_det_saved = True
            if self.pending_still:
                still(self.pending_still, frame)
                self.pending_still = None
            if self.frames == 40:
                still('search_start', frame)
            last_frame[0] = frame

        def on_tracks(self, m):
            sc.tracks = [(d.results[0].pose.pose.position.x, d.results[0].pose.pose.position.y)
                         for d in m.detections if d.results]
            sc.events['localizing'] = time.time() if sc.tracks else sc.events['localizing']

        def poll_log(self):
            try:
                with open(a.planner_log, errors='ignore') as f:
                    f.seek(self.log_pos)
                    chunk = f.read()
                    self.log_pos = f.tell()
            except OSError:
                return
            for line in chunk.splitlines():
                if 'MISSION ACCEPTED' in line:
                    sc.events['accepted'] = True
                    sc.t0 = time.time()
                m = re.search(r'COVERAGE WAYPOINT \| (\d+)/(\d+) at \(([-\d.]+), ([-\d.]+)\)', line)
                if m:
                    sc.events['wp'] = (int(m[1]), int(m[2]))
                    sc.visited.append((float(m[3]), float(m[4])))
                if 'CANDIDATE FOUND' in line:
                    sc.events['candidate'] = True
                if re.search(r'CANDIDATE REJECTED|DUPLICATE', line):
                    sc.events['candidate'] = False
                m = re.search(r'TARGET VERIFIED \| id=\d+ \| position=\(([-\d.]+), ([-\d.]+)\)', line)
                if m:
                    sc.events['candidate'] = False
                    sc.verified.append((float(m[1]), float(m[2])))
                    self.pending_still = f'verified_{len(sc.verified)}'
                m = re.search(r'MISSION COMPLETE \| status=(\S+) \| found=(\d+) \| locations=(.*?) \| duration=(\d+) s', line)
                if m:
                    sc.events['complete'] = True
                    sc.report = {'status': m[1], 'found': int(m[2]), 'duration_s': int(m[4]),
                                 'locations': [(float(x), float(y)) for x, y in
                                               re.findall(r'\(([-\d.]+), ([-\d.]+)\)', m[3])]}
                    self.t_done = time.time()

        pending_still = None


    rclpy.init()
    node = Rec()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.t_done and time.time() - node.t_done > a.tail_s:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.poll_log()
        last = last_frame[0] if last_frame[0] is not None else sc.compose(None)
        if sc.report:
            tp, errs = score(sc.truth, sc.report['locations'])
            mean = sum(errs) / len(errs) if errs else float('nan')
            emit(last, 20)
            still('mission_complete', last)
            card = sc.title_card(
                [('MISSION COMPLETE', 1.8, GREEN),
                 (f"{tp} / {len(sc.truth)} people found", 1.2, (255, 255, 255))],
                [f"mean position error {mean:.2f} m   |   mission time {sc.report['duration_s']} s   |   "
                 f"battery {sc.batt if sc.batt is not None else 0:.0f}% left",
                 (f"{len(sc.report['locations']) - tp} extra report(s): the walking person was reported twice"
                  if len(sc.report['locations']) > tp else 'Returned home and landed autonomously')])
            still('result_card', card)
            emit(card, 20 * 5)
            json.dump({'world': a.world, 'report': sc.report, 'true_positives': tp, 'errors_m': errs,
                       'codec': codec, 'frames': node.frames}, open(os.path.join(a.out, 'summary.json'), 'w'))
        writer.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
