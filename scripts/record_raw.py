#!/usr/bin/env python3
"""Lightweight in-flight logger: saves JPEG frames, poses, detections and battery to disk.
No composing, no video encoding, so the simulator is not slowed down. Videos are rendered afterwards
by scripts/render_demo.py.   usage: python3 scripts/record_raw.py --out results/raw/<run_id> [--tail-s 25]
Files: cam/<ns>.jpg (YOLO-annotated onboard frames), top/<ns>.jpg (Gazebo overhead), pose.jsonl, energy.jsonl
Time base: the ROS/system clock, the same clock as the [..] stamps in planner.log."""
import argparse
import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray
from px4_msgs.msg import VehicleLocalPosition

ap = argparse.ArgumentParser()
ap.add_argument('--out', required=True)
ap.add_argument('--tail-s', type=float, default=25.0)
ap.add_argument('--planner-log', default='/tmp/sage_logs/planner.log')
a = ap.parse_args()
for d in ('cam', 'top'):
    os.makedirs(os.path.join(a.out, d), exist_ok=True)


class Raw(Node):
    def __init__(self):
        super().__init__('sage_raw_recorder')
        q = qos_profile_sensor_data
        self.create_subscription(CompressedImage, '/sage/perception/annotated/compressed', self.on_cam, q)
        self.create_subscription(CompressedImage, '/uav_top_cam/compressed', self.on_top, q)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.on_pos, q)
        self.create_subscription(Float32MultiArray, '/sage/energy/status', self.on_energy, q)
        self.pose = open(os.path.join(a.out, 'pose.jsonl'), 'a')
        self.energy = open(os.path.join(a.out, 'energy.jsonl'), 'a')
        self.last_pose = 0.0
        self.t_done = None
        self.pos = 0
        self.create_timer(1.0, self.check_done)

    def ns(self):
        return self.get_clock().now().nanoseconds

    def save(self, kind, m):
        with open(os.path.join(a.out, kind, f'{self.ns()}.jpg'), 'wb') as f:
            f.write(bytes(m.data))

    def on_cam(self, m):
        self.save('cam', m)

    def on_top(self, m):
        self.save('top', m)

    def on_pos(self, m):
        t = self.ns()
        if t - self.last_pose < 80_000_000 or not (m.xy_valid and m.z_valid):
            return
        self.last_pose = t
        self.pose.write(json.dumps([t, float(m.x), float(m.y), float(m.z), float(m.heading)]) + '\n')
        self.pose.flush()

    def on_energy(self, m):
        if len(m.data) >= 1:
            self.energy.write(json.dumps([self.ns(), float(m.data[0])]) + '\n')
            self.energy.flush()

    def check_done(self):
        try:
            with open(a.planner_log, errors='ignore') as f:
                f.seek(self.pos)
                chunk = f.read()
                self.pos = f.tell()
        except OSError:
            return
        if 'MISSION COMPLETE' in chunk and self.t_done is None:
            self.t_done = time.time()


rclpy.init()
node = Raw()
try:
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.5)
        if node.t_done and time.time() - node.t_done > a.tail_s:
            break
except KeyboardInterrupt:
    pass
node.pose.close()
node.energy.close()
node.destroy_node()
rclpy.shutdown()
