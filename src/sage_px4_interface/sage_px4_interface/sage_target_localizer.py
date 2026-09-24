#!/usr/bin/env python3

import math
import os
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from geometry_msgs.msg import PointStamped
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2DArray

from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleAttitude


WORLD = os.environ.get('SAGE_WORLD', 'sage_test')


class SageTargetLocalizer(Node):

    def __init__(self):
        super().__init__('sage_target_localizer')

        # =========================================================
        # Camera intrinsics
        # =========================================================

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # =========================================================
        # PX4 vehicle state
        # =========================================================

        self.vehicle_x = None
        self.vehicle_y = None
        self.vehicle_z = None
        self.vehicle_heading = None

        # Full attitude quaternion (w, x, y, z), body FRD -> NED.
        # With use_attitude the ray is rotated by roll/pitch/yaw
        # instead of heading only (False = old behaviour, for A/B).
        self.declare_parameter('use_attitude', True)
        self.use_attitude = bool(
            self.get_parameter('use_attitude').value
        )
        self.vehicle_q = None

        # SITL-only: 'gz_truth' takes the attitude from Gazebo ground
        # truth (needs ros_gz_bridge of /world/<world>/dynamic_pose/info as
        # TFMessage) because PX4's yaw estimate is off in this sim.
        # 'px4' (default) uses the estimator - the real-world path.
        self.declare_parameter('attitude_source', 'px4')
        self.attitude_source = str(
            self.get_parameter('attitude_source').value
        )
        self.gz_model_name = 'x500_mono_cam_0'
        # (t_s, n, e, d, q) history of the Gazebo pose, node clock.
        self.gz_history = deque(maxlen=400)

        # Skip localization while the UAV is tilted (range error grows
        # fast with pitch/roll error and image/pose time skew).
        self.declare_parameter('max_tilt_deg', 8.0)
        self.max_tilt_deg = float(
            self.get_parameter('max_tilt_deg').value
        )
        self.last_tilt_warn = 0.0

        # =========================================================
        # Camera mounting position relative to PX4 body
        #
        # From x500_mono_cam model:
        #
        # x = 0.12
        # y = 0.03
        # z = 0.242
        # =========================================================

        self.camera_body_x = 0.12
        self.camera_body_y = 0.03
        self.camera_body_z = 0.242

        # =========================================================
        # Detection
        # =========================================================

        self.latest_detection = None

        # Use an internal point near the bottom of the bbox.
        #
        # 0.50 = bbox center
        # 0.90 = 90% from top toward bottom
        #
        self.anchor_fraction = 0.90

        # =========================================================
        # Localization safety parameters
        # =========================================================

        # Don't attempt ground localization when the camera is
        # almost touching the ground.
        #
        # Camera height above ground must exceed this value.
        self.min_camera_height = 0.30

        # Reject rays that are too close to horizontal.
        #
        # ned_z is the downward component of the ray.
        self.min_downward_component = 0.05

        # Don't accept targets absurdly far from the UAV.
        self.max_target_distance = 30.0

        # =========================================================
        # PX4 QoS
        # =========================================================

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # =========================================================
        # Subscribers
        # =========================================================

        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/sage/perception/detections',
            self.detection_callback,
            10,
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            f'/world/{WORLD}/model/'
            'x500_mono_cam_0/link/camera_link/'
            'sensor/imager/camera_info',
            self.camera_info_callback,
            10,
        )

        self.vehicle_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_position_callback,
            px4_qos,
        )

        # =========================================================
        # Publisher
        # =========================================================

        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            ),
        )

        if self.attitude_source == 'gz_truth':
            self.gz_pose_sub = self.create_subscription(
                TFMessage,
                f'/world/{WORLD}/dynamic_pose/info',
                self.gz_pose_callback,
                10,
            )
            self.get_logger().warn(
                'attitude_source=gz_truth (SITL ground truth, '
                'not the PX4 estimate).'
            )

        self.target_pub = self.create_publisher(
            PointStamped,
            '/sage/perception/target_position',
            10,
        )

        # =========================================================
        # Timer
        # =========================================================

        self.timer = self.create_timer(
            0.1,
            self.localize_target,
        )

        self.get_logger().info(
            'SAGE 3D target localizer started.'
        )

        self.get_logger().info(
            f'BBox anchor fraction = '
            f'{self.anchor_fraction:.2f}'
        )

        self.get_logger().info(
            f'Minimum camera height = '
            f'{self.min_camera_height:.2f} m'
        )

        self.get_logger().info(
            f'Minimum downward ray component = '
            f'{self.min_downward_component:.3f}'
        )

        self.get_logger().info(
            'Waiting for camera/PX4 data...'
        )

    # =============================================================
    # CameraInfo
    # =============================================================

    def camera_info_callback(self, msg):

        self.fx = msg.k[0]
        self.fy = msg.k[4]

        self.cx = msg.k[2]
        self.cy = msg.k[5]

    # =============================================================
    # Detection
    # =============================================================

    def detection_callback(self, msg):

        if not msg.detections:
            self.latest_detection = None
            return

        self.latest_detection = msg.detections[0]

    # =============================================================
    # PX4 local position
    # =============================================================

    def vehicle_position_callback(self, msg):

        if not msg.xy_valid or not msg.z_valid:
            return

        self.vehicle_x = msg.x
        self.vehicle_y = msg.y
        self.vehicle_z = msg.z

        self.vehicle_heading = msg.heading

    def gz_pose_callback(self, msg):
        # The bridged TFMessage carries no frame names; in
        # dynamic_pose/info the first entry is the UAV model itself.
        for tr in msg.transforms[:1]:
            r = tr.transform.rotation

            # ENU/FLU quaternion -> NED/FRD quaternion.
            # q_ned_frd = q_ned_enu * q_enu_flu * q_flu_frd
            # q_ned_enu = (0, s, s, 0), s = 1/sqrt(2)  (x<->y, z flip)
            # q_flu_frd = (0, 1, 0, 0)                 (180 deg about x)
            def mul(a, b):
                aw, ax, ay, az = a
                bw, bx, by, bz = b
                return (
                    aw * bw - ax * bx - ay * by - az * bz,
                    aw * bx + ax * bw + ay * bz - az * by,
                    aw * by - ax * bz + ay * bw + az * bx,
                    aw * bz + ax * by - ay * bx + az * bw,
                )

            s2 = math.sqrt(0.5)
            q = (r.w, r.x, r.y, r.z)
            q = mul((0.0, s2, s2, 0.0), q)
            q = mul(q, (0.0, 1.0, 0.0, 0.0))
            self.vehicle_q = q

            t = tr.transform.translation
            self.gz_history.append((
                self.get_clock().now().nanoseconds / 1e9,
                t.y, t.x, -t.z, q
            ))
            return

    def gz_pose_at(self, t):
        """Gazebo pose (n, e, d, q) at node-clock time t (s), linearly
        interpolated; None if the history does not cover t."""
        h = self.gz_history

        if len(h) < 2 or t < h[0][0] or t > h[-1][0] + 0.05:
            return None

        prev = h[0]

        for cur in h:
            if cur[0] >= t:
                break
            prev = cur

        if cur[0] <= prev[0]:
            return cur[1], cur[2], cur[3], cur[4]

        a = (t - prev[0]) / (cur[0] - prev[0])
        a = max(0.0, min(1.0, a))

        n = prev[1] + a * (cur[1] - prev[1])
        e = prev[2] + a * (cur[2] - prev[2])
        d = prev[3] + a * (cur[3] - prev[3])

        # nlerp (shortest arc)
        q0, q1 = prev[4], cur[4]
        sign = 1.0 if sum(x * y for x, y in zip(q0, q1)) >= 0 else -1.0
        q = tuple(x + a * (sign * y - x) for x, y in zip(q0, q1))
        norm = math.sqrt(sum(x * x for x in q))

        return n, e, d, tuple(x / norm for x in q)

    def attitude_callback(self, msg):
        if self.attitude_source == 'gz_truth':
            return

        self.vehicle_q = (
            msg.q[0], msg.q[1], msg.q[2], msg.q[3]
        )

    @staticmethod
    def rotate_body_to_ned(q, v):
        w, x, y, z = q
        n = math.sqrt(w * w + x * x + y * y + z * z)
        w, x, y, z = w / n, x / n, y / n, z / n

        r = (
            (1 - 2 * (y * y + z * z),
             2 * (x * y - w * z),
             2 * (x * z + w * y)),
            (2 * (x * y + w * z),
             1 - 2 * (x * x + z * z),
             2 * (y * z - w * x)),
            (2 * (x * z - w * y),
             2 * (y * z + w * x),
             1 - 2 * (x * x + y * y)),
        )

        return tuple(
            r[i][0] * v[0] + r[i][1] * v[1] + r[i][2] * v[2]
            for i in range(3)
        )

    # =============================================================
    # Localization
    # =============================================================

    def localize_target(self):

        # ---------------------------------------------------------
        # Check detection
        # ---------------------------------------------------------

        if self.latest_detection is None:
            return

        # ---------------------------------------------------------
        # Check camera intrinsics
        # ---------------------------------------------------------

        if (
            self.fx is None
            or self.fy is None
            or self.cx is None
            or self.cy is None
        ):
            return

        # ---------------------------------------------------------
        # Check PX4 state
        # ---------------------------------------------------------

        if (
            self.vehicle_x is None
            or self.vehicle_y is None
            or self.vehicle_z is None
        ):
            return

        if self.vehicle_heading is None:
            return

        detection = self.latest_detection

        # Ground-truth mode: use the Gazebo pose at the time the frame
        # was captured (YOLO adds ~0.3 s latency; the UAV moves meanwhile).
        if self.attitude_source == 'gz_truth':
            stamp = detection.header.stamp
            pose = self.gz_pose_at(stamp.sec + stamp.nanosec * 1e-9)

            if pose is None:
                return

            (
                self.vehicle_x, self.vehicle_y, self.vehicle_z,
                self.vehicle_q
            ) = pose

        # ---------------------------------------------------------
        # Bounding box
        # ---------------------------------------------------------

        bbox_center_x = detection.bbox.center.position.x
        bbox_center_y = detection.bbox.center.position.y

        bbox_width = detection.bbox.size_x
        bbox_height = detection.bbox.size_y

        if bbox_width <= 0.0 or bbox_height <= 0.0:
            return

        # ---------------------------------------------------------
        # Camera height above ground
        #
        # PX4 NED:
        #
        # vehicle_z < 0 when above ground
        #
        # camera is 0.242 m below/above body according to
        # the model's local Z convention.
        # ---------------------------------------------------------

        camera_ned_z = (
            self.vehicle_z
            - self.camera_body_z
        )

        camera_height = -camera_ned_z

        # ---------------------------------------------------------
        # Safety: UAV too close to ground
        # ---------------------------------------------------------

        if camera_height < self.min_camera_height:

            self.get_logger().warn(
                f'3D localization skipped: '
                f'camera too close to ground | '
                f'vehicle_z={self.vehicle_z:.3f} | '
                f'camera_height={camera_height:.3f}'
            )

            return

        # ---------------------------------------------------------
        # BBox anchor
        # ---------------------------------------------------------

        v = (
            bbox_center_y
            + (
                self.anchor_fraction - 0.50
            ) * bbox_height
        )

        u = bbox_center_x

        # ---------------------------------------------------------
        # Image ray
        # ---------------------------------------------------------

        ray_x = (
            u - self.cx
        ) / self.fx

        ray_y = (
            v - self.cy
        ) / self.fy

        ray_z = 1.0

        # ---------------------------------------------------------
        # Normalize
        # ---------------------------------------------------------

        ray_norm = math.sqrt(
            ray_x * ray_x
            + ray_y * ray_y
            + ray_z * ray_z
        )

        if ray_norm <= 1e-9:
            return

        ray_x /= ray_norm
        ray_y /= ray_norm
        ray_z /= ray_norm

        # ---------------------------------------------------------
        # Camera optical frame -> PX4 body frame
        #
        # Camera optical:
        #
        # X = right
        # Y = down
        # Z = forward
        #
        # PX4 body:
        #
        # X = forward
        # Y = right
        # Z = down
        #
        # ---------------------------------------------------------

        body_x = ray_z
        body_y = ray_x
        body_z = ray_y

        # ---------------------------------------------------------
        # Rotate body frame into PX4 local NED
        #
        # Heading rotates body X/Y into local X/Y.
        # ---------------------------------------------------------

        cos_h = math.cos(
            self.vehicle_heading
        )

        sin_h = math.sin(
            self.vehicle_heading
        )

        ned_x = (
            cos_h * body_x
            - sin_h * body_y
        )

        ned_y = (
            sin_h * body_x
            + cos_h * body_y
        )

        ned_z = body_z

        use_q = self.use_attitude and self.vehicle_q is not None

        if use_q:
            # Tilt = angle between body Z and NED down.
            _, _, tz = self.rotate_body_to_ned(
                self.vehicle_q, (0.0, 0.0, 1.0)
            )
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, tz))))

            if tilt > self.max_tilt_deg:
                now = self.get_clock().now().nanoseconds / 1e9

                if now - self.last_tilt_warn > 5.0:
                    self.last_tilt_warn = now
                    self.get_logger().warn(
                        f'3D localization skipped: UAV tilt '
                        f'{tilt:.1f} deg > {self.max_tilt_deg:.1f}'
                    )

                return

        if use_q:
            ned_x, ned_y, ned_z = self.rotate_body_to_ned(
                self.vehicle_q, (body_x, body_y, body_z)
            )

        # ---------------------------------------------------------
        # IMPORTANT:
        #
        # For ground intersection the ray must point DOWN.
        #
        # In PX4 NED:
        #
        # positive Z = down
        #
        # ---------------------------------------------------------

        if ned_z <= self.min_downward_component:

            self.get_logger().warn(
                f'3D localization skipped: '
                f'ray too close to horizontal/upward | '
                f'bbox_center_y={bbox_center_y:.1f} | '
                f'bbox_height={bbox_height:.1f} | '
                f'anchor_v={v:.1f} | '
                f'cy={self.cy:.1f} | '
                f'ray_y={ray_y:.4f} | '
                f'ned_z={ned_z:.4f}'
            )

            return

        # ---------------------------------------------------------
        # Camera position in PX4 local NED
        # ---------------------------------------------------------

        camera_ned_x = (
            self.vehicle_x
            + cos_h * self.camera_body_x
            - sin_h * self.camera_body_y
        )

        camera_ned_y = (
            self.vehicle_y
            + sin_h * self.camera_body_x
            + cos_h * self.camera_body_y
        )

        # camera_ned_z already calculated above

        if use_q:
            ox, oy, oz = self.rotate_body_to_ned(
                self.vehicle_q,
                (self.camera_body_x, self.camera_body_y,
                 -self.camera_body_z)
            )
            camera_ned_x = self.vehicle_x + ox
            camera_ned_y = self.vehicle_y + oy
            camera_ned_z = self.vehicle_z + oz

        # ---------------------------------------------------------
        # Ground intersection
        #
        # Ground Z = 0
        #
        # P = camera + scale * ray
        #
        # 0 = camera_z + scale * ray_z
        #
        # scale = -camera_z / ray_z
        # ---------------------------------------------------------

        scale = (
            -camera_ned_z
        ) / ned_z

        if scale <= 0:

            self.get_logger().warn(
                f'3D localization stopped: '
                f'ground intersection behind camera | '
                f'bbox_center_y={bbox_center_y:.1f} | '
                f'bbox_height={bbox_height:.1f} | '
                f'anchor_v={v:.1f} | '
                f'cy={self.cy:.1f} | '
                f'ray_y={ray_y:.4f} | '
                f'ned_z={ned_z:.4f} | '
                f'camera_ned_z={camera_ned_z:.4f} | '
                f'scale={scale:.4f}'
            )

            return

        # ---------------------------------------------------------
        # Maximum range protection
        # ---------------------------------------------------------

        horizontal_distance = math.sqrt(
            (scale * ned_x) ** 2
            + (scale * ned_y) ** 2
        )

        if (
            horizontal_distance
            > self.max_target_distance
        ):

            self.get_logger().warn(
                f'3D localization rejected: '
                f'target too far | '
                f'distance={horizontal_distance:.2f} m | '
                f'ned_z={ned_z:.4f} | '
                f'scale={scale:.2f}'
            )

            return

        # ---------------------------------------------------------
        # Target position
        # ---------------------------------------------------------

        target_x = (
            camera_ned_x
            + scale * ned_x
        )

        target_y = (
            camera_ned_y
            + scale * ned_y
        )

        target_z = 0.0

        # ---------------------------------------------------------
        # Publish
        # ---------------------------------------------------------

        msg = PointStamped()

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = (
            'px4_local_ned'
        )

        msg.point.x = target_x
        msg.point.y = target_y
        msg.point.z = target_z

        self.target_pub.publish(msg)

        # ---------------------------------------------------------
        # Debug
        # ---------------------------------------------------------

        self.get_logger().info(
            f'Person 3D | '
            f'pixel=({u:.1f},{v:.1f}) | '
            f'position=('
            f'{target_x:.2f},'
            f'{target_y:.2f},'
            f'{target_z:.2f}) | '
            f'range={horizontal_distance:.2f} m'
        )


def main(args=None):

    rclpy.init(args=args)

    node = SageTargetLocalizer()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
