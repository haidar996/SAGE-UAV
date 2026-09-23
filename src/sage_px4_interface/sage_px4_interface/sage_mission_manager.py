#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from px4_msgs.msg import VehicleLocalPosition


class SageMissionManager(Node):

    def __init__(self):
        super().__init__('sage_mission_manager')

        # PX4 uORB topics use BEST_EFFORT reliability.
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.viewpoint_sub = self.create_subscription(
            PoseStamped,
            '/sage/planning/viewpoint',
            self.viewpoint_callback,
            10
        )

        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_position_callback,
            px4_qos
        )

        # Validated mission output.
        # This does NOT publish any PX4 flight commands.
        self.validated_viewpoint_pub = self.create_publisher(
            PoseStamped,
            '/sage/mission/validated_viewpoint',
            10
        )

        # Energy-driven return home (Step 20c).
        self.return_home_sub = self.create_subscription(
            Bool,
            '/sage/energy/return_home',
            self.return_home_callback,
            10
        )

        # Mission completion (planner) also returns home.
        self.mission_return_sub = self.create_subscription(
            Bool,
            '/sage/mission/return_home',
            self.return_home_callback,
            10
        )

        self.land_request_pub = self.create_publisher(
            Bool,
            '/sage/mission/land_request',
            10
        )

        self.returning_home = False
        self.land_requested = False
        self.home_position = (0.0, 0.0, -2.0)
        self.return_step = 1.5
        self.home_tolerance = 0.4

        self.return_timer = self.create_timer(
            0.2,
            self.return_timer_callback
        )

        self.vehicle_position = None
        self.last_accepted_viewpoint = None

        # Safety limits
        self.min_altitude = -5.0
        self.max_altitude = -1.0

        self.max_horizontal_distance = 8.0
        self.max_viewpoint_jump = 3.0
        self.max_viewpoint_age = 1.0

        # If no viewpoint was accepted for this long, the previous
        # viewpoint is no longer a valid reference for the jump check
        # (e.g. the planner was restarted).
        self.last_accepted_timeout = 3.0
        self.last_accepted_time = None

        self.get_logger().info(
            'SAGE Mission Manager started.'
        )

        self.get_logger().info(
            'Validation-only mode: NO PX4 flight commands.'
        )

        self.get_logger().info(
            'Validated viewpoint output: '
            '/sage/mission/validated_viewpoint'
        )

    def local_position_callback(self, msg):
        if not msg.xy_valid or not msg.z_valid:
            return

        self.vehicle_position = (
            float(msg.x),
            float(msg.y),
            float(msg.z)
        )

    def return_home_callback(self, msg):
        if msg.data and not self.returning_home:
            self.returning_home = True
            self.last_accepted_viewpoint = None
            self.get_logger().warn(
                'RETURN HOME ACTIVE | planner viewpoints ignored.'
            )

    def return_timer_callback(self):
        if not self.returning_home or self.vehicle_position is None:
            return

        ux, uy, uz = self.vehicle_position
        hx, hy, hz = self.home_position

        distance = math.hypot(hx - ux, hy - uy)

        if distance <= self.home_tolerance:
            if not self.land_requested:
                self.land_requested = True
                self.get_logger().warn(
                    'HOME REACHED | requesting LAND.'
                )
            land = Bool()
            land.data = True
            self.land_request_pub.publish(land)
            return

        scale = min(1.0, self.return_step / distance)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'px4_local_ned'
        msg.pose.position.x = ux + (hx - ux) * scale
        msg.pose.position.y = uy + (hy - uy) * scale
        msg.pose.position.z = hz
        msg.pose.orientation.w = 1.0

        self.validated_viewpoint_pub.publish(msg)

    def viewpoint_callback(self, msg):

        # Return home overrides the planner completely.
        if self.returning_home:
            return

        # ---------------------------------------------------------
        # 1. Frame validation
        # ---------------------------------------------------------

        if msg.header.frame_id != 'px4_local_ned':
            self.reject(
                f'invalid frame: {msg.header.frame_id}'
            )
            return

        # ---------------------------------------------------------
        # 2. Position validity
        # ---------------------------------------------------------

        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        z = float(msg.pose.position.z)

        if not all(math.isfinite(v) for v in (x, y, z)):
            self.reject('non-finite position')
            return

        # ---------------------------------------------------------
        # 3. Altitude limits
        # ---------------------------------------------------------

        if z < self.min_altitude or z > self.max_altitude:
            self.reject(
                f'altitude outside limits: z={z:.2f} m'
            )
            return

        # ---------------------------------------------------------
        # 4. UAV position must be available
        # ---------------------------------------------------------

        if self.vehicle_position is None:
            self.reject(
                'UAV position not available'
            )
            return

        ux, uy, uz = self.vehicle_position

        # ---------------------------------------------------------
        # 5. Horizontal distance from UAV
        # ---------------------------------------------------------

        horizontal_distance = math.hypot(
            x - ux,
            y - uy
        )

        if horizontal_distance > self.max_horizontal_distance:
            self.reject(
                'viewpoint too far from UAV: '
                f'{horizontal_distance:.2f} m'
            )
            return

        # ---------------------------------------------------------
        # 6. Viewpoint jump validation
        # ---------------------------------------------------------

        if (
            self.last_accepted_viewpoint is not None
            and self.last_accepted_time is not None
            and (
                self.get_clock().now() - self.last_accepted_time
            ).nanoseconds / 1e9 > self.last_accepted_timeout
        ):
            self.get_logger().info(
                'Previous viewpoint is stale: '
                'jump check reset.'
            )
            self.last_accepted_viewpoint = None

        if self.last_accepted_viewpoint is not None:

            last_x, last_y, last_z = (
                self.last_accepted_viewpoint
            )

            jump_distance = math.sqrt(
                (x - last_x) ** 2 +
                (y - last_y) ** 2 +
                (z - last_z) ** 2
            )

            if jump_distance > self.max_viewpoint_jump:
                self.reject(
                    'viewpoint jump too large: '
                    f'{jump_distance:.2f} m'
                )
                return

        # ---------------------------------------------------------
        # 7. Quaternion validation
        # ---------------------------------------------------------

        qx = float(msg.pose.orientation.x)
        qy = float(msg.pose.orientation.y)
        qz = float(msg.pose.orientation.z)
        qw = float(msg.pose.orientation.w)

        if not all(
            math.isfinite(v)
            for v in (qx, qy, qz, qw)
        ):
            self.reject('non-finite orientation')
            return

        quaternion_norm = math.sqrt(
            qx * qx +
            qy * qy +
            qz * qz +
            qw * qw
        )

        if quaternion_norm < 1e-6:
            self.reject(
                'invalid zero-length quaternion'
            )
            return

        # ---------------------------------------------------------
        # 8. Accept viewpoint
        # ---------------------------------------------------------

        self.last_accepted_viewpoint = (
            x,
            y,
            z
        )
        self.last_accepted_time = self.get_clock().now()

        # Publish only validated viewpoints.
        # This remains completely independent of PX4 commands.
        self.validated_viewpoint_pub.publish(msg)

        self.get_logger().info(
            'VIEWPOINT ACCEPTED | '
            f'position=({x:.2f}, {y:.2f}, {z:.2f}) | '
            f'distance_from_uav={horizontal_distance:.2f} m'
        )

        self.get_logger().info(
            'VALIDATED VIEWPOINT PUBLISHED'
        )

    def reject(self, reason):
        self.get_logger().warn(
            f'VIEWPOINT REJECTED | {reason}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = SageMissionManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
