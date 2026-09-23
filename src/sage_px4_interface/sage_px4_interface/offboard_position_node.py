#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)


class OffboardPositionNode(Node):

    def __init__(self):
        super().__init__('offboard_position_node')

        # =========================================================
        # PX4 QoS
        # =========================================================

        px4_pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # =========================================================
        # Publishers
        # =========================================================

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            px4_pub_qos
        )

        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            px4_pub_qos
        )

        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            px4_pub_qos
        )

        # =========================================================
        # Subscribers
        # =========================================================

        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_position_callback,
            px4_qos
        )

        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            px4_qos
        )

        # ---------------------------------------------------------
        # NEW:
        # Receive only Mission Manager validated viewpoints.
        # ---------------------------------------------------------

        self.validated_viewpoint_sub = self.create_subscription(
            PoseStamped,
            '/sage/mission/validated_viewpoint',
            self.validated_viewpoint_callback,
            10
        )

        # Landing request from the Mission Manager (energy abort).
        self.land_request_sub = self.create_subscription(
            Bool,
            '/sage/mission/land_request',
            self.land_request_callback,
            10
        )

        self.land_commanded = False
        self.has_been_armed = False
        self.last_offboard_request_time = None
        self.last_arm_request_time = None
        self.disarm_sent = False
        self.ground_since = None

        # PX4's land detector can stay silent in SITL; disarm once the
        # UAV has been on the ground for this long after a LAND request.
        self.ground_height = 0.15
        self.ground_hold_s = 3.0

        # =========================================================
        # Vehicle state
        # =========================================================

        self.current_position = None
        self.vehicle_status = None

        # =========================================================
        # Mission state
        # =========================================================

        # Safe fallback before the first validated viewpoint.
        self.safe_hover_position = [0.0, 0.0, -2.0]
        self.safe_hover_yaw = 1.5708

        # Latest approved viewpoint from Mission Manager.
        self.validated_position = None
        self.validated_yaw = None

        # =========================================================
        # Offboard state
        # =========================================================

        self.offboard_setpoint_counter = 0
        self.offboard_requested = False
        self.arm_requested = False

        self.required_setpoint_count = 20

        # =========================================================
        # Logging state
        # =========================================================

        self.first_validated_viewpoint_logged = False
        self.current_target_logged = False

        # =========================================================
        # 20 Hz control loop
        # =========================================================

        self.timer = self.create_timer(
            0.05,
            self.timer_callback
        )

        self.get_logger().info(
            'SAGE PX4 interface started.'
        )

        self.get_logger().info(
            'Waiting for validated viewpoints on '
            '/sage/mission/validated_viewpoint'
        )

        self.get_logger().info(
            'Safe fallback position: '
            '[0.0, 0.0, -2.0]'
        )

    # =============================================================
    # Subscribers
    # =============================================================

    def local_position_callback(self, msg):
        self.current_position = msg

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def land_request_callback(self, msg):
        if not msg.data or self.land_commanded:
            return

        self.land_commanded = True
        self.get_logger().warn('LAND requested by Mission Manager.')

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND
        )

    def check_disarm_after_land(self):
        if (
            not self.land_commanded
            or self.disarm_sent
            or self.current_position is None
        ):
            return

        # NED: z is negative above home.
        if self.current_position.z < -self.ground_height:
            self.ground_since = None
            return

        now = self.get_clock().now().nanoseconds / 1e9

        if self.ground_since is None:
            self.ground_since = now
            return

        if now - self.ground_since >= self.ground_hold_s:
            self.disarm_sent = True
            self.get_logger().warn('On ground after LAND | DISARM.')
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                0.0,
                21196.0  # force: PX4 land detector is unreliable in SITL
            )

    def validated_viewpoint_callback(self, msg):

        # ---------------------------------------------------------
        # Frame validation
        # ---------------------------------------------------------

        if msg.header.frame_id != 'px4_local_ned':
            self.get_logger().warn(
                'Ignoring validated viewpoint with invalid frame: '
                f'{msg.header.frame_id}'
            )
            return

        # ---------------------------------------------------------
        # Extract position
        # ---------------------------------------------------------

        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        z = float(msg.pose.position.z)

        if not all(
            math.isfinite(v)
            for v in (x, y, z)
        ):
            self.get_logger().warn(
                'Ignoring validated viewpoint with '
                'non-finite position.'
            )
            return

        # ---------------------------------------------------------
        # Extract quaternion
        # ---------------------------------------------------------

        qx = float(msg.pose.orientation.x)
        qy = float(msg.pose.orientation.y)
        qz = float(msg.pose.orientation.z)
        qw = float(msg.pose.orientation.w)

        if not all(
            math.isfinite(v)
            for v in (qx, qy, qz, qw)
        ):
            self.get_logger().warn(
                'Ignoring validated viewpoint with '
                'non-finite orientation.'
            )
            return

        quaternion_norm = math.sqrt(
            qx * qx +
            qy * qy +
            qz * qz +
            qw * qw
        )

        if quaternion_norm < 1e-6:
            self.get_logger().warn(
                'Ignoring validated viewpoint with '
                'zero-length quaternion.'
            )
            return

        # Normalize quaternion before extracting yaw.
        qx /= quaternion_norm
        qy /= quaternion_norm
        qz /= quaternion_norm
        qw /= quaternion_norm

        # ---------------------------------------------------------
        # Quaternion -> yaw
        # ---------------------------------------------------------

        sin_yaw = 2.0 * (
            qw * qz +
            qx * qy
        )

        cos_yaw = 1.0 - 2.0 * (
            qy * qy +
            qz * qz
        )

        yaw = math.atan2(
            sin_yaw,
            cos_yaw
        )

        # ---------------------------------------------------------
        # Store approved viewpoint
        # ---------------------------------------------------------

        self.validated_position = [
            x,
            y,
            z
        ]

        self.validated_yaw = yaw

        # ---------------------------------------------------------
        # Logging
        # ---------------------------------------------------------

        if not self.first_validated_viewpoint_logged:

            self.first_validated_viewpoint_logged = True

            self.get_logger().info(
                'VALIDATED VIEWPOINT RECEIVED'
            )

        if not self.current_target_logged:

            self.current_target_logged = True

            self.get_logger().info(
                'PX4 target updated: '
                f'position=({x:.2f}, {y:.2f}, {z:.2f}), '
                f'yaw={yaw:.2f} rad'
            )

    # =============================================================
    # PX4 publishers
    # =============================================================

    def publish_offboard_control_mode(self):

        msg = OffboardControlMode()

        msg.timestamp = self.get_timestamp()

        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False

        self.offboard_control_mode_pub.publish(msg)

    def publish_position_setpoint(
        self,
        position,
        yaw
    ):

        msg = TrajectorySetpoint()

        msg.timestamp = self.get_timestamp()

        msg.position = position

        msg.velocity = [
            math.nan,
            math.nan,
            math.nan
        ]

        msg.acceleration = [
            math.nan,
            math.nan,
            math.nan
        ]

        msg.jerk = [
            math.nan,
            math.nan,
            math.nan
        ]

        msg.yaw = yaw
        msg.yawspeed = math.nan

        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(
        self,
        command,
        param1=0.0,
        param2=0.0
    ):

        msg = VehicleCommand()

        msg.timestamp = self.get_timestamp()

        msg.param1 = float(param1)
        msg.param2 = float(param2)

        msg.command = command

        msg.target_system = 1
        msg.target_component = 1

        msg.source_system = 1
        msg.source_component = 1

        msg.confirmation = 0
        msg.from_external = True
        self.get_logger().info(
    f'Publishing VehicleCommand: command={command}, '
    f'param1={param1}, param2={param2}'
)
        self.vehicle_command_pub.publish(msg)
        

    # =============================================================
    # Time
    # =============================================================

    def get_timestamp(self):

        return int(
            self.get_clock().now().nanoseconds / 1000
        )

    # =============================================================
    # Main control loop
    # =============================================================

    def timer_callback(self):

        # =========================================================
        # 1. Maintain Offboard heartbeat
        # =========================================================

        self.publish_offboard_control_mode()

        # =========================================================
        # 2. Determine active setpoint
        # =========================================================

        if self.validated_position is not None:

            position = self.validated_position
            yaw = self.validated_yaw

        else:

            # No approved viewpoint yet.
            # Remain at the known-safe hover position.
            position = self.safe_hover_position
            yaw = self.safe_hover_yaw

        # =========================================================
        # 3. Publish PX4 position setpoint
        # =========================================================

        self.publish_position_setpoint(
            position,
            yaw
        )

        self.offboard_setpoint_counter += 1

        # =========================================================
        # 4. Initial startup message
        # =========================================================

        if self.offboard_setpoint_counter == 1:

            self.get_logger().info(
                'Publishing OffboardControlMode + '
                'TrajectorySetpoint.'
            )

        # =========================================================
        # 5. Request OFFBOARD
        # =========================================================

        if (
            self.offboard_setpoint_counter
            >= self.required_setpoint_count
            and not self.offboard_requested
        ):

            self.get_logger().info(
                'Requesting OFFBOARD mode...'
            )

            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                1.0,
                6.0
            )

            self.offboard_requested = True

        # =========================================================
        # 5b. Disarm after landing (energy abort)
        # =========================================================

        self.check_disarm_after_land()

        # =========================================================
        # 6. Wait for vehicle status
        # =========================================================

        if self.vehicle_status is None:
            return

        # ---------------------------------------------------------
        # Retry OFFBOARD until PX4 accepts it (the request can be
        # lost if the DDS link is not fully up yet). Never after the
        # vehicle has flown or a LAND was requested.
        # ---------------------------------------------------------

        now_s = self.get_clock().now().nanoseconds / 1e9

        if (
            self.offboard_requested
            and not self.has_been_armed
            and not self.land_commanded
            and self.vehicle_status.nav_state
            != VehicleStatus.NAVIGATION_STATE_OFFBOARD
        ):
            if (
                self.last_offboard_request_time is None
                or now_s - self.last_offboard_request_time >= 2.0
            ):
                self.last_offboard_request_time = now_s
                self.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    1.0,
                    6.0
                )

        # =========================================================
        # 7. ARM after entering OFFBOARD
        # =========================================================

        if (
            self.offboard_requested
            and not self.arm_requested
            and self.vehicle_status.nav_state
            == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        ):

            self.get_logger().info(
                'PX4 entered OFFBOARD. Requesting ARM...'
            )

            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0,
                0.0
            )

            self.arm_requested = True
            self.last_arm_request_time = (
                self.get_clock().now().nanoseconds / 1e9
            )

        # ---------------------------------------------------------
        # Retry ARM (e.g. transient "High Gyro Bias" preflight fail)
        # until the vehicle has armed once. Never after LAND.
        # ---------------------------------------------------------

        if (
            self.vehicle_status.arming_state
            == VehicleStatus.ARMING_STATE_ARMED
        ):
            self.has_been_armed = True

        if (
            self.arm_requested
            and not self.has_been_armed
            and not self.land_commanded
            and self.vehicle_status.nav_state
            == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            and (
                self.get_clock().now().nanoseconds / 1e9
                - self.last_arm_request_time
            ) >= 2.0
        ):
            self.last_arm_request_time = (
                self.get_clock().now().nanoseconds / 1e9
            )
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0,
                0.0
            )

        # =========================================================
        # 8. Report armed state
        # =========================================================

        if (
            self.arm_requested
            and self.vehicle_status.arming_state
            == VehicleStatus.ARMING_STATE_ARMED
        ):

            # Only log target changes through the callback.
            pass


def main(args=None):

    rclpy.init(args=args)

    node = OffboardPositionNode()

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
