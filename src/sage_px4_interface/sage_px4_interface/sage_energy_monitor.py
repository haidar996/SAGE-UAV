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

from std_msgs.msg import Bool, Float32MultiArray
from px4_msgs.msg import BatteryStatus, VehicleLocalPosition


class SageEnergyMonitor(Node):
    """Steps 20a/20c: energy estimation and return-home decision.

    Reads PX4 battery state, estimates the battery drain rate and the
    energy needed to return to the home position. When the margin
    after returning (remaining - return cost - reserve) becomes
    negative, publishes /sage/energy/return_home = True. The decision
    is latched: once triggered it never clears.
    """

    def __init__(self):
        super().__init__('sage_energy_monitor')

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.battery_sub = self.create_subscription(
            BatteryStatus,
            '/fmu/out/battery_status',
            self.battery_callback,
            px4_qos
        )

        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_position_callback,
            px4_qos
        )

        # Home is the takeoff point in the local NED frame.
        self.home_xy = (0.0, 0.0)

        # Planner cruise speed used for the return-time estimate.
        # The planner moves in 2 m steps; 1.0 m/s is a conservative
        # average for the offboard position controller.
        self.cruise_speed = 1.0

        # Battery fraction that must remain when landing.
        self.declare_parameter('reserve_fraction', 0.20)
        self.reserve_fraction = float(
            self.get_parameter('reserve_fraction').value
        )

        self.return_home = False

        self.return_home_pub = self.create_publisher(
            Bool, '/sage/energy/return_home', 10
        )

        # [remaining, drain_rate (fraction/s, NaN if unknown), reserve]
        self.status_pub = self.create_publisher(
            Float32MultiArray, '/sage/energy/status', 10
        )

        # Drain-rate estimation window.
        self.rate_window_s = 30.0
        self.samples = []  # (time_s, remaining)

        self.remaining = None
        self.voltage = None
        self.position = None

        self.timer = self.create_timer(1.0, self.report)

        self.get_logger().info('SAGE energy monitor started.')
        self.get_logger().info(
            f'Cruise speed {self.cruise_speed:.1f} m/s | '
            f'reserve {self.reserve_fraction:.2f}'
        )

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def battery_callback(self, msg):
        if not msg.connected or not math.isfinite(msg.remaining):
            return

        self.remaining = float(msg.remaining)
        self.voltage = float(msg.voltage_v)

        t = self.now_s()
        self.samples.append((t, self.remaining))
        self.samples = [
            s for s in self.samples if t - s[0] <= self.rate_window_s
        ]

    def local_position_callback(self, msg):
        if msg.xy_valid and msg.z_valid:
            self.position = (float(msg.x), float(msg.y), float(msg.z))

    def drain_rate(self):
        """Battery fraction lost per second over the window (>= 0)."""
        if len(self.samples) < 2:
            return None

        t0, r0 = self.samples[0]
        t1, r1 = self.samples[-1]

        if t1 - t0 < 5.0:
            return None

        return max(0.0, (r0 - r1) / (t1 - t0))

    def report(self):
        if self.remaining is None or self.position is None:
            return

        x, y, z = self.position
        home_distance = math.hypot(x - self.home_xy[0], y - self.home_xy[1])

        # Return trip: fly home at cruise speed, then descend/land.
        return_time = home_distance / self.cruise_speed + abs(z) / 0.5

        rate = self.drain_rate()

        if rate is None:
            rate_text = 'n/a'
            return_cost_text = 'n/a'
            margin_text = 'n/a'
            margin = self.remaining - self.reserve_fraction
        else:
            return_cost = rate * return_time
            margin = self.remaining - return_cost - self.reserve_fraction
            rate_text = f'{rate * 100.0:.3f} %/s'
            return_cost_text = f'{return_cost * 100.0:.1f} %'
            margin_text = f'{margin * 100.0:.1f} %'

        if not self.return_home and margin < 0.0:
            self.return_home = True
            self.get_logger().warn(
                'ENERGY MARGIN EXHAUSTED | '
                f'remaining={self.remaining * 100.0:.1f} % | '
                f'margin={margin * 100.0:.1f} % | '
                'RETURN HOME REQUESTED.'
            )

        flag = Bool()
        flag.data = self.return_home
        self.return_home_pub.publish(flag)

        status = Float32MultiArray()
        status.data = [
            float(self.remaining),
            float('nan') if rate is None else float(rate),
            float(self.reserve_fraction),
        ]
        self.status_pub.publish(status)

        self.get_logger().info(
            'ENERGY | '
            f'remaining={self.remaining * 100.0:.1f} % | '
            f'voltage={self.voltage:.2f} V | '
            f'drain_rate={rate_text} | '
            f'home_distance={home_distance:.1f} m | '
            f'return_time={return_time:.0f} s | '
            f'return_cost={return_cost_text} | '
            f'margin_after_return={margin_text}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = SageEnergyMonitor()

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
