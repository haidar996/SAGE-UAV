import math
from collections import deque

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Vector3

from vision_msgs.msg import Detection3D
from vision_msgs.msg import Detection3DArray
from vision_msgs.msg import ObjectHypothesisWithPose


class SAGESemanticWorldModel(Node):

    def __init__(self):
        super().__init__("sage_semantic_world_model")

        # --------------------------------------------------
        # Stored semantic targets
        # --------------------------------------------------

        self.targets = []

        self.next_target_id = 1

        # --------------------------------------------------
        # Position smoothing
        # --------------------------------------------------

        self.alpha = 0.25

        # --------------------------------------------------
        # Target association
        # --------------------------------------------------

        self.association_gate = 2.0

        # --------------------------------------------------
        # Motion estimation
        # --------------------------------------------------

        # Short-term velocity threshold.
        #
        # This is NOT sufficient by itself to classify a
        # target as moving anymore.
        self.moving_speed_threshold = 0.30

        self.stationary_speed_threshold = 0.15

        # Number of consecutive strong motion windows
        # required before confirming MOVING.
        self.motion_confirmation_count = 3

        self.minimum_dt = 0.01

        # --------------------------------------------------
        # Velocity estimation window
        # --------------------------------------------------

        # Short temporal window used for velocity estimation.
        self.motion_window_size = 5

        # --------------------------------------------------
        # Persistent-motion history
        # --------------------------------------------------

        # Longer history used for robust motion classification.
        #
        # This is intentionally larger than the velocity
        # estimation window.
        self.classification_window_size = 10

        # Minimum net displacement required before a target
        # can become MOVING.
        self.minimum_net_displacement = 0.50

        # Minimum average displacement speed over the
        # classification window.
        self.minimum_average_speed = 0.20

        # A straightness ratio near 1 means the target moved
        # consistently in one direction.
        #
        # A low value means the trajectory contains a lot
        # of back-and-forth motion.
        self.minimum_straightness = 0.70

        # Fraction of motion steps that must point generally
        # in the same direction as the overall displacement.
        self.minimum_direction_consistency = 0.70

        # --------------------------------------------------
        # Subscribe to 3D localization
        # --------------------------------------------------

        self.subscription = self.create_subscription(
            PointStamped,
            "/sage/perception/target_position",
            self.target_callback,
            10,
        )

        # --------------------------------------------------
        # Publish semantic world model
        # --------------------------------------------------

        self.publisher = self.create_publisher(
            Detection3DArray,
            "/sage/world_model/targets",
            10,
        )

        # --------------------------------------------------
        # Logging
        # --------------------------------------------------

        self.get_logger().info(
            "SAGE Semantic World Model started."
        )

        self.get_logger().info(
            f"Position smoothing enabled | "
            f"alpha={self.alpha}"
        )

        self.get_logger().info(
            f"Target association enabled | "
            f"gate={self.association_gate:.2f} m"
        )

        self.get_logger().info(
            f"Velocity estimation enabled | "
            f"window={self.motion_window_size} observations"
        )

        self.get_logger().info(
            f"Persistent motion classification enabled | "
            f"classification_window="
            f"{self.classification_window_size} observations"
        )

        self.get_logger().info(
            f"Motion requirements | "
            f"net_displacement>="
            f"{self.minimum_net_displacement:.2f} m | "
            f"avg_speed>="
            f"{self.minimum_average_speed:.2f} m/s | "
            f"straightness>="
            f"{self.minimum_straightness:.2f} | "
            f"direction_consistency>="
            f"{self.minimum_direction_consistency:.2f}"
        )

        self.get_logger().info(
            f"Motion confirmation required | "
            f"{self.motion_confirmation_count} "
            f"consecutive strong windows"
        )

    # ======================================================
    # Motion classification
    # ======================================================

    def evaluate_motion(self, target):

        history = target["classification_history"]

        if len(history) < self.classification_window_size:
            return

        # --------------------------------------------------
        # Extract oldest/newest positions
        # --------------------------------------------------

        old_x, old_y, old_z, old_time = history[0]

        new_x, new_y, new_z, new_time = history[-1]

        total_dt = new_time - old_time

        if total_dt < self.minimum_dt:
            return

        # --------------------------------------------------
        # Net displacement
        # --------------------------------------------------

        displacement_x = new_x - old_x
        displacement_y = new_y - old_y
        displacement_z = new_z - old_z

        net_displacement = math.sqrt(
            displacement_x ** 2
            + displacement_y ** 2
            + displacement_z ** 2
        )

        # --------------------------------------------------
        # Average speed over the whole history
        # --------------------------------------------------

        average_speed = (
            net_displacement / total_dt
        )

        # --------------------------------------------------
        # Cumulative path length
        # --------------------------------------------------

        path_length = 0.0

        for i in range(1, len(history)):

            x1, y1, z1, _ = history[i - 1]
            x2, y2, z2, _ = history[i]

            step_distance = math.sqrt(
                (x2 - x1) ** 2
                + (y2 - y1) ** 2
                + (z2 - z1) ** 2
            )

            path_length += step_distance

        # --------------------------------------------------
        # Straightness ratio
        # --------------------------------------------------

        if path_length > self.minimum_dt:

            straightness = (
                net_displacement / path_length
            )

        else:

            straightness = 0.0

        # --------------------------------------------------
        # Direction consistency
        # --------------------------------------------------
        #
        # Compare each displacement step with the overall
        # displacement vector.
        #
        # A dot product close to +1 means the step is moving
        # in the same direction as the overall displacement.
        #
        # A negative value means the step is moving backwards.
        # --------------------------------------------------

        direction_consistent_steps = 0
        valid_steps = 0

        if net_displacement > self.minimum_net_displacement:

            overall_x = displacement_x / net_displacement
            overall_y = displacement_y / net_displacement
            overall_z = displacement_z / net_displacement

            for i in range(1, len(history)):

                x1, y1, z1, _ = history[i - 1]
                x2, y2, z2, _ = history[i]

                step_x = x2 - x1
                step_y = y2 - y1
                step_z = z2 - z1

                step_length = math.sqrt(
                    step_x ** 2
                    + step_y ** 2
                    + step_z ** 2
                )

                if step_length < 0.001:
                    continue

                step_x /= step_length
                step_y /= step_length
                step_z /= step_length

                dot = (
                    step_x * overall_x
                    + step_y * overall_y
                    + step_z * overall_z
                )

                valid_steps += 1

                # A step is considered directionally
                # consistent if it points at least generally
                # toward the overall displacement direction.
                if dot >= 0.50:
                    direction_consistent_steps += 1

        if valid_steps > 0:

            direction_consistency = (
                direction_consistent_steps
                / valid_steps
            )

        else:

            direction_consistency = 0.0

        # --------------------------------------------------
        # Store diagnostic motion metrics
        # --------------------------------------------------

        target["net_displacement"] = net_displacement
        target["average_speed"] = average_speed
        target["path_length"] = path_length
        target["straightness"] = straightness
        target["direction_consistency"] = direction_consistency

        # --------------------------------------------------
        # Strong persistent-motion condition
        # --------------------------------------------------

        strong_motion = (
            net_displacement
            >= self.minimum_net_displacement
            and average_speed
            >= self.minimum_average_speed
            and straightness
            >= self.minimum_straightness
            and direction_consistency
            >= self.minimum_direction_consistency
        )

        # --------------------------------------------------
        # Strong stationary condition
        # --------------------------------------------------

        strong_stationary = (
            net_displacement
            < self.minimum_net_displacement * 0.50
            and average_speed
            < self.stationary_speed_threshold
        )

        current_state = target["motion_state"]

        # --------------------------------------------------
        # Persistent MOVING evidence
        # --------------------------------------------------

        if strong_motion:

            target["moving_evidence"] += 1
            target["stationary_evidence"] = 0

            if (
                target["moving_evidence"]
                >= self.motion_confirmation_count
            ):

                if current_state != "MOVING":

                    target["motion_state"] = "MOVING"

                    self.get_logger().info(
                        f"Target {target['id']} "
                        f"confirmed MOVING | "
                        f"net={net_displacement:.2f} m | "
                        f"avg_speed={average_speed:.2f} m/s | "
                        f"straightness={straightness:.2f} | "
                        f"direction={direction_consistency:.2f}"
                    )

                target["moving_evidence"] = 0

        # --------------------------------------------------
        # Persistent STATIONARY evidence
        # --------------------------------------------------

        elif strong_stationary:

            target["stationary_evidence"] += 1
            target["moving_evidence"] = 0

            if (
                target["stationary_evidence"]
                >= self.motion_confirmation_count
            ):

                if current_state != "STATIONARY":

                    target["motion_state"] = "STATIONARY"

                    self.get_logger().info(
                        f"Target {target['id']} "
                        f"confirmed STATIONARY | "
                        f"net={net_displacement:.2f} m | "
                        f"avg_speed={average_speed:.2f} m/s"
                    )

                target["stationary_evidence"] = 0

        # --------------------------------------------------
        # Ambiguous motion
        # --------------------------------------------------

        else:

            # Do not immediately change the state.
            #
            # This is important because noisy localization
            # should not cause rapid MOVING/STATIONARY
            # oscillations.

            target["moving_evidence"] = 0
            target["stationary_evidence"] = 0

    # ======================================================
    # Target callback
    # ======================================================

    def target_callback(self, msg):

        # --------------------------------------------------
        # Raw 3D measurement
        # --------------------------------------------------

        x = msg.point.x
        y = msg.point.y
        z = msg.point.z

        measurement_time = (
            msg.header.stamp.sec
            + msg.header.stamp.nanosec * 1e-9
        )

        if measurement_time <= 0.0:

            measurement_time = (
                self.get_clock().now().nanoseconds
                * 1e-9
            )

        # --------------------------------------------------
        # Find nearest existing target
        # --------------------------------------------------

        best_target = None
        best_distance = float("inf")

        for target in self.targets:

            distance = math.sqrt(
                (x - target["x"]) ** 2
                + (y - target["y"]) ** 2
                + (z - target["z"]) ** 2
            )

            if distance < best_distance:

                best_distance = distance
                best_target = target

        # --------------------------------------------------
        # Associate measurement with existing target
        # --------------------------------------------------

        if (
            best_target is not None
            and best_distance <= self.association_gate
        ):

            target = best_target

            # --------------------------------------------------
            # Save raw measurement
            # --------------------------------------------------

            target["last_raw_x"] = x
            target["last_raw_y"] = y
            target["last_raw_z"] = z

            # --------------------------------------------------
            # Exponential Moving Average
            # --------------------------------------------------

            target["x"] = (
                self.alpha * x
                + (1.0 - self.alpha) * target["x"]
            )

            target["y"] = (
                self.alpha * y
                + (1.0 - self.alpha) * target["y"]
            )

            target["z"] = (
                self.alpha * z
                + (1.0 - self.alpha) * target["z"]
            )

            # --------------------------------------------------
            # Short-term velocity history
            # --------------------------------------------------

            target["position_history"].append(
                (
                    target["x"],
                    target["y"],
                    target["z"],
                    measurement_time,
                )
            )

            # --------------------------------------------------
            # Longer classification history
            # --------------------------------------------------

            target["classification_history"].append(
                (
                    target["x"],
                    target["y"],
                    target["z"],
                    measurement_time,
                )
            )

            # --------------------------------------------------
            # Temporal velocity estimation
            # --------------------------------------------------

            history = target["position_history"]

            if len(history) >= self.motion_window_size:

                old_x, old_y, old_z, old_time = history[0]

                dt = measurement_time - old_time

                if dt >= self.minimum_dt:

                    vx = (
                        target["x"] - old_x
                    ) / dt

                    vy = (
                        target["y"] - old_y
                    ) / dt

                    vz = (
                        target["z"] - old_z
                    ) / dt

                    speed = math.sqrt(
                        vx ** 2
                        + vy ** 2
                        + vz ** 2
                    )

                    target["vx"] = vx
                    target["vy"] = vy
                    target["vz"] = vz
                    target["speed"] = speed

            # --------------------------------------------------
            # Robust motion classification
            # --------------------------------------------------

            self.evaluate_motion(target)

            # --------------------------------------------------
            # Update timestamp
            # --------------------------------------------------

            target["last_update_time"] = measurement_time

            target["detections"] += 1

            # --------------------------------------------------
            # Publish updated world model
            # --------------------------------------------------

            self.publish_world_model()

            self.get_logger().info(
                f"Target {target['id']} updated | "
                f"association_distance="
                f"{best_distance:.2f} m | "
                f"raw=("
                f"{x:.2f}, "
                f"{y:.2f}, "
                f"{z:.2f}) | "
                f"smoothed=("
                f"{target['x']:.2f}, "
                f"{target['y']:.2f}, "
                f"{target['z']:.2f}) | "
                f"velocity=("
                f"{target['vx']:.2f}, "
                f"{target['vy']:.2f}, "
                f"{target['vz']:.2f}) m/s | "
                f"speed={target['speed']:.2f} m/s | "
                f"net={target['net_displacement']:.2f} m | "
                f"straightness="
                f"{target['straightness']:.2f} | "
                f"direction="
                f"{target['direction_consistency']:.2f} | "
                f"state="
                f"{target['motion_state']} | "
                f"detections="
                f"{target['detections']}"
            )

            return

        # --------------------------------------------------
        # No existing target matched
        # --------------------------------------------------

        position_history = deque(
            maxlen=self.motion_window_size
        )

        classification_history = deque(
            maxlen=self.classification_window_size
        )

        position_history.append(
            (
                x,
                y,
                z,
                measurement_time,
            )
        )

        classification_history.append(
            (
                x,
                y,
                z,
                measurement_time,
            )
        )

        target = {
            "id": self.next_target_id,

            "class": "person",

            # --------------------------------------------------
            # Smoothed position
            # --------------------------------------------------

            "x": x,
            "y": y,
            "z": z,

            # --------------------------------------------------
            # Raw measurement
            # --------------------------------------------------

            "last_raw_x": x,
            "last_raw_y": y,
            "last_raw_z": z,

            # --------------------------------------------------
            # Velocity
            # --------------------------------------------------

            "vx": 0.0,
            "vy": 0.0,
            "vz": 0.0,

            # --------------------------------------------------
            # Speed
            # --------------------------------------------------

            "speed": 0.0,

            # --------------------------------------------------
            # Robust motion metrics
            # --------------------------------------------------

            "net_displacement": 0.0,
            "average_speed": 0.0,
            "path_length": 0.0,
            "straightness": 0.0,
            "direction_consistency": 0.0,

            # --------------------------------------------------
            # Motion state
            # --------------------------------------------------

            "motion_state": "STATIONARY",

            # --------------------------------------------------
            # Evidence counters
            # --------------------------------------------------

            "moving_evidence": 0,
            "stationary_evidence": 0,

            # --------------------------------------------------
            # Number of observations
            # --------------------------------------------------

            "detections": 1,

            # --------------------------------------------------
            # Timestamp
            # --------------------------------------------------

            "last_update_time": measurement_time,

            # --------------------------------------------------
            # Short velocity history
            # --------------------------------------------------

            "position_history": position_history,

            # --------------------------------------------------
            # Long classification history
            # --------------------------------------------------

            "classification_history": classification_history,
        }

        self.targets.append(target)

        self.next_target_id += 1

        self.publish_world_model()

        self.get_logger().info(
            f"NEW TARGET | "
            f"id={target['id']} | "
            f"class={target['class']} | "
            f"position=("
            f"{x:.2f}, "
            f"{y:.2f}, "
            f"{z:.2f}) | "
            f"state=STATIONARY"
        )

    # ======================================================
    # Publish semantic world model
    # ======================================================

    def publish_world_model(self):

        msg = Detection3DArray()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.header.frame_id = "px4_local_ned"

        for target in self.targets:

            detection = Detection3D()

            detection.id = str(target["id"])

            # --------------------------------------------------
            # Target pose
            # --------------------------------------------------

            pose = PoseWithCovariance()

            pose.pose = Pose()

            pose.pose.position.x = target["x"]
            pose.pose.position.y = target["y"]
            pose.pose.position.z = target["z"]

            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0

            # --------------------------------------------------
            # Semantic hypothesis
            # --------------------------------------------------

            result = ObjectHypothesisWithPose()

            result.hypothesis.class_id = target["class"]

            # Raw YOLO confidence intentionally excluded.
            result.hypothesis.score = 0.0

            result.pose = pose

            detection.results.append(result)

            # --------------------------------------------------
            # Bounding box
            # --------------------------------------------------

            detection.bbox.center = pose.pose
            detection.bbox.size = Vector3()

            msg.detections.append(detection)

        self.publisher.publish(msg)

    # ======================================================
    # Print world model
    # ======================================================

    def print_world_model(self):

        if not self.targets:

            self.get_logger().info(
                "World model: no targets."
            )

            return

        self.get_logger().info(
            f"World model contains "
            f"{len(self.targets)} target(s):"
        )

        for target in self.targets:

            self.get_logger().info(
                f"  Target {target['id']} | "
                f"class={target['class']} | "
                f"position=("
                f"{target['x']:.2f}, "
                f"{target['y']:.2f}, "
                f"{target['z']:.2f}) | "
                f"velocity=("
                f"{target['vx']:.2f}, "
                f"{target['vy']:.2f}, "
                f"{target['vz']:.2f}) m/s | "
                f"speed={target['speed']:.2f} m/s | "
                f"net={target['net_displacement']:.2f} m | "
                f"straightness="
                f"{target['straightness']:.2f} | "
                f"direction="
                f"{target['direction_consistency']:.2f} | "
                f"state="
                f"{target['motion_state']} | "
                f"detections="
                f"{target['detections']}"
            )


def main(args=None):

    rclpy.init(args=args)

    node = SAGESemanticWorldModel()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
