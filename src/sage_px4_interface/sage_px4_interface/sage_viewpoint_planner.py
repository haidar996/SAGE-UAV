
#!/usr/bin/env python3

import json
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
from px4_msgs.msg import VehicleLocalPosition
from std_msgs.msg import Bool, Float32MultiArray, String
from vision_msgs.msg import Detection3DArray, Detection2DArray


class SageViewpointPlanner(Node):

    def __init__(self):
        super().__init__('sage_viewpoint_planner')

        # =========================================================
        # PX4 QoS
        # =========================================================

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # =========================================================
        # Subscribers
        # =========================================================

        self.world_model_sub = self.create_subscription(
            Detection3DArray,
            '/sage/world_model/targets',
            self.world_model_callback,
            10
        )

        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_position_callback,
            px4_qos
        )

        # ---------------------------------------------------------
        # Active Vision:
        # Subscribe directly to raw YOLO detections because the
        # current semantic world model does not preserve confidence.
        # ---------------------------------------------------------

        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/sage/perception/detections',
            self.detection_callback,
            10
        )

        # ---------------------------------------------------------
        # Step 20e: energy status [remaining, drain_rate, reserve]
        # from sage_energy_monitor.
        # ---------------------------------------------------------

        self.energy_sub = self.create_subscription(
            Float32MultiArray,
            '/sage/energy/status',
            self.energy_callback,
            10
        )

        # ---------------------------------------------------------
        # Step 19: mission spec from sage_mission_parser
        # (transient-local: a late planner still gets the mission).
        # ---------------------------------------------------------

        spec_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.mission_sub = self.create_subscription(
            String,
            '/sage/mission/spec',
            self.mission_callback,
            spec_qos
        )

        # Step 19 (completion): energy abort ends the mission too.
        self.energy_return_sub = self.create_subscription(
            Bool,
            '/sage/energy/return_home',
            self.energy_return_callback,
            10
        )

        # Mission report (latched) and return-home request.
        self.report_pub = self.create_publisher(
            String,
            '/sage/mission/report',
            spec_qos
        )

        self.return_home_pub = self.create_publisher(
            Bool,
            '/sage/mission/return_home',
            10
        )

        # =========================================================
        # Publisher
        # =========================================================

        self.viewpoint_pub = self.create_publisher(
            PoseStamped,
            '/sage/planning/viewpoint',
            10
        )

        # =========================================================
        # State
        # =========================================================

        self.latest_target = None
        self.vehicle_position = None

        # =========================================================
        # Active Vision state
        # =========================================================

        # Latest YOLO confidence for a person detection.
        self.latest_person_confidence = None

        # Latest normalized person bounding-box area.
        #
        # Camera resolution confirmed from CameraInfo:
        # width  = 1280 pixels
        # height = 960 pixels
        self.latest_person_bbox_area_fraction = None

        # ---------------------------------------------------------
        # Step 18.3a:
        #
        # Latest person bounding-box center in image pixels.
        # ---------------------------------------------------------

        self.latest_person_bbox_center_x = None
        self.latest_person_bbox_center_y = None

        # ---------------------------------------------------------
        # Step 18.3a:
        #
        # Latest person bounding-box center normalized to [0, 1].
        #
        # normalized_x:
        #   0.0 = left edge
        #   0.5 = image center
        #   1.0 = right edge
        #
        # normalized_y:
        #   0.0 = top edge
        #   0.5 = image center
        #   1.0 = bottom edge
        # ---------------------------------------------------------

        self.latest_person_image_x = None
        self.latest_person_image_y = None

        # ---------------------------------------------------------
        # Step 18.3c:
        #
        # Smallest distance between the person's bounding box and
        # any image border, as a fraction of the image size.
        # 0.0 = box touches the border (possibly clipped).
        # ---------------------------------------------------------

        self.latest_person_bbox_margin = None

        # Minimum required margin. Data-driven (see docs/progress.md,
        # 18.3e): margins cluster at <= 0.005 (box clipped by the
        # border) with a gap up to ~0.03, so 0.02 rejects clipped
        # views without penalising merely close-to-edge ones.
        self.observation_edge_margin_threshold = 0.02

        self.camera_width = 1280.0
        self.camera_height = 960.0

        # Minimum confidence required for a sufficient observation.
        self.observation_confidence_threshold = 0.6

        # ---------------------------------------------------------
        # Step 18.2b:
        #
        # Minimum fraction of the image occupied by the person's
        # bounding box.
        #
        # 0.01 = 1% of the 1280x960 image.
        # ---------------------------------------------------------

        self.observation_bbox_area_threshold = 0.01

        # Prevent old detections from being treated as current.
        self.detection_timeout = 1.0

        self.last_detection_time = None

        # True when the current viewpoint has produced a
        # sufficiently strong and fresh observation.
        self.observation_sufficient = False

        # =========================================================
        # Search parameters
        # =========================================================

        self.standoff_distance = 3.0
        self.viewpoint_altitude = -2.0

        # UAV is considered to have reached the active waypoint
        # when it is within this horizontal distance.
        self.arrival_tolerance = 0.35

        # Must remain compatible with Mission Manager.
        self.max_step_distance = 2.0

        # Four viewpoints around the target.
        self.search_angles = [
            0.0,
            math.pi / 2.0,
            math.pi,
            3.0 * math.pi / 2.0,
        ]

        self.current_viewpoint_index = 0

        # ---------------------------------------------------------
        # Step 20e: energy-aware viewpoint selection.
        #
        # True : pick the cheapest affordable unvisited viewpoint.
        # False: fixed order 1 -> 2 -> 3 -> 4 (baseline).
        # ---------------------------------------------------------

        # Step 19: when True the planner stays idle until it has
        # received a valid, supported mission.
        self.declare_parameter('require_mission', True)
        self.require_mission = bool(
            self.get_parameter('require_mission').value
        )
        self.mission_active = False
        self.mission = None

        # Mission completion state.
        self.verified = {}          # target id -> found record
        self.mission_start_time = None
        self.return_requested = False

        # Evidence accumulation: a target is verified once it has
        # accumulated this many seconds of sufficient observations
        # (not necessarily continuous; detections flicker). The
        # accumulator resets after `evidence_gap_s` without any
        # sufficient observation.
        self.verify_evidence_s = 2.0
        self.evidence_gap_s = 5.0
        self.evidence_s = 0.0
        self.last_good_time = None

        # Give up (report what was found) after this long.
        self.declare_parameter('mission_timeout_s', 300.0)
        self.mission_timeout_s = float(
            self.get_parameter('mission_timeout_s').value
        )

        self.return_timer = self.create_timer(
            1.0,
            self.return_timer_callback
        )

        self.declare_parameter('energy_aware', True)
        self.energy_aware = bool(
            self.get_parameter('energy_aware').value
        )

        self.energy_remaining = None
        self.energy_drain_rate = None
        self.energy_reserve = 0.20
        self.cruise_speed = 1.0
        self.home_xy = (0.0, 0.0)
        self.landing_time = 4.0

        self.visited_indices = set()

        # Desired search-circle viewpoint.
        self.desired_viewpoint = None

        # Current planner waypoint.
        #
        # IMPORTANT:
        # For subsequent steps this is also the reference from
        # which the next waypoint is generated.
        self.current_viewpoint = None

        # Detect a new target.
        self.current_target_id = None

        # =========================================================
        # Planner timer
        # =========================================================

        self.timer = self.create_timer(
            0.2,
            self.timer_callback
        )

        self.get_logger().info(
            'SAGE active-vision viewpoint planner started.'
        )

        self.get_logger().info(
            'Standoff distance: '
            f'{self.standoff_distance:.1f} m'
        )

        self.get_logger().info(
            'Viewpoint altitude: '
            f'{self.viewpoint_altitude:.1f} m'
        )

        self.get_logger().info(
            'Search viewpoints: '
            f'{len(self.search_angles)}'
        )

        self.get_logger().info(
            'Arrival tolerance: '
            f'{self.arrival_tolerance:.2f} m'
        )

        self.get_logger().info(
            'Maximum planner step: '
            f'{self.max_step_distance:.1f} m'
        )

        self.get_logger().info(
            'Observation confidence threshold: '
            f'{self.observation_confidence_threshold:.2f}'
        )

        self.get_logger().info(
            'Observation bbox-area threshold: '
            f'{self.observation_bbox_area_threshold:.4f} '
            '(1% of image)'
        )

        self.get_logger().info(
            'Camera resolution for bbox measurement: '
            f'{int(self.camera_width)}x'
            f'{int(self.camera_height)}'
        )

        self.get_logger().info(
            'Image-border margin threshold: '
            f'{self.observation_edge_margin_threshold:.2f}'
        )

    # =============================================================
    # Mission (Step 19)
    # =============================================================

    def mission_callback(self, msg):
        try:
            spec = json.loads(msg.data)
        except ValueError:
            self.get_logger().warn('MISSION REJECTED | invalid JSON.')
            return

        if not spec.get('valid') or not spec.get('supported'):
            self.mission_active = False
            self.get_logger().warn(
                'MISSION REJECTED | '
                f"reason={spec.get('reason', 'unknown')} | "
                'planner idle.'
            )
            return

        self.mission = spec
        self.mission_active = True
        self.verified = {}
        self.return_requested = False
        self.evidence_s = 0.0
        self.last_good_time = None
        self.mission_start_time = self.get_clock().now()

        # New mission: restart the search from scratch.
        self.current_target_id = None
        self.desired_viewpoint = None
        self.current_viewpoint = None
        self.current_viewpoint_index = 0
        self.visited_indices = set()
        self.observation_sufficient = False

        self.get_logger().info(
            'MISSION ACCEPTED | '
            f"target={spec['target_class']} | "
            f"attribute={spec['attribute']} | "
            f"quantity={spec['quantity']} | "
            f"output={spec['output']} | "
            f"text='{spec['text']}'"
        )

    # =============================================================
    # Mission completion (Step 19)
    # =============================================================

    def verify_current_target(self):
        target_id = self.current_target_id
        x, y, z = self.latest_target

        self.verified[target_id] = {
            'id': target_id,
            'x': round(x, 2),
            'y': round(y, 2),
            'confidence': round(
                float(self.latest_person_confidence), 3
            ),
            'viewpoint': self.current_viewpoint_index + 1,
        }

        self.get_logger().info(
            'TARGET VERIFIED | '
            f'id={target_id} | '
            f'position=({x:.2f}, {y:.2f}) | '
            f'confidence={self.latest_person_confidence:.3f} | '
            f'verified_total={len(self.verified)}'
        )

        # Prepare to look for the next unverified target.
        self.current_target_id = None
        self.latest_target = None
        self.desired_viewpoint = None
        self.current_viewpoint = None
        self.current_viewpoint_index = 0
        self.visited_indices = set()
        self.observation_sufficient = False
        self.evidence_s = 0.0
        self.last_good_time = None

        quantity = self.mission['quantity']

        if isinstance(quantity, int) and len(self.verified) >= quantity:
            self.complete_mission('quantity_reached')

    def complete_mission(self, reason, request_return=True):
        if not self.mission_active:
            return

        elapsed = (
            self.get_clock().now() - self.mission_start_time
        ).nanoseconds / 1e9

        found = list(self.verified.values())

        report = {
            'mission': self.mission['text'],
            'status': reason,
            'quantity_requested': self.mission['quantity'],
            'output': self.mission['output'],
            'count': len(found),
            'found': found,
            'duration_s': round(elapsed, 1),
        }

        out = String()
        out.data = json.dumps(report)
        self.report_pub.publish(out)

        locations = '; '.join(
            f"#{f['id']} ({f['x']:.1f}, {f['y']:.1f})"
            for f in found
        ) or 'none'

        self.get_logger().info(
            'MISSION COMPLETE | '
            f'status={reason} | '
            f'found={len(found)} | '
            f'locations={locations} | '
            f'duration={elapsed:.0f} s'
        )

        self.mission_active = False
        self.observation_sufficient = False

        if request_return:
            self.return_requested = True
            self.get_logger().info(
                'Requesting return home + land.'
            )

    def return_timer_callback(self):
        if self.return_requested:
            msg = Bool()
            msg.data = True
            self.return_home_pub.publish(msg)

    def energy_return_callback(self, msg):
        if msg.data and self.mission_active:
            self.complete_mission(
                'energy_abort',
                request_return=False
            )

    # =============================================================
    # Energy
    # =============================================================

    def energy_callback(self, msg):
        if len(msg.data) < 3:
            return

        self.energy_remaining = float(msg.data[0])

        rate = float(msg.data[1])
        self.energy_drain_rate = rate if math.isfinite(rate) else None

        self.energy_reserve = float(msg.data[2])

    def estimated_cost(self, viewpoint_x, viewpoint_y, distance):
        """Battery fraction to fly to the viewpoint and then return home.

        Returns None if the drain rate is not known yet.
        """
        if self.energy_drain_rate is None:
            return None

        travel_time = distance / self.cruise_speed

        return_distance = math.hypot(
            viewpoint_x - self.home_xy[0],
            viewpoint_y - self.home_xy[1]
        )

        return_time = (
            return_distance / self.cruise_speed
            + self.landing_time
        )

        return self.energy_drain_rate * (travel_time + return_time)

    def choose_energy_aware_viewpoint(self, target_x, target_y):
        """Cheapest affordable unvisited viewpoint index, or None."""
        if self.vehicle_position is None:
            return None

        ux, uy, _ = self.vehicle_position

        best = None
        skipped = 0

        for index, angle in enumerate(self.search_angles):

            if index in self.visited_indices:
                continue

            vx, vy, _, _ = self.calculate_viewpoint(
                target_x,
                target_y,
                angle
            )

            distance = math.hypot(vx - ux, vy - uy)

            cost = self.estimated_cost(vx, vy, distance)

            if (
                cost is not None
                and self.energy_remaining is not None
                and self.energy_remaining - cost
                < self.energy_reserve
            ):
                skipped += 1
                continue

            if best is None or distance < best[1]:
                best = (index, distance, cost)

        if best is None:
            return None

        cost_text = (
            f'{best[2] * 100.0:.1f} %'
            if best[2] is not None
            else 'n/a'
        )

        self.get_logger().info(
            'ENERGY-AWARE SELECTION | '
            f'chosen={best[0] + 1}/{len(self.search_angles)} | '
            f'distance={best[1]:.1f} m | '
            f'est_cost_incl_return={cost_text} | '
            f'unaffordable_skipped={skipped}'
        )

        return best[0]

    # =============================================================
    # Raw YOLO detection callback
    # =============================================================

    def detection_callback(self, msg):

        best_person_confidence = None
        best_person_bbox_area_fraction = None

        # ---------------------------------------------------------
        # Step 18.3a:
        #
        # Temporary measurements associated with the strongest
        # person detection in this message.
        # ---------------------------------------------------------

        best_person_bbox_center_x = None
        best_person_bbox_center_y = None
        best_person_bbox_margin = None

        # ---------------------------------------------------------
        # Search all detections for the strongest person result.
        # ---------------------------------------------------------

        for detection in msg.detections:

            person_confidence = None

            for result in detection.results:

                if result.hypothesis.class_id != 'person':
                    continue

                confidence = float(
                    result.hypothesis.score
                )

                if (
                    person_confidence is None
                    or confidence > person_confidence
                ):
                    person_confidence = confidence

            if person_confidence is None:
                continue

            # -----------------------------------------------------
            # Calculate normalized bounding-box area.
            # -----------------------------------------------------

            bbox_width = float(
                detection.bbox.size_x
            )

            bbox_height = float(
                detection.bbox.size_y
            )

            bbox_area = (
                bbox_width
                * bbox_height
            )

            image_area = (
                self.camera_width
                * self.camera_height
            )

            bbox_area_fraction = (
                bbox_area
                / image_area
            )

            # -----------------------------------------------------
            # Keep the strongest person detection.
            #
            # Confidence remains the primary selection criterion.
            # -----------------------------------------------------

            if (
                best_person_confidence is None
                or person_confidence > best_person_confidence
            ):
                best_person_confidence = person_confidence

                best_person_bbox_area_fraction = (
                    bbox_area_fraction
                )

                # -------------------------------------------------
                # Step 18.3a:
                #
                # Bounding-box center in image pixels.
                # -------------------------------------------------

                best_person_bbox_center_x = float(
                    detection.bbox.center.position.x
                )

                best_person_bbox_center_y = float(
                    detection.bbox.center.position.y
                )

                # Step 18.3c: margin from the nearest image border,
                # measured on the box extents (detects clipping).
                half_w = bbox_width / 2.0
                half_h = bbox_height / 2.0

                best_person_bbox_margin = min(
                    (best_person_bbox_center_x - half_w)
                    / self.camera_width,
                    (self.camera_width
                     - (best_person_bbox_center_x + half_w))
                    / self.camera_width,
                    (best_person_bbox_center_y - half_h)
                    / self.camera_height,
                    (self.camera_height
                     - (best_person_bbox_center_y + half_h))
                    / self.camera_height,
                )

        # ---------------------------------------------------------
        # Store latest observation.
        # ---------------------------------------------------------

        if best_person_confidence is not None:

            self.latest_person_confidence = (
                best_person_confidence
            )

            self.latest_person_bbox_area_fraction = (
                best_person_bbox_area_fraction
            )

            self.latest_person_bbox_center_x = (
                best_person_bbox_center_x
            )

            self.latest_person_bbox_center_y = (
                best_person_bbox_center_y
            )

            self.latest_person_bbox_margin = (
                best_person_bbox_margin
            )

            # -----------------------------------------------------
            # Step 18.3a:
            #
            # Normalize image position to [0, 1].
            # -----------------------------------------------------

            self.latest_person_image_x = (
                best_person_bbox_center_x
                / self.camera_width
            )

            self.latest_person_image_y = (
                best_person_bbox_center_y
                / self.camera_height
            )

            self.last_detection_time = (
                self.get_clock().now()
            )

            # -----------------------------------------------------
            # Step 18.3a:
            #
            # Measurement only.
            #
            # IMPORTANT:
            # These values do NOT affect HOLD/SEARCH yet.
            # -----------------------------------------------------

            self.get_logger().info(
                'PERSON OBSERVATION | '
                f'confidence='
                f'{self.latest_person_confidence:.3f} | '
                f'bbox_area_fraction='
                f'{self.latest_person_bbox_area_fraction:.4f} | '
                f'bbox_center_px='
                f'('
                f'{self.latest_person_bbox_center_x:.1f}, '
                f'{self.latest_person_bbox_center_y:.1f}'
                f') | '
                f'image_position='
                f'('
                f'{self.latest_person_image_x:.3f}, '
                f'{self.latest_person_image_y:.3f}'
                f') | '
                f'bbox_margin='
                f'{self.latest_person_bbox_margin:.3f}',
                throttle_duration_sec=1.0
            )

    # =============================================================
    # Active Vision observation quality
    # =============================================================

    def get_observation_quality(self):

        # ---------------------------------------------------------
        # No usable detection has ever been received.
        # ---------------------------------------------------------

        if (
            self.latest_person_confidence is None
            or self.latest_person_bbox_area_fraction is None
            or self.last_detection_time is None
        ):
            return False, False, False, False

        elapsed = (
            self.get_clock().now()
            - self.last_detection_time
        ).nanoseconds / 1e9

        # ---------------------------------------------------------
        # Freshness condition.
        # ---------------------------------------------------------

        fresh = elapsed <= self.detection_timeout

        # ---------------------------------------------------------
        # Confidence condition.
        # ---------------------------------------------------------

        confidence_ok = (
            self.latest_person_confidence
            >= self.observation_confidence_threshold
        )

        # ---------------------------------------------------------
        # Bounding-box size condition.
        # ---------------------------------------------------------

        bbox_size_ok = (
            self.latest_person_bbox_area_fraction
            >= self.observation_bbox_area_threshold
        )

        # Step 18.3c: target must be fully inside the image with
        # a margin from every border.
        position_ok = (
            self.latest_person_bbox_margin is not None
            and self.latest_person_bbox_margin
            >= self.observation_edge_margin_threshold
        )

        return (
            fresh,
            confidence_ok,
            bbox_size_ok,
            position_ok
        )

    def observation_is_sufficient(self):

        (
            fresh,
            confidence_ok,
            bbox_size_ok,
            position_ok
        ) = self.get_observation_quality()

        return (
            fresh
            and confidence_ok
            and bbox_size_ok
            and position_ok
        )

    # =============================================================
    # Subscribers
    # =============================================================

    def world_model_callback(self, msg):

        if len(msg.detections) == 0:
            return

        # ---------------------------------------------------------
        # Find first person target.
        # ---------------------------------------------------------

        persons = []

        for detection in msg.detections:

            for result in detection.results:

                if result.hypothesis.class_id == 'person':
                    persons.append(detection)
                    break

        if self.mission_active:

            # Step 19: only consider targets not yet verified.
            unverified = [
                d for d in persons if d.id not in self.verified
            ]

            if not unverified:
                self.latest_target = None

                if (
                    self.mission['quantity'] == 'all'
                    and len(self.verified) >= 1
                ):
                    self.complete_mission('all_tracked_verified')

                return

            persons = unverified

            # Keep the current target while it is still valid.
            current = [
                d for d in persons if d.id == self.current_target_id
            ]

            if current:
                persons = current

            elif self.vehicle_position is not None:
                ux, uy, _ = self.vehicle_position

                persons.sort(
                    key=lambda d: math.hypot(
                        d.results[0].pose.pose.position.x - ux,
                        d.results[0].pose.pose.position.y - uy,
                    )
                )

        if not persons:
            return

        selected_target = persons[0]

        # ---------------------------------------------------------
        # Extract target position.
        # ---------------------------------------------------------

        position = (
            selected_target.results[0].pose.pose.position
        )

        x = float(position.x)
        y = float(position.y)
        z = float(position.z)

        target_id = selected_target.id

        # ---------------------------------------------------------
        # Detect new target.
        # ---------------------------------------------------------

        if target_id != self.current_target_id:

            self.current_target_id = target_id

            self.current_viewpoint_index = 0
            self.desired_viewpoint = None
            self.current_viewpoint = None
            self.visited_indices = set()

            self.observation_sufficient = False

            self.get_logger().info(
                'New target acquired | '
                f'target_id={target_id}'
            )

        self.latest_target = (
            x,
            y,
            z
        )

    def local_position_callback(self, msg):

        if not msg.xy_valid or not msg.z_valid:
            return

        self.vehicle_position = (
            float(msg.x),
            float(msg.y),
            float(msg.z)
        )

    # =============================================================
    # Viewpoint generation
    # =============================================================

    def calculate_viewpoint(
        self,
        target_x,
        target_y,
        angle
    ):

        viewpoint_x = (
            target_x
            + self.standoff_distance * math.cos(angle)
        )

        viewpoint_y = (
            target_y
            + self.standoff_distance * math.sin(angle)
        )

        viewpoint_z = self.viewpoint_altitude

        # Face the target.
        yaw = math.atan2(
            target_y - viewpoint_y,
            target_x - viewpoint_x
        )

        return (
            viewpoint_x,
            viewpoint_y,
            viewpoint_z,
            yaw
        )

    # =============================================================
    # Quaternion
    # =============================================================

    def yaw_to_quaternion(self, yaw):

        return (
            0.0,
            0.0,
            math.sin(yaw / 2.0),
            math.cos(yaw / 2.0)
        )

    # =============================================================
    # Generate next reachable waypoint
    # =============================================================

    def calculate_reachable_viewpoint(self):

        if self.desired_viewpoint is None:
            return None

        # ---------------------------------------------------------
        # First waypoint:
        #
        # There is no previous planner waypoint, so start from
        # the UAV's current position.
        # ---------------------------------------------------------

        if self.current_viewpoint is None:

            if self.vehicle_position is None:
                return None

            start_x, start_y, _ = self.vehicle_position

        # ---------------------------------------------------------
        # Subsequent waypoints:
        #
        # Start from the previous planner waypoint.
        # This guarantees that the planner waypoint jump is
        # never greater than max_step_distance.
        # ---------------------------------------------------------

        else:

            start_x, start_y, _, _ = self.current_viewpoint

        target_x, target_y, target_z, target_yaw = (
            self.desired_viewpoint
        )

        dx = target_x - start_x
        dy = target_y - start_y

        distance = math.hypot(dx, dy)

        # ---------------------------------------------------------
        # Desired viewpoint is reachable in one step.
        # ---------------------------------------------------------

        if distance <= self.max_step_distance:

            return (
                target_x,
                target_y,
                self.viewpoint_altitude,
                target_yaw
            )

        # ---------------------------------------------------------
        # Move exactly max_step_distance toward desired viewpoint.
        # ---------------------------------------------------------

        scale = self.max_step_distance / distance

        next_x = start_x + dx * scale
        next_y = start_y + dy * scale

        return (
            next_x,
            next_y,
            self.viewpoint_altitude,
            target_yaw
        )

    # =============================================================
    # Arrival check
    # =============================================================

    def viewpoint_reached(self):

        if (
            self.vehicle_position is None
            or self.current_viewpoint is None
        ):
            return False

        vx, vy, _ = self.vehicle_position

        waypoint_x, waypoint_y, _, _ = (
            self.current_viewpoint
        )

        distance = math.hypot(
            waypoint_x - vx,
            waypoint_y - vy
        )

        return distance <= self.arrival_tolerance

    # =============================================================
    # Desired search viewpoint reached
    # =============================================================

    def desired_viewpoint_reached(self):

        if (
            self.vehicle_position is None
            or self.desired_viewpoint is None
        ):
            return False

        vx, vy, _ = self.vehicle_position

        desired_x, desired_y, _, _ = (
            self.desired_viewpoint
        )

        distance = math.hypot(
            desired_x - vx,
            desired_y - vy
        )

        return distance <= self.arrival_tolerance

    # =============================================================
    # Publish viewpoint
    # =============================================================

    def publish_viewpoint(self):

        if self.current_viewpoint is None:
            return

        x, y, z, yaw = self.current_viewpoint

        msg = PoseStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'px4_local_ned'

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ) = self.yaw_to_quaternion(yaw)

        self.viewpoint_pub.publish(msg)

    # =============================================================
    # Main planner
    # =============================================================

    def timer_callback(self):

        # ---------------------------------------------------------
        # Step 19: no mission, no search.
        # ---------------------------------------------------------

        if self.require_mission and not self.mission_active:
            return

        if self.mission_active:
            elapsed = (
                self.get_clock().now() - self.mission_start_time
            ).nanoseconds / 1e9

            if elapsed > self.mission_timeout_s:
                self.complete_mission('timeout')
                return

        # ---------------------------------------------------------
        # Require target and UAV position.
        # ---------------------------------------------------------

        if (
            self.latest_target is None
            or self.vehicle_position is None
        ):
            return

        target_x, target_y, target_z = self.latest_target

        # ---------------------------------------------------------
        # Step 19: accumulate evidence; verify the target.
        # ---------------------------------------------------------

        if self.mission_active:
            now = self.get_clock().now()

            if self.observation_is_sufficient():
                self.evidence_s += 0.2
                self.last_good_time = now

                if self.evidence_s >= self.verify_evidence_s:
                    self.verify_current_target()
                    return

            elif (
                self.last_good_time is not None
                and (now - self.last_good_time).nanoseconds / 1e9
                > self.evidence_gap_s
            ):
                self.evidence_s = 0.0
                self.last_good_time = None

        # ---------------------------------------------------------
        # Active Vision:
        #
        # If a previously sufficient observation becomes stale,
        # loses confidence, or has insufficient bbox size,
        # resume searching.
        # ---------------------------------------------------------

        if self.observation_sufficient:

            if not self.observation_is_sufficient():

                self.observation_sufficient = False

                (
                    fresh,
                    confidence_ok,
                    bbox_size_ok,
                    position_ok
                ) = self.get_observation_quality()

                self.get_logger().info(
                    'OBSERVATION QUALITY LOST | '
                    f'fresh={fresh} | '
                    f'confidence_ok={confidence_ok} | '
                    f'bbox_size_ok={bbox_size_ok} | '
                    f'position_ok={position_ok} | '
                    'resuming active search.'
                )

            else:

                self.publish_viewpoint()

                return

        # ---------------------------------------------------------
        # Initialize desired viewpoint.
        # ---------------------------------------------------------

        if self.desired_viewpoint is None:

            angle = self.search_angles[
                self.current_viewpoint_index
            ]

            self.desired_viewpoint = (
                self.calculate_viewpoint(
                    target_x,
                    target_y,
                    angle
                )
            )

        # ---------------------------------------------------------
        # Generate reachable planner waypoint.
        # ---------------------------------------------------------

        next_viewpoint = (
            self.calculate_reachable_viewpoint()
        )

        if next_viewpoint is None:
            return

        # ---------------------------------------------------------
        # Detect whether planner waypoint changed.
        # ---------------------------------------------------------

        waypoint_changed = (
            self.current_viewpoint != next_viewpoint
        )

        self.current_viewpoint = next_viewpoint

        self.publish_viewpoint()

        # ---------------------------------------------------------
        # Wait until current planner waypoint is reached.
        # ---------------------------------------------------------

        if not self.viewpoint_reached():
            return

        # ---------------------------------------------------------
        # If the desired search-circle viewpoint has not yet been
        # reached, continue generating reachable steps.
        # ---------------------------------------------------------

        if not self.desired_viewpoint_reached():
            return

        # ---------------------------------------------------------
        # We are at the desired viewpoint.
        # Evaluate the current observation.
        # ---------------------------------------------------------

        (
            fresh,
            confidence_ok,
            bbox_size_ok,
            position_ok
        ) = self.get_observation_quality()

        if (
            fresh
            and confidence_ok
            and bbox_size_ok
            and position_ok
        ):

            self.observation_sufficient = True

            self.get_logger().info(
                'OBSERVATION SUFFICIENT | '
                f'viewpoint='
                f'{self.current_viewpoint_index + 1}/'
                f'{len(self.search_angles)} | '
                f'person_confidence='
                f'{self.latest_person_confidence:.3f} | '
                f'bbox_area_fraction='
                f'{self.latest_person_bbox_area_fraction:.4f} | '
                f'fresh={fresh} | '
                f'confidence_ok={confidence_ok} | '
                f'bbox_size_ok={bbox_size_ok} | '
                f'position_ok={position_ok} | '
                f'bbox_margin={self.latest_person_bbox_margin:.3f} | '
                f'confidence_threshold='
                f'{self.observation_confidence_threshold:.2f} | '
                f'bbox_threshold='
                f'{self.observation_bbox_area_threshold:.4f} | '
                'HOLDING VIEWPOINT.'
            )

            self.publish_viewpoint()

            return

        # ---------------------------------------------------------
        # Observation insufficient.
        # ---------------------------------------------------------

        confidence_text = (
            f'{self.latest_person_confidence:.3f}'
            if self.latest_person_confidence is not None
            else 'None'
        )

        bbox_text = (
            f'{self.latest_person_bbox_area_fraction:.4f}'
            if self.latest_person_bbox_area_fraction is not None
            else 'None'
        )

        bbox_margin_text = (
            f'{self.latest_person_bbox_margin:.3f}'
            if self.latest_person_bbox_margin is not None
            else 'None'
        )

        self.get_logger().info(
            'OBSERVATION INSUFFICIENT | '
            f'viewpoint='
            f'{self.current_viewpoint_index + 1}/'
            f'{len(self.search_angles)} | '
            f'person_confidence={confidence_text} | '
            f'bbox_area_fraction={bbox_text} | '
            f'fresh={fresh} | '
            f'confidence_ok={confidence_ok} | '
            f'bbox_size_ok={bbox_size_ok} | '
            f'position_ok={position_ok} | '
            f'bbox_margin={bbox_margin_text} | '
            f'confidence_threshold='
            f'{self.observation_confidence_threshold:.2f} | '
            f'bbox_threshold='
            f'{self.observation_bbox_area_threshold:.4f}'
        )

        # ---------------------------------------------------------
        # Advance to next viewpoint.
        # ---------------------------------------------------------

        if self.energy_aware:

            self.visited_indices.add(self.current_viewpoint_index)

            if len(self.visited_indices) >= len(self.search_angles):
                self.visited_indices = {self.current_viewpoint_index}

                self.get_logger().info(
                    'SEARCH CYCLE COMPLETE | '
                    'starting next cycle.'
                )

            choice = self.choose_energy_aware_viewpoint(
                target_x,
                target_y
            )

            if choice is None:
                # Nothing affordable: hold position and let the
                # energy monitor trigger the return home.
                self.visited_indices.discard(
                    self.current_viewpoint_index
                )

                self.get_logger().warn(
                    'NO AFFORDABLE VIEWPOINT | holding position.',
                    throttle_duration_sec=2.0
                )

                return

            self.current_viewpoint_index = choice

            self.desired_viewpoint = self.calculate_viewpoint(
                target_x,
                target_y,
                self.search_angles[choice]
            )

            return

        self.current_viewpoint_index += 1

        if (
            self.current_viewpoint_index
            >= len(self.search_angles)
        ):

            self.current_viewpoint_index = 0

            self.get_logger().info(
                'SEARCH CYCLE COMPLETE | '
                'starting next cycle.'
            )

        else:

            self.get_logger().info(
                'NEXT SEARCH VIEWPOINT | '
                f'{self.current_viewpoint_index + 1}/'
                f'{len(self.search_angles)}'
            )

        # ---------------------------------------------------------
        # Generate next desired viewpoint.
        # ---------------------------------------------------------

        angle = self.search_angles[
            self.current_viewpoint_index
        ]

        self.desired_viewpoint = (
            self.calculate_viewpoint(
                target_x,
                target_y,
                angle
            )
        )


def main(args=None):
    rclpy.init(args=args)

    node = SageViewpointPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


