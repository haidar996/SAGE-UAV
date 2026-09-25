import os
import threading
import time

import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D
from vision_msgs.msg import ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO


WORLD = os.environ.get("SAGE_WORLD", "sage_test")
IMAGE_TOPIC = (
    f"/world/{WORLD}/model/x500_mono_cam_0/link/camera_link/"
    "sensor/imager/image"
)
DETECTION_TOPIC = "/sage/perception/detections"


class YOLODetector(Node):

    def __init__(self):
        super().__init__("sage_yolo_detector")

        self.bridge = CvBridge()
        self.model = YOLO("yolo26n.pt")

        self.detection_publisher = self.create_publisher(
            Detection2DArray,
            DETECTION_TOPIC,
            10,
        )

        # Optional annotated frames for filming (SAGE_YOLO_ANNOTATE=1): the boxes are drawn
        # on the exact frame the detector analysed, so video and boxes cannot drift apart.
        self.annotate = os.environ.get("SAGE_YOLO_ANNOTATE") == "1"
        self.annotated_pub = (
            self.create_publisher(Image, "/sage/perception/annotated", 2)
            if self.annotate else None
        )

        self.latest_frame = None
        self.latest_header = None
        self.frame_lock = threading.Lock()

        self.frame_count = 0
        self.inference_count = 0

        self.subscription = self.create_subscription(
            Image,
            IMAGE_TOPIC,
            self.image_callback,
            10,
        )

        # Inference rate cap (Hz). The simulator, PX4 and YOLO share
        # one CPU; unthrottled YOLO starves the simulator (real-time
        # factor < 0.5 -> PX4 estimator failures).
        detector_hz = float(os.environ.get("SAGE_YOLO_HZ", "5"))

        self.timer = self.create_timer(
            1.0 / detector_hz,
            self.process_latest_frame,
        )

        self.get_logger().info("SAGE YOLO detector started.")
        self.get_logger().info(f"Camera topic: {IMAGE_TOPIC}")
        self.get_logger().info(f"Detection topic: {DETECTION_TOPIC}")
        self.get_logger().info("Model: YOLO26n")
        self.get_logger().info("Inference device: CPU")

    def image_callback(self, msg):

        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )

            with self.frame_lock:
                self.latest_frame = frame
                # Stamp with the arrival time (node clock) so consumers can
                # match the detection to the UAV pose at capture time.
                header = msg.header
                header.stamp = self.get_clock().now().to_msg()
                self.latest_header = header

            self.frame_count += 1

        except Exception as e:
            self.get_logger().error(
                f"Image conversion error: {e}"
            )

    def process_latest_frame(self):

        with self.frame_lock:

            if self.latest_frame is None:
                return

            frame = self.latest_frame.copy()
            header = self.latest_header

        start_time = time.perf_counter()

        try:

            results = self.model(
                frame,
                device="cpu",
                imgsz=320,
                verbose=False,
            )

            inference_time = time.perf_counter() - start_time
            result = results[0]

            persons = []

            for box in result.boxes:

                class_id = int(box.cls[0])
                confidence = float(box.conf[0])

                if class_id == 0 and confidence >= 0.30:

                    x1, y1, x2, y2 = box.xyxy[0].tolist()

                    persons.append(
                        (
                            confidence,
                            x1,
                            y1,
                            x2,
                            y2,
                        )
                    )

            self.inference_count += 1

            if self.annotate:
                vis = cv2.resize(frame, (1024, 768))
                k = 1024.0 / frame.shape[1]
                for confidence, x1, y1, x2, y2 in persons:
                    p1, p2 = (int(x1 * k), int(y1 * k)), (int(x2 * k), int(y2 * k))
                    cv2.rectangle(vis, p1, p2, (96, 201, 66), 2)
                    cv2.rectangle(vis, (p1[0], p1[1] - 20), (p1[0] + 118, p1[1]), (96, 201, 66), -1)
                    cv2.putText(vis, f"person {confidence:.2f}", (p1[0] + 4, p1[1] - 6),
                                cv2.FONT_HERSHEY_DUPLEX, 0.5, (20, 20, 20), 1, cv2.LINE_AA)
                # built by hand: cv_bridge.cv2_to_imgmsg fails inside the vision venv
                out = Image()
                if header is not None:
                    out.header = header
                out.height, out.width = vis.shape[0], vis.shape[1]
                out.encoding = "bgr8"
                out.step = vis.shape[1] * 3
                out.data = vis.tobytes()
                self.annotated_pub.publish(out)

            detection_msg = Detection2DArray()

            if header is not None:
                detection_msg.header = header

            for confidence, x1, y1, x2, y2 in persons:

                detection = Detection2D()

                if header is not None:
                    detection.header = header

                hypothesis = ObjectHypothesisWithPose()

                hypothesis.hypothesis.class_id = "person"
                hypothesis.hypothesis.score = confidence

                detection.results.append(
                    hypothesis
                )

                detection.bbox.center.position.x = (
                    (x1 + x2) / 2.0
                )

                detection.bbox.center.position.y = (
                    (y1 + y2) / 2.0
                )

                detection.bbox.size_x = x2 - x1
                detection.bbox.size_y = y2 - y1

                detection_msg.detections.append(
                    detection
                )

            self.detection_publisher.publish(
                detection_msg
            )

            annotated = frame.copy()

            for confidence, x1, y1, x2, y2 in persons:

                cv2.rectangle(
                    annotated,
                    (int(x1), int(y1)),
                    (int(x2), int(y2)),
                    (0, 255, 0),
                    2,
                )

                label = f"PERSON {confidence:.2f}"

                cv2.putText(
                    annotated,
                    label,
                    (
                        int(x1),
                        max(25, int(y1) - 10),
                    ),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )

            fps = (
                1.0 / inference_time
                if inference_time > 0
                else 0.0
            )

            status = (
                f"YOLO26n | "
                f"Persons: {len(persons)} | "
                f"Inference: {inference_time:.2f}s | "
                f"FPS: {fps:.1f}"
            )

            cv2.putText(
                annotated,
                status,
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )

            cv2.imshow(
                "SAGE-UAV YOLO Perception",
                annotated,
            )

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):

                self.get_logger().info(
                    "Q pressed. Shutting down YOLO detector."
                )

                rclpy.shutdown()
                return

            if persons:

                self.get_logger().info(
                    f"Detected {len(persons)} person(s) | "
                    f"inference={inference_time:.2f}s"
                )

                for i, (
                    confidence,
                    x1,
                    y1,
                    x2,
                    y2,
                ) in enumerate(persons):

                    self.get_logger().info(
                        f"  Person {i + 1}: "
                        f"confidence={confidence:.2f}, "
                        f"bbox=["
                        f"{x1:.0f}, "
                        f"{y1:.0f}, "
                        f"{x2:.0f}, "
                        f"{y2:.0f}"
                        f"]"
                    )

        except Exception as e:

            self.get_logger().error(
                f"YOLO inference error: {e}"
            )


def main(args=None):

    rclpy.init(args=args)

    node = YOLODetector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        cv2.destroyAllWindows()

        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
