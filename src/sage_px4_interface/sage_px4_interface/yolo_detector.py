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

        self.timer = self.create_timer(
            0.05,
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
                self.latest_header = msg.header

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
