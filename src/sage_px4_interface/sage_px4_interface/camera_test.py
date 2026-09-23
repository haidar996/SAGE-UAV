import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


IMAGE_TOPIC = "/world/default/model/x500_mono_cam_0/link/camera_link/sensor/imager/image"


class CameraTest(Node):
    def __init__(self):
        super().__init__("camera_test")

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            IMAGE_TOPIC,
            self.image_callback,
            10,
        )

        self.frame_count = 0

        self.get_logger().info("Subscribing to camera...")
        self.get_logger().info(f"Topic: {IMAGE_TOPIC}")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            self.frame_count += 1

            if self.frame_count == 1:
                self.get_logger().info(
                    f"First frame received: {frame.shape[1]}x{frame.shape[0]}"
                )

            cv2.imshow("SAGE-UAV Camera", frame)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")


def main(args=None):
    rclpy.init(args=args)

    node = CameraTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()


if __name__ == "__main__":
    main()
