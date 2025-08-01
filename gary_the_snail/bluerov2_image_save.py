import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

import cv2

from sensor_msgs.msg import Image

class ImageSaver(Node):
    def __init__(self):
        super().__init__("image_saver")

        self.num = 0

        self.create_subscription(
            Image,
            "camera",
            self.image_callback,
            10
        )

        self.bridge = CvBridge()

    def image_callback(self, msg):
        self.get_logger().info(f"image saved {self.num}")
        self.num += 1

        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        cv2.imwrite(f"image_{self.num}.png", img)

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()    

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()