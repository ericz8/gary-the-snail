import rclpy
from rclpy.node import Node

from gary_the_snail import lane_detection
from gary_the_snail import lane_following

from std_msgs.msg import Int16, Float32
from sensor_msgs.msg import Image

from cv_bridge import CvBridge

import numpy as np
from time import time

class LaneFollow(Node):
    def __init__(self):
        super().__init__("lane_following")

        self.pub_lateral = self.create_publisher(
            Float32,
            "/target_y",
            10
        )

        self.pub_relative_heading = self.create_publisher(
            Int16,
            "/relative_heading",
            10
        )

        self.sub_camera = self.create_subscription(
            Image,
            "/camera", # bluerov2/camera ?
            self.camera_callback,
            10
        )

        self.Kp = 60.0
        self.Ki = 8.0
        self.Kd = 50.0

        self.integral = 0.0
        self.last_error = 0.0

        self.bridge = CvBridge()
        
        self.last_time = 0
        self.first_run = True
        
    def camera_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        img = lane_detection.crop_half(img)
        self.IMAGE_WIDTH = img.shape[1]
        self.IMAGE_HEIGHT = img.shape[0]

        self.lane_follow_publisher(img)

    def lane_follow_publisher(self, img):
        lines = lane_detection.detect_lines(img, threshold1=20, threshold2=60, aperture_size=3, minLineLength=25, maxLineGap=25)
        lanes = lane_detection.detect_lanes(lines)

        intercept, slope = lane_following.get_lane_center(img, lanes)
        recommended = lane_following.recommend_direction(img, intercept, slope)

        msg = Int16()
        if recommended == "counter":
            msg.data = -20
        elif recommended == "clock":
            msg.data = 20

        self.pub_relative_heading.publish(msg)

        self.get_logger().info("move: " + recommended)

        error = self.IMAGE_WIDTH // 2 - intercept

        if error < 25:
            msg = Int16()

            angle_line = np.arctan(slope)
            relative_angle = np.pi / 2 - angle_line

            msg.data = int(np.degrees(relative_angle))
            self.get_logger().info("centered lane")
            self.pub_relative_heading.publish(msg)
        else:
            self.get_logger().info("errpr: " + str(error))
            
            dt = time() - self.last_time
            
            self.integral += max(-20.0, min(20.0, dt*error))
            
            if self.first_run:
                derivative = 0.0
                self.first_run = False
            else:
                derivative = (error - self.last_error) / dt

            output = error * self.Kp + self.integral * self.Ki + derivative * self.Kd

            self.last_error = error
            self.last_time = time()

            self.publish_lateral(output)

            self.get_logger().info("output: " + str(output))
    
    def publish_lateral(self, out):
        msg = Float32()
        msg.data = np.clip(out, -100.0, 100.0)
        self.pub_lateral.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollow()    

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()