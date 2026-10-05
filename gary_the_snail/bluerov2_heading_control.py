import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Float32
from mavros_msgs.msg import ManualControl
from time import time
import numpy as np

class HeadingControl(Node):
    def __init__(self):
        super().__init__("heading_control")
        
        self.target_heading = 0

        self.Kp = 0.70
        self.Ki = 0.20
        self.Kd = 0.20
        
        self.integral = 0.0
        self.last_error = 0.0
        self.first_run = True

        self.sub = self.create_subscription(
            Int16,
            "heading",
            self.heading_callback,
            10
        )

        self.pub = self.create_publisher(
            ManualControl,
            "manual_control",
            10
        )

        self.sub_target = self.create_subscription(
            Int16,
            "target_heading",
            self.get_target_heading,
            10
        )

        self.sub_relative = self.create_subscription(
            Int16,
            "relative_heading",
            self.get_relative_target,
            10
        )

        self.pub = self.create_publisher(
            Float32,
            "heading_control",
            10
        )
        
        self.last_time = time()
        self.get_logger().info("initialized heading control subscriber node")

    def get_target_heading(self, msg):
        self.target_heading = msg.data
    
    def get_relative_target(self, msg):
        self.target_heading = self.target_heading + msg.data

    def normalize_angle(self, angle):
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    def heading_callback(self, msg):
        current_time = time()
        dt = current_time - self.last_time
            
        heading = msg.data
        self.get_logger().info(f"heading: {heading}")

        heading = self.normalize_angle(heading)
        target_heading = self.normalize_angle(self.target_heading)

        error = target_heading - heading
        error = self.normalize_angle(error)
        
        self.integral += error * dt
        self.integral = max(-20.0, min(20.0, self.integral))

        if self.first_run:
            derivative = 0.0
            self.first_run = False
        else:
            derivative = (error - self.last_error) / dt

        output = error * self.Kp + self.integral * self.Ki + derivative * self.Kd

        self.last_error = error
        self.last_time = current_time

        self.publish_rotation(output)
    
    def publish_rotation(self, r):
        msg = Float32()

        r = max(-100.0, min(100.0, float(r)))
        msg.data = r

        self.pub.publish(msg)

    
def main(args=None):
    rclpy.init(args=args)
    node = HeadingControl()    

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()