import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

from dt_apriltags import Detector
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Bool, Int16, Float32
from time import sleep

import yaml
import numpy as np

class AprilTag(Node):
    def __init__(self):
        super().__init__("april_tag_detector")

        self.at_detector = Detector(
            families='tag36h11', 
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        self.valid_back = [6, 7, 10, 11, 2, 3]
        self.valid_front = [4, 5, 8, 9, 0, 1]

        self.fx = 273.25
        self.fy = 261.76
        self.cx = 307.89
        self.cy = 153.84

        self.heading = 0

        self.sub_camera = self.create_subscription(
            Image,
            "/camera",
            self.camera_callback,
            10
        )

        self.bridge = CvBridge()

        self.pub_lights = self.create_publisher(
            Bool,
            "/flash",
            10
        )

        self.pub_heading = self.create_publisher(
            Int16,
            "/target_heading",
            10
        )

        self.pub__rel_heading = self.create_publisher(
            Int16,
            "/relative_heading",
            10
        )

        self.pub_depth = self.create_publisher(
            Float32,
            "/relative_depth",
            10
        )
        
        self.pub_speed = self.create_publisher(
            Float32,
            "/target_x",
            10
        )

        self.sub_heading = self.create_subscription(
            Int16,
            "/heading",
            self.heading_callback,
            10
        )

    def heading_callback(self, msg):
        self.heading = msg.data

    def camera_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # change img to undistorted
        
        tags = self.at_detector.detect(
            gray, 
            estimate_tag_pose=True, 
            camera_params=(self.fx, self.fy, self.cx, self.cy), 
            tag_size=0.1
        )


        robot_tags = []
        for tag in tags:
            print(f"Detected tag ID: {tag.tag_id}")

            # if tag in back
            if tag.tag_id in self.valid_back:
                robot_tags.append(tag)

                if np.linalg.norm(tag.pose_t.flatten()) <= 1: #yes
                    # flashlight on
                    msg = Bool()
                    msg.data = True
                    self.pub_lights.publish(msg)

            # if tag in front
            if tag.tag_id in self.valid_front:
                if np.linalg.norm(tag.pose_t.flatten()) <= 1:
                    msg = Bool()
                    msg.data = True
                    self.pub_lights.publish(msg)
                else:
                    msg = Float32()
                    msg.data = 1.0
                    self.pub_depth.publish(msg) #go down

                    sleep(1)
                    
                    msg = Float32()
                    msg.data = 20.0 # TODO? 
                    self.pub_speed.publish(msg) #move forward

                    sleep(1)

                    msg = Float32()
                    msg.data = 0.0
                    self.pub_speed.publish(msg) # stop moving

                    sleep(0.5)

                    msg = Float32()
                    msg.data = 180.0
                    self.pub_rel_heading.publish(msg) #turns

                    sleep(1)
                    
                    msg = Float32()
                    msg.data = -1.0
                    self.pub_depth.publish(msg) #go up 

                    return
                
        if len(tags) == 0:
            msg = Float32()
            msg.data = 1.0
            self.pub_depth.publish(msg) #go down

            sleep(1)
                    
            msg = Float32()
            msg.data = 20.0
            self.pub_speed.publish(msg) #move forward

            sleep(1)

            msg = Float32()
            msg.data = 0.0
            self.pub_speed.publish(msg) # stop moving

            sleep(0.5)
                    
            msg = Float32()
            msg.data = -1.0
            self.pub_depth.publish(msg) #go up 
                    

        def get_angle_yaw(t):
            t = t.flatten()
            return np.degrees(np.arctan2(t[0], t[2]))

        def get_vert_dist(t):
            t = t.flatten()
            return np.arctan2(t[1], t[2]) * np.sqrt(t[1] ** 2 + t[2] ** 2)
        
        average_angle = 0
        if len(robot_tags) == 2:
            average_angle = (get_angle_yaw(robot_tags[0].pose_t) + get_angle_yaw(robot_tags[1].pose_t)) / 2

        if len(robot_tags) > 0:
            average_angle = get_angle_yaw(robot_tags[0].pose_t)
            msg = Int16()
            msg.data = int(average_angle) + self.heading

            self.get_logger().info("delta heading: " + str(average_angle))

            self.pub_heading.publish(msg)

            # vert_dist = np.sin(get_vert_dist(robot_tags[0].pose_t))
            # msg = Float32()
            # msg.data = -vert_dist

            # self.get_logger().info("delta depth: " + str(-vert_dist))

            # self.pub_depth.publish(msg)

            msg = Float32()
            msg.data = 20.0
            if np.linalg.norm(robot_tags[0].pose_t.flatten()) < 0.2:
                msg.data = 0.0
            self.pub_speed.publish(msg)
        else:
            # print("no robot fetected")

            msg = Float32()
            msg.data = 0.0
            self.pub_speed.publish(msg)

            # msg = Bool()
            # msg.data = False
            # self.pub_lights.publish(msg)
            
            # yse fetected
            # yse nlo
            # nlo :P)  :)
            # ඞ sus
        
def main(args=None):
    rclpy.init(args=args)
       
    node = AprilTag()    

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()