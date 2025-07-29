import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

from dt_apriltags import Detector
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Bool, Int16, Float32

import yaml
import numpy as np

class april_tag_detector(Node):
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

        self.valid_back = [18]

        self.fx = 273.25
        self.fy = 261.76
        self.cx = 307.89
        self.cy = 153.84

        self.sub_camera = self.create_subscription(
            Image,
            "/camera",
            self.camera_callback,
            10
        )

        self.bridge = CvBridge()

        self.pub_lights = self.create_publisher(
            Bool,
            "flash",
            10
        )

        self.pub_heading = self.create_publisher(
            Int16,
            "relative_heading",
            10
        )
        
        self.pub_speed = self.create_publisher(
            Float32,
            "target_x",
            10
        )

    def camera_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # change img to undistorted

        tags = self.at_detector.detect(
            gray, 
            estimate_tag_pose=True, 
            camera_params=(self.fx, self.fy, self.cx, self.cy), 
            tag_size=0.1
        )

        # if two tags are both robot tags then go toward the middle
        # do we care if theyre on the back 

        robot_tags = []
        for tag in tags:
            print(f"Detected tag ID: {tag.tag_id}")
            print("Translation (t):", tag.pose_t.flatten())
            print("Rotation matrix (R):\n", tag.pose_R) 
            if tag.tag_id in self.valid_back:
                robot_tags.append(tag)

                if np.linalg.norm(tag.pose_t.flatten()) <= 1: #yes
                    # flashlight on
                    msg = Bool()
                    msg.data = True
                    self.pub_lights.publish(msg)
        
        # average_yaw = 0
        # if len(robot_tags) == 2:
        #     average_yaw = (robot_tags[0].pose_R[2] + robot_tags[1].pose_R[2]) / 2

        # if len(robot_tags) > 0:
        #     average_yaw = robot_tags[0].pose_R[2]
        #     msg = Int16() 
        #     msg.data = average_yaw

        #     self.pub_heading.publish(msg)

        #     vert_dist = np.sin(np.radians(robot_tags[0].pose_R[1]))
        #     msg = Float32()
        #     msg.data = -vert_dist

        #     self.pub_depth.publish(msg)

        #     msg = Int16()
        #     msg.data = 30
        #     if np.linalg.norm(robot_tags[0].pose_t.flatten()) < 0.2:
        #         msg.data = 0
        #     self.pub_speed.publish(msg)
        # else:
        #     print("no robot fetected")

        #     msg = Float32()
        #     msg.data = 0.0
        #     self.pub_speed.publish(msg)
        #     # yse fetected
        #     # yse nlo
        #     # nlo :P)  :)
        #     # ඞ
        
        def rotation_matrix_to_euler(R):
            """
            Extract yaw, pitch, roll from 3x3 rotation matrix using ZYX convention
            Returns angles in degrees
            """
            # Check for gimbal lock
            if abs(R[2, 0]) < 0.99999:  # Normal case
                pitch = np.arcsin(-R[2, 0])
                yaw = np.arctan2(R[1, 0], R[0, 0])
                roll = np.arctan2(R[2, 1], R[2, 2])
            else:  # Gimbal lock
                roll = 0
                if R[2, 0] < 0:
                    pitch = np.pi / 2
                    yaw = np.arctan2(R[0, 1], R[1, 1])
                else:
                    pitch = -np.pi / 2
                    yaw = np.arctan2(-R[0, 1], R[1, 1])
            
            # Convert to degrees
            print(f"Yaw: {np.degrees(yaw)}, Pitch: {np.degrees(pitch)}, Roll: {np.degrees(roll)}")
            return np.degrees(yaw), np.degrees(pitch), np.degrees(roll)

        # Your corrected code:
        average_yaw = 0
        if len(robot_tags) == 2:
            yaw1, _, _ = rotation_matrix_to_euler(robot_tags[0].pose_R)
            yaw2, _, _ = rotation_matrix_to_euler(robot_tags[1].pose_R)
            average_yaw = (yaw1 + yaw2) / 2

        if len(robot_tags) > 0:
            yaw, pitch, roll = rotation_matrix_to_euler(robot_tags[0].pose_R)
            average_yaw = yaw
            
            msg = Int16() 
            msg.data = int(average_yaw)  # Convert to int for Int16
            self.pub_heading.publish(msg)
 

            msg = Float32()
            msg.data = 30.0
            if np.linalg.norm(robot_tags[0].pose_t.flatten()) < 0.2:
                msg.data = 0.0
            self.pub_speed.publish(msg)

        else:
            print("no robot detected")
            msg = Float32()
            msg.data = 0.0
            self.pub_speed.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    node = april_tag_detector()    

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()