import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

from dt_apriltags import Detector
import matplotlib as plt
import cv2
from cv_bridge import CvBridge

import yaml
import numpy as np

class april_tag_detector(Node):
    def __init__(self):
        super().__init__("april_tag_detector")

        with open("/full_path_to/blue_rov_camera.yaml", 'r') as f: # replace with your saved path from previous step
            self.calib_data = yaml.safe_load(f)

        self.K = np.array(calib_data["camera_matrix"]["data"]).reshape((3, 3))  # fx, 0, cx, 0, fy, cy, 0, 0, 1
        self.D = np.array(calib_data["distortion_coefficients"]["data"])        # [k1, k2, p1, p2, k3]

        self.at_detector = Detector(
            families='tag36h11', 
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        # Camera intrinsics for pose estimation
        self.fx = K[0, 0]
        self.fy = K[1, 1]
        self.cx = K[0, 2]
        self.cy = K[1, 2]

        self.sub_camera = self.create_subscription(
            Image,
            "/camera",
            self.camera_callback,
            10
        )

        self.bridge = CvBridge()

        self.pub_tags = self.create_subscription(
            
        )


    def camera_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Get optimal new camera matrix
        h, w = img.shape[:2]
        new_K, _ = cv2.getOptimalNewCameraMatrix(self.K, self.D, (w, h), 1, (w, h))

        # Undistort the image
        undistorted = cv2.undistort(img, self.K, self.D, None, new_K)

        gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)

        tags = self.at_detector.detect(
            gray, 
            estimate_tag_pose=True, 
            camera_params=(self.fx, self.fy, self.cx, self.cy), 
            tag_size=0.1
        )

        for tag in tags:
            print(f"Detected tag ID: {tag.tag_id}")
            print("Translation (t):", tag.pose_t.flatten())
            print("Rotation matrix (R):\n", tag.pose_R)

            