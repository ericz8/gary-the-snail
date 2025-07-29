import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

from dt_apriltags import Detector
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray

import yaml
import numpy as np

class april_tag_detector(Node):
    def __init__(self):
        super().__init__("april_tag_detector")

        # with open("/full_path_to/blue_rov_camera.yaml", 'r') as f: # replace with your saved path from previous step
        #     calib_data = yaml.safe_load(f)

        # self.K = np.array(calib_data["camera_matrix"]["data"]).reshape((3, 3))  # fx, 0, cx, 0, fy, cy, 0, 0, 1
        # self.D = np.array(calib_data["distortion_coefficients"]["data"])        # [k1, k2, p1, p2, k3]

        self.at_detector = Detector(
            families='tag36h11', 
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        # # Camera intrinsics for pose estimation
        # self.fx = self.K[0, 0]
        # self.fy = self.K[1, 1]
        # self.cx = self.K[0, 2]
        # self.cy = self.K[1, 2]

        self.sub_camera = self.create_subscription(
            Image,
            "/camera",
            self.camera_callback,
            10
        )

        self.bridge = CvBridge()

        self.pub_tags = self.create_publisher(
            Float32MultiArray,
            "/tags",
            10
        )


    def camera_callback(self, msg):
        self.get_logger().info("hi")
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # # Get optimal new camera matrix
        # h, w = img.shape[:2]
        # new_K, _ = cv2.getOptimalNewCameraMatrix(self.K, self.D, (w, h), 1, (w, h))

        # # Undistort the image
        # undistorted = cv2.undistort(img, self.K, self.D, None, new_K)

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # change img to undistorted

        tags = self.at_detector.detect(
            gray, 
            estimate_tag_pose=False, 
            camera_params=None, 
            tag_size=0.1
        )

        for tag in tags:
            msg = Float32MultiArray()

            # msg.data = [float(tag.tag_id)] + [float(t) for t in tag.pose_t.flatten()] + [float(t) for t in tag.pose_R]
            print(f"Detected tag ID: {tag.tag_id}")
            # print("Translation (t):", tag.pose_t.flatten())
            # print("Rotation matrix (R):\n", tag.pose_R)

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