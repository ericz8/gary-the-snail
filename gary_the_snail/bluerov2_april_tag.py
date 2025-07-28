import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

from dt_apriltags import Detector
import matplotlib as plt
import cv2
from cv_bridge import CvBridge

class april_tag_detector(Node):
    def __init__(self):
        super().__init__("april_tag_detector")

        self.sub_camera = self.create_subscription(
            Image,
            "/camera",
            self.camera_callback,
            10
        )

        self.bridge = CvBridge()


    def camera_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
        img = cv2.imread(img, cv2.IMREAD_GRAYSCALE)
        at_detector = Detector(families='tag36h11', 
                            nthreads=1,
                            quad_decimate=1.0,
                            quad_sigma=0.0,
                            refine_edges=1,
                            decode_sharpening=0.25,
                            debug=0)
        tags = at_detector.detect(img, estimate_tag_pose=True, camera_params=None, tag_size=None)

        