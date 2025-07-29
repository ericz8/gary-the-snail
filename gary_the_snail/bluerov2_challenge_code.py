import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

import cv2

from sensor_msgs.msg import Image

class challenge_code(Node): #change name idk
    def __init__(self):
        super().__init__("challenge_code") #change name 

        self.valid_front = []
        self.valid_back = []
    
    def april_tag_callback(self, msg):
        """
        disable lane following

        """
        tags = msg.data
        
        
        
