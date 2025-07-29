# import rclpy
# from rclpy.node import Node
# from cv_bridge import CvBridge

# import cv2

# from sensor_msgs.msg import Image
# from std_msgs.msg import Float32MultiArray

# class challenge_code(Node): #change name idk
#     def __init__(self):
#         super().__init__("challenge_code") #change name 

#         self.valid_front = []
#         self.valid_back = []

#         self.sub = self.create_subscription(
#             Float32MultiArray,
#             "/tags",
#             self.april_tag_callback,
#             10
#         )
    
#     def april_tag_callback(self, msg):
#         """
#         disable lane following

#         """

        
