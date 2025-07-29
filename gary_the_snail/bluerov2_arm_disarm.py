from std_srvs.srv import SetBool

import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class

from time import time, sleep

TIME_TO_MOVE = 300 # seconds

class ArmDisarm(Node):
    def __init__(self):
        super().__init__("arm_disarm")    # names the node when running

        self.cli = self.create_client(
            SetBool,
            "/arming"
        )

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("arming service not available, waiting...")

        self.req = SetBool.Request()

    def send_request(self, data):
        self.req.data = data
        return self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)
    node = ArmDisarm()
    future = node.send_request(True)
    rclpy.spin_until_future_complete(node, future)
    response = future.result()

    if response.success:
        node.get_logger().info("arm successful!")
    else:
        node.get_logger().warn("arm failed!")

    start = time()
    while (time() - start < TIME_TO_MOVE):
        node.get_logger().info(str(time() - start))
        try:
            sleep(0.1)
        except KeyboardInterrupt:
            break

    future = node.send_request(False)
    rclpy.spin_until_future_complete(node, future)
    response = future.result()

    if response.success:
        node.get_logger().info("disarm successful!")
    else:
        node.get_logger().warn("disarm failed!")
    
    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()