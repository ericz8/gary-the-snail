import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from std_msgs.msg import Int16

class TargetHeadingPublisher(Node):
    def __init__(self):
        super().__init__("tutorial_publisher")    # names the node when running

        self.pub = self.create_publisher(
            Int16,        # the message type
            "target_heading",    # the topic name
            10              # QOS (will be covered later)
        )

        self.get_logger().info("initialized publisher node")

    def publish_target_heading(self, data):
        msg = Int16()

        msg.data = data

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TargetHeadingPublisher()

    try:
        while True:
            val = input("enter target heading in degrees: ")

            try:
                val = int(val)
                val %= 360
            except TypeError:
                node.get_logger().warn("invalid input!")

            node.publish_target_heading(val)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

