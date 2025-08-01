import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from std_msgs.msg import Float32

class TargetDepthPublisher(Node):
    def __init__(self):
        super().__init__("tutorial_publisher")    # names the node when running

        self.pub = self.create_publisher(
            Float32,        # the message type
            "target_depth",    # the topic name
            10              # QOS (will be covered later)
        )

        self.get_logger().info("initialized publisher node")

    def publish_target_depth(self, data):
        msg = Float32()

        msg.data = data

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TargetDepthPublisher()

    try:
        while True:
            val = input("enter target depth (float): ")

            try:
                val = float(val)
            except TypeError:
                node.get_logger().warn("invalid input!")

            node.publish_target_depth(val)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

