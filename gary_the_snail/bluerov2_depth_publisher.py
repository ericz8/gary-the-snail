import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from sensor_msgs.msg import FluidPressure 
from std_msgs.msg import Float32

ATMOSPHERIC_PRESSURE = 101325
GRAVITATIONAL_ACC = 9.81
WATER_DENSITY = 1000

class DepthPublisher(Node):
    def __init__(self):
        super().__init__("pressure_converter")

        self.sub = self.create_subscription(
            FluidPressure,
            "/pressure",
            self.depth_callback,
            10
        )
        
        self.pub = self.create_publisher(
            Float32,
            "/depth",
            10
        )

        self.get_logger().info("created depth publisher node")

    def depth_callback(self, msg):
        depth_msg = Float32()
        depth_msg.data = (msg.fluid_pressure - ATMOSPHERIC_PRESSURE) / (WATER_DENSITY * GRAVITATIONAL_ACC)
        self.pub.publish(depth_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DepthPublisher()    

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()