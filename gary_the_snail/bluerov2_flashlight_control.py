import rclpy
from rclpy.node import Node

from mavros_msgs.msg import OverrideRCIn
from std_msgs.msg import Bool
from time import sleep


class FlashlightControl(Node):
    def __init__(self):
        super().__init__("flashlight_control")
        
        self.pub_light = self.create_publisher(
            OverrideRCIn, 
            "override_rc", 
            10
        )

        self.sub_light = self.create_subscription(
            Bool,
            "flash",
            self.flash_robot,
            10
        )
    
    def flash_robot(self, msg):
        """
        level goes from 0 to 100
        """

        level = 0
        if msg.data:
            level = 100

        self.get_logger().info(f"turning on lights to level {level}")

        commands = OverrideRCIn()
        commands.channels = [OverrideRCIn.CHAN_NOCHANGE] * 10
        commands.channels[8] = 1000 + level * 10
        commands.channels[9] = 1000 + level * 10
        self.pub_light.publish(commands)

def main(args=None):
    rclpy.init(args=args)
    node = FlashlightControl()    

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()