import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from mavros_msgs.msg import ManualControl    # the Vector3 message type definition
from time import time, sleep

TURN_180_TIME = 0 # seconds
TURN_360_TIME = 0 # seconds

class Movement(Node):
    def __init__(self):
        super().__init__("movement")    # names the node when running
        
        self.LIST_MOVES = [("forward", 2), ("counter", 2), ("left", 2), ("clock", 2), ("left", 2), ("counter", 2), ("right", 2), ("clock", 2), ("right", 1), ("left", 1), ("right", 1), ("forward", 1), ("backward", 1), ("forward", 1), ("clock", 2), ("counter", 2), ("down", 1), ("up", 1), ("clock", 2), ("counter", 2), ("down", 1), ("up", 1), ("down", 3), ("up", 3), ("counter", 2), ("clock", 2), ("right", 1),("down", 1), ("left", 1), ("right", 1), ("left", 1), ("right", 3), ("left", 2), ("right", 2), ("backward", 3), ("left", 3)] # each move is a tuple(move_type, time)
        # self.LIST_MOVES = [("forward", 2), ("left", 2), ("left", 2), ("right", 2), ("right", 1), ("left", 1), ("right", 1), ("forward", 1), ("backward", 2), ("down", 1), ("up", 1), ("down", 1), ("up", 1), ("down", 3), ("up", 1), ("up", 1), ("up", 1), ("right", 1),("down", 1), ("left", 1), ("right", 1), ("left", 1), ("right", 3), ("left", 2), ("right", 2), ("forward", 3), ("left", 3)] # each move is a tuple(move_type, time)
        # self.LIST_MOVES = [("counter", 2), ("clock",2)]
        self.pub = self.create_publisher(
            ManualControl,        # the message type
            "manual_control",    # the topic name
            10              # QOS (will be covered later)
        )

        self.get_logger().info("initialized movement node")

        # self.play_moves()

    def publish_move(self, move, pct=0.0):
        """
        Does a move relative to the AUV

        Options:
        - up
        - down
        - left
        - right
        - forward
        - backward
        - turn (category)
            - clock
            - counter
        """
        msg = ManualControl()

        
        msg.x = 0.0
        msg.y = 0.0
        msg.z = 0.0
        msg.r = 0.0

        if move == "forward":
            msg.x = pct
        elif move == "backward":
            msg.x = -pct
        elif move == "left":
            msg.y = -pct
        elif move == "right":
            msg.y = pct
        elif move == "up":
            msg.z = pct+20
        elif move == "down":
            msg.z = -pct
        elif move == "clock":
            msg.r = pct
        elif move == "counter":
            msg.r = -pct
        else:
            msg.x = 0.0
            msg.y = 0.0
            msg.z = 0.0
            msg.r = 0.0
        
        self.pub.publish(msg)

    def time_move(self, move, t):
        start = time()
        
        while time() - start < t:
            if time() - start >= 0.9 * t:
                self.publish_move(move, pct=-10.0)
            else:
                self.publish_move(move, pct=50.0)
            sleep(0.1)

        self.publish_move("stop")

    def play_moves(self):
        for move in self.LIST_MOVES:
            self.get_logger().info("started " + move[0])

            self.time_move(move[0], move[1])
            
            self.get_logger().info("ended " + move[0])
        
        self.get_logger().info("stopped dance")

def main(args=None):
    rclpy.init(args=args)

    node = Movement()

    node.play_moves()

    
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

