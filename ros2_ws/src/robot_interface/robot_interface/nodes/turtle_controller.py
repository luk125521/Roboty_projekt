# turtle_controller.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.sub = self.create_subscription(String, '/aruco/pos', self.aruco_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.forward_speed = 0.15
        self.moving = False

    def aruco_cb(self, msg):
        status = msg.data
        twist = Twist()
        if status == "above":
            twist.linear.x = self.forward_speed
            self.moving = True
        else:
            twist.linear.x = 0.0
            self.moving = False
        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
