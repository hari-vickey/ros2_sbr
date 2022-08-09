"""
Self Balancing Robot
Controller
"""
# Importing Dependencies
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class SelfBalancingBot(Node):
    def __init__(self):
        super().__init__('self_balancer')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.create_subscription(Odometry, '/odom', self.pos_callback, 1)

    def pos_callback(self, msg):
        print("------------Position-------------")
        print("x = %.5f, y = %.5f, z = %.5f" % \
            (msg.pose.pose.position.x, msg.pose.pose.position.y,
             msg.pose.pose.position.z))
        # Quartenion Angular Co-ordinates
        print("------------Quartenion-------------")
        print("x = %.5f, y = %.5f, z = %.5f, w = %.5f" % \
            (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        velocity = Twist()
        velocity.linear.x = -0.25
        self.publisher.publish(velocity)

def main():
    rclpy.init()
    try:
        self_balancer = SelfBalancingBot()
        rclpy.spin(self_balancer)
    except KeyboardInterrupt:
        self_balancer.destroy_node()


if __name__ == '__main__':
    main()