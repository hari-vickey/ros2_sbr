"""
Self Balancing Robot
PID Controller
"""
# Importing Dependencies
import math
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion

from bd1_self_balancer import PID_controller

class SelfBalancingBot(Node):
    """
    Class SelfBalancingBot
    """
    # Constructor
    def __init__(self):
        """
        constructor
        """
        # Initializing Node
        super().__init__('self_balancer')
        self.kp,self.ki,self.kd = 1,0,0
        self.vel = Twist()
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.create_subscription(Odometry, '/odom', self.pos_callback, 1)

    # Position Callback
    def pos_callback(self, msg):
        self.kp = 30
        self.posi = msg.pose.pose.position
        self.orient = msg.pose.pose.orientation
        print("------------Position-------------")
        print("x = %.5f, y = %.5f, z = %.5f" % \
            (msg.pose.pose.position.x, msg.pose.pose.position.y,
             msg.pose.pose.position.z))

        # Quartenion Angular Co-ordinates
        print("---------Quaternion---------")
        print("x = %.5f, y = %.5f, z = %.5f, w = %.5f" % \
            (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        
        error = (0.707-self.orient.w)
        print("error = ", error)
        out = self.kp*error
        
        if self.posi.x < 0 and self.orient.w <= 0.9999:
            self.vel.linear.x = -out
            # self.vel.linear.x = -2.0
            print("Wheel rotating backward , veloity = ",self.vel.linear.x)
        else:
            # self.vel.linear.x = 2.0
            self.vel.linear.x = out
            print("Wheel rotating forward , veloity = ", self.vel.linear.x )
        self.publisher.publish(self.vel)

# Main Function
def main():
    """
    Main Function to instantiate Class
    """
    # Initializing ROS2
    rclpy.init()
    try:
        # Instantiation of the class SelfBalancingBot
        self_balancer = SelfBalancingBot()
        # Spin forever
        rclpy.spin(self_balancer)
    except KeyboardInterrupt:
        # Destroy the Node
        self_balancer.destroy_node()

if __name__ == '__main__':
    main()