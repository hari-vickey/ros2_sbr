"""
Self Balancing Robot
PID Controller
"""
# Importing Dependencies
import math
import time
import rclpy
import threading
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion

# Class Self Balancing Bot
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
        # PID Variables
        self.set_pos, self.Kp, self.Ki, self.Kd = 0.0, 2, 0, 300
        self.curr_pos, self.prev_error, self.error_sum = 1e-4, 0, 0

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.create_subscription(Odometry, '/odom', self.pos_callback, 1)
        self.velocity = Twist()
        # Initializing Velocity to zero
        self.velocity.linear.x, self.velocity.linear.y = 0.0, 0.0
        self.velocity.linear.z, self.velocity.angular.x = 0.0, 0.0
        self.velocity.angular.y, self.velocity.angular.z = 0.0, 0.0

    # Position Callback
    def pos_callback(self, msg):
        """
        callback function for ROS2 Subscriber to the topic /odom
        """
        orientation_list = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]

        # Converting Quaternion to euler angles
        self.curr_pos, p, y = euler_from_quaternion(orientation_list)

        # Computing Error (for Proportional Term)
        self.error = self.curr_pos - self.set_pos

        # Computing Change in Error (For Derivative Term)
        self.change_in_error = self.error - self.prev_error

        # Computing Sum of errors (For Integral Term)
        self.error_sum += self.error

        # Computing the output of controller
        if abs(self.error) > 0.01:
            p = self.Kp * self.error
            i = self.Ki * self.error_sum
            d = self.Kd * self.change_in_error
            pid = p + i + d
            self.velocity.linear.x = pid*math.exp(4)
        else:
            self.error_sum, self.change_in_error = 0.0, 0.0
            self.prev_error, self.error, pid = 0.0, 0.0, 0.0
            self.velocity.linear.x, p, i, d = 0.0, 0.0, 0.0, 0.0

        # Publishing Velocity
        self.publisher.publish(self.velocity)

        # Position of the Bot
        print("-----------------------------------------")
        print("-----------Position (in meter)-----------")
        print("x = %.5f, y = %.5f, z = %.5f" % \
            (msg.pose.pose.position.x, msg.pose.pose.position.y,
             msg.pose.pose.position.z))
        # Quaternion Position
        print("---------Quaternion (in radians)---------")
        print("x = %.5f, y = %.5f, z = %.5f, w = %.5f" % \
            (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        # Euler Angles
        print("-------roll pitch yaw (in radians)-------")
        print("r = %.5f, p = %.5f, y = %.5f" % (self.curr_pos, p, y))
        print("-------roll pitch yaw (in degrees)-------")
        print("r = %.5f, p = %.5f, y = %.5f" % (math.degrees(self.curr_pos),
                                                math.degrees(p), math.degrees(y)))
        # PID Values
        print("-----------------P I D-------------------")
        print("Pitch Error = %f" % self.error)
        print("Previous Error = %f" % self.prev_error)
        print("Error Sum = %f" % self.error_sum)
        print("Change in Error = %f" % self.change_in_error)
        print("PID = %f" % self.velocity.linear.x)
        print("p = %.5f, i = %.5f, d = %.5f" % (p, i, d))
        print("----------------------------------------\n")

        # Updating pervious errors
        self.prev_error = self.error

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