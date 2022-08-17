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
        self.set_pos = 0.0
        self.Kp, self.Ki, self.Kd = 2, 0, 300
        self.curr_pos, self.prev_error, self.error_sum = 1e-4, 0, 0

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.create_subscription(Odometry, '/odom', self.pos_callback, 1)
        self.velocity = Twist()

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
        self.pitch_error = self.curr_pos - self.set_pos

        # Computing Sum of errors (For Integral Term)
        self.error_sum += self.pitch_error # This value is not used in trial and error method  

        # Computing Change in Error (For Derivative Term)
        self.change_in_error = self.pitch_error - self.prev_error

        # Computing the output of controller
        self.velocity.linear.x = ((self.Kp * self.pitch_error) + \
                                 (self.Ki * self.error_sum) + \
                                 (self.Kd * self.change_in_error))

        self.velocity.linear.x = self.velocity.linear.x*math.exp(4)

        # Constraining Error Sum in the range -300 to 300
        # if int(self.error_sum) not in range(-300, 300):
        #     if self.error_sum <= 0:
        #         self.error_sum = self.error_sum + 300
        #     else:
        #         self.error_sum = self.error_sum - 300

        # Updating pervious errors
        self.prev_error = self.pitch_error

        # print(self.velocity.linear.x)
        self.publisher.publish(self.velocity)

        # Position of the Bot
        print("----------------------------")
        print("----------Position----------")
        print("x = %.5f, y = %.5f, z = %.5f" % \
            (msg.pose.pose.position.x, msg.pose.pose.position.y,
             msg.pose.pose.position.z))
        # Quaternion Position
        print("---------Quaternion---------")
        print("x = %.5f, y = %.5f, z = %.5f, w = %.5f" % \
            (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        # Euler Angles
        print("-------roll pitch yaw-------")
        print(math.degrees(self.curr_pos), math.degrees(p), math.degrees(y))
        # PID Values
        print("---------PID Values---------")
        print("Previous Error = %f" % self.prev_error)
        print("Error Sum = %f" % self.error_sum)
        print("PID = %f" % self.velocity.linear.x)
        print("Change in Error = %f" % self.change_in_error)
        print("Pitch Error = %f" % self.pitch_error)

        print("Speed = ", self.velocity.linear.x)
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