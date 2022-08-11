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

<<<<<<< Updated upstream
# Class Self Balancing Bot
=======
from bd1_self_balancer import PID_controller

>>>>>>> Stashed changes
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
<<<<<<< Updated upstream
        # PID Variables
        self.position = [0.0, 0.0, 0.0]
        self.Kp, self.Ki, self.Kd = 16.5, 1e-5, 350
        self.p, self.prev_error, self.error_sum = 1e-4, 0, 0

=======
        self.vel = Twist()
>>>>>>> Stashed changes
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.create_subscription(Odometry, '/odom', self.pos_callback, 1)
        self.velocity = Twist()

    # Position Callback
    def pos_callback(self, msg):
<<<<<<< Updated upstream
        """
        callback function for ROS2 Subscriber to the topic /odom
        """
        print("----------------------------")
        print("----------Position----------")
=======
        self.posi = msg.pose.pose.position
        self.orient = msg.pose.pose.orientation 

        print("------------Position-------------")
>>>>>>> Stashed changes
        print("x = %.5f, y = %.5f, z = %.5f" % \
            (msg.pose.pose.position.x, msg.pose.pose.position.y,
             msg.pose.pose.position.z))

        # Quartenion Angular Co-ordinates
        print("---------Quaternion---------")
        print("x = %.5f, y = %.5f, z = %.5f, w = %.5f" % \
            (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
<<<<<<< Updated upstream
        orientation_list = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]

        # Converting Quaternion to euler angles
        self.r, self.p, self.y = euler_from_quaternion(orientation_list)
        print("-------roll pitch yaw-------")
        print(self.r, self.p, self.y)

        # Computing Error (for Proportional Term)
        self.pitch_error = self.position[1] - self.p
        print("Pitch Error = %f" % self.pitch_error)

        # Computing Change in Error (For Derivative Term)
        self.change_in_error = self.pitch_error - self.prev_error
        print("Change in Error = %f" % self.change_in_error)
=======
        if self.posi.x > 0:
            self.vel.linear.x = -2.0
            print("Wheel rotating backward ")
        else:
            self.vel.linear.x = 2.0
            print("Wheel rotating forward")
        self.publisher.publish(self.vel)
>>>>>>> Stashed changes

        # Computing the output of controller
        self.velocity.linear.x = (self.Kp * self.pitch_error) + \
                                 (self.Ki * self.error_sum) + \
                                 (self.Kd * self.change_in_error)
        print("PID = %f" % self.velocity.linear.x)

        # Computing Sum of errors (For Integral Term)
        self.error_sum += self.pitch_error
        print("Error Sum = %f" % self.error_sum)

        # Updating pervious errors
        self.prev_error = self.pitch_error
        print("Previous Error = %f" % self.prev_error)

        # print(self.velocity.linear.x)
        self.publisher.publish(self.velocity)

        # Reducing sudden change
        time.sleep(0.05)

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