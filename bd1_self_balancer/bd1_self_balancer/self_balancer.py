"""
Self Balancing Robot
tilt_pid Controller
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

        # tilt_pid variables
        self.set_theta = 0.0
        self.tilt_Kp, self.tilt_Ki, self.tilt_Kd = 100.0, 0.0, 0.0
        self.curr_theta, self.prev_tilt_error, self.tilt_error_sum = 0.0, 0.0, 0.0

        # pos_pid variables
        self.set_pos = 0.05
        self.pos_Kp, self.pos_Ki, self.pos_Kd = 20.0, 0.0, 0.0
        self.curr_pos, self.prev_pos_error, self.pos_error_sum = 0.0, 0.0, 0.0

        # yaw_pid values
        self.set_yaw = 0.0
        self.yaw_Kp, self.yaw_Ki, self.yaw_Kd = 10.0, 0.0, 0.0
        self.curr_yaw, self.prev_yaw_error, self.yaw_error_sum = 0.0, 0.0, 0.0

        self.create_subscription(Odometry, '/odom', self.pos_callback, 1)

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.velocity = Twist()
        # Initializing Velocity to zero
        self.velocity.linear.x, self.velocity.linear.y = 0.0, 0.0
        self.velocity.linear.z, self.velocity.angular.x = 0.0, 0.0
        self.velocity.angular.y, self.velocity.angular.z = 0.0, 0.0

    # PID Controller
    def pid_controller(self, *pid):
        """
        Computes pid output for the input state values
        """
        Kp, Ki, Kd, curr_value, set_value, prev_error, error_sum = pid
        # Computing Error (for Proportional Term)
        error = curr_value - set_value

        # Computing Change in Error (For Derivative Term)
        change_in_error = error - prev_error

        # Computing Sum of errors (For Integral Term)
        error_sum += error

        # Computing the output of controller
        p = Kp * error
        i = Ki * error_sum
        d = Kd * change_in_error
        pid = p + i + d

        prev_error = error

        return prev_error, error_sum, pid

    # Display PID Values
    def display_pid(self, *msg):
        """
        Display PID values of all the pid controllers.
        """
        # Position of the Bot
        print("-----------------------------------------")
        print("-----------Position (in meter)-----------")
        print("x = %.5f, y = %.5f, z = %.5f" % (msg[0], msg[1], msg[2]))

        # Quaternion Position
        print("---------Quaternion (in radians)---------")
        print("x = %.5f, y = %.5f, z = %.5f, w = %.5f" % (msg[3], msg[4], msg[5], msg[6]))

        # Euler Angles
        print("-------roll pitch yaw (in radians)-------")
        print("r = %.5f, p = %.5f, y = %.5f" % (self.curr_theta, msg[7], msg[8]))
        print("-------roll pitch yaw (in degrees)-------")
        print("r = %.5f, p = %.5f, y = %.5f" % (math.degrees(self.curr_theta),
                                                math.degrees(msg[7]), math.degrees(msg[8])))
        # tilt_pid Values
        # print("-----------------P I D (theta) -------------------")
        # print("Previous Error = %f" % self.prev_tilt_error)
        # print("Error Sum = %f" % self.tilt_error_sum)
        # print("theta PID = %f" % msg[9])
        # print("Wheel Velocity = %f" % self.velocity.linear.x)
        # # print("p = %.5f, i = %.5f, d = %.5f" % (theta_p, theta_i, theta_d))
        # print("----------------------------------------\n")

        # # pos_pid Values
        # print("-----------------P I D (pos) -------------------")
        # print("Previous Error = %f" % self.prev_pos_error)
        # print("Error Sum = %f" % self.pos_error_sum)
        # print("Pos PID = %f" % msg[10])
        # print("Wheel Velocity = %f" % self.velocity.linear.x)
        # print("----------------------------------------\n")

        print("-----------------P I D (yaw) -------------------")
        print("Previous Error = %f" % self.prev_yaw_error)
        print("Error Sum = %f" % self.yaw_error_sum)
        print("YAW PID = %f" % msg[11])
        print("Wheel angular velocity = %f" % self.velocity.angular.z)
        print("----------------------------------------\n")

    # Position Callback
    def pos_callback(self, msg):
        """
        callback function for ROS2 Subscriber to the topic /odom
        """

        position_list = [msg.pose.pose.position.x, msg.pose.pose.position.y,
                         msg.pose.pose.position.z]

        orientation_list = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]

        # tracking current position of the robot
        self.curr_pos = msg.pose.pose.position.y

        # Converting Quaternion to euler angles
        self.curr_theta, p, self.curr_yaw = euler_from_quaternion(orientation_list)

        # Tilt controller
        tilt_pid_ls = [self.tilt_Kp, self.tilt_Ki, self.tilt_Kd, self.curr_theta,
                       self.set_theta, self.prev_tilt_error, self.tilt_error_sum]
        self.prev_tilt_error, self.tilt_error_sum, tilt_pid = self.pid_controller(*tilt_pid_ls)

        # Position Controller
        pos_pid_ls = [self.pos_Kp, self.pos_Ki, self.pos_Kd, self.curr_pos,
                      self.set_pos, self.prev_pos_error, self.pos_error_sum]
        self.prev_pos_error, self.pos_error_sum, pos_pid = self.pid_controller(*pos_pid_ls)

        # Yaw Controller
        yaw_pid_ls = [self.yaw_Kp, self.yaw_Ki, self.yaw_Kd, self.curr_yaw,
                      self.set_yaw, self.prev_yaw_error, self.yaw_error_sum]
        self.prev_yaw_error, self.yaw_error_sum, yaw_pid = self.pid_controller(*yaw_pid_ls)

        # wheel velocity publisher
        self.velocity.linear.x = tilt_pid - pos_pid
        # self.velocity.angular.z = -1 *  yaw_pid/1000
        self.publisher.publish(self.velocity)

        msg_values = position_list + orientation_list + [p, self.curr_yaw, tilt_pid, pos_pid, yaw_pid]
        self.display_pid(*msg_values)

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