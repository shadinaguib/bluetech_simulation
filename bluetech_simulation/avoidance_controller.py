"""forward effort will turn off friction of the left wheel and apply an effort
to the right wheel and vice versa in an infinite loop

The effort is applied to the /right_wheel_effort_controller/commands and
/left_wheel_effort_controller/commands
Use this to move the robot step by step (each step for specific time period)

This was initially used to model the adhesion force in the Gazebo simulation. 
The link force is implemented as a service that applies a specified force on the right
link in the negative Z direction
More info here http://docs.ros.org/en/jade/api/gazebo_msgs/html/srv/ApplyBodyWrench.html

usage
make sure the robot is spawned in Gazebo. Make sure you specify the force applied on
the link as an argument, in the example below, the force applied is 10

ros2 run bluetech_simulation forward_effort"""

import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool, String, Float64
import numpy as np

class AvoidanceController(Node):
    """Controller making the robot move randomly inside a welded rectangle and use a displacement sensor to detect weld lines
    """
    def __init__(self, effort: float):
        super().__init__('avoidance_controller')

        # effort topic publishers
        self.send_string = self.create_publisher(String, '/cmd_vel', 10)        
        self.right_encoder = self.create_subscription(Float64, '/right_encoder', self.change_direction_right_to_left, qos_profile=0)
        self.left_encoder = self.create_subscription(Float64, '/left_encoder', self.change_direction_left_to_right, qos_profile=0)

        # distance sensor subscription 
        self.get_int = self.create_subscription(Bool, "/weld_line_detected", self.avoid_weld_line, qos_profile=0)

        # init the friction service clients        
        self.get_logger().info(f"Frictions of both wheels can now be toggled")

        # callback params        
        self.right_encoder_goal_value = 1
        self.left_encoder_goal_value = 1

        self.turning_right = 0 
        self.turning_left = 1

        self.state = self.turning_right
        self.turn_angle = 1
        
        self.current_right_encoder_value = 0
        self.current_left_encoder_value = 0

        self.get_logger().info("Started avoidance_controller node")

    def change_direction_right_to_left(self, right_encoder_msg):
        """Change the direction of rotation of the robot. Turn around the left foot instead of turning around the right foot.

        Args:
            right_encoder_msg (Float64): right foot motor encoder value coming from the /right_encoder ros2 topic
        """
        motor_command = String()

        if self.state == self.turning_right:
            self.current_right_encoder_value = right_encoder_msg.data
            if right_encoder_msg.data > self.right_encoder_goal_value:
                # stop motors 
                motor_command.data = ' '
                self.send_string.publish(motor_command)
                time.sleep(0.2)
                # disengage right magnet 
                motor_command.data = 'f'
                self.send_string.publish(motor_command)
                time.sleep(0.2)
                # start turning left motor
                motor_command.data = 'l'
                self.send_string.publish(motor_command)
                self.state = self.turning_left
                self.right_encoder_goal_value = self.right_encoder_goal_value + self.turn_angle

    def change_direction_left_to_right(self, left_encoder_msg):
        """Change the direction of rotation of the robot. Turn around the right foot instead of turning around the left foot.

        Args:
            left_encoder_msg (Float64): left foot motor encoder value coming from the /left_encoder ros2 topic
        """
        motor_command = String()

        if self.state == self.turning_left:
            self.current_left_encoder_value = left_encoder_msg.data

            if left_encoder_msg.data > self.left_encoder_goal_value:
                # stop motors
                motor_command.data = ' '
                self.send_string.publish(motor_command)
                time.sleep(0.2)
                # disengage left magnet 
                motor_command.data = 's'
                self.send_string.publish(motor_command)
                time.sleep(0.2)
                # start turning left motor
                motor_command.data = 'r'
                self.send_string.publish(motor_command)
                self.state = self.turning_right
                self.left_encoder_goal_value = self.left_encoder_goal_value + self.turn_angle

    def avoid_weld_line(self, weld_line_status_msg):
        """_summary_

        Args:
            weld_line_status_msg (_type_): _description_
        """
        if weld_line_status_msg.data:
            self.get_logger().info(f"First pass over weld line")
            if self.state == self.turning_right:
                self.right_encoder_goal_value = self.current_right_encoder_value + 10 # (3 is almost half a turn)
            elif self.state == self.turning_left:
                self.left_encoder_goal_value = self.current_left_encoder_value + 10


        else: #this means we got a false, meaning we have gone over the weld line again. 
            self.get_logger().info(f"Got back over the weld line")
            if self.state == self.turning_right:
                self.right_encoder_goal_value = self.current_right_encoder_value + 1.5*np.random.random_sample() + 1.5
                
            else:
                self.left_encoder_goal_value = self.current_left_encoder_value + 1.5*np.random.random_sample() + 1.5
def main(args=None):
    rclpy.init(args=args)

    avoidance_controller = AvoidanceController(1.0)
    
    rclpy.spin(avoidance_controller)

    avoidance_controller.destroy_node()
    avoidance_controller.get_logger().info(f"Shutting down the node")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
