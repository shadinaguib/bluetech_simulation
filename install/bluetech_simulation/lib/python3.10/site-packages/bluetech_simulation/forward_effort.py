# forward effort will turn off friction of the left wheel and apply an effort
# to the right wheel and vice versa in an infinite loop
# 
# The effort is applied to the /right_wheel_effort_controller/commands and
# /left_wheel_effort_controller/commands
# Use this to move the robot step by step (each step for specific time period)

# This was initially used to model the adhesion force in the Gazebo simulation. 
# The link force is implemented as a service that applies a specified force on the right
# link in the negative Z direction
# More info here http://docs.ros.org/en/jade/api/gazebo_msgs/html/srv/ApplyBodyWrench.html

# usage
# make sure the robot is spawned in Gazebo. Make sure you specify the force applied on
# the link as an argument, in the example below, the force applied is 10

# ros2 run bluetech_simulation forward_effort

import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool, String


class FeedForward(Node):

    def __init__(self, effort: float):
        super().__init__('feed_forward_controller')

        # effort topic publishers
        self.send_string = self.create_publisher(String, '/cmd_vel', 10)

        self.effort_right = self.create_publisher(Float64MultiArray, '/right_wheel_effort_controller/commands', 10)
        self.effort_left = self.create_publisher(Float64MultiArray, '/left_wheel_effort_controller/commands', 10)
        
        # init the friction service clients
        self.friction = self.create_publisher(String,'/toggle/friction_status', 10)

        
        self.get_logger().info(f"Frictions of both wheels can now be toggled")

        # callback params
        timer_period = 0.0001
        self.effort = effort
        
        # initializae messages
        self.effort_command = Float64MultiArray()      
        self.counter = 0
        self.time_period = 5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        

    def timer_callback(self):

        # set the requests to be true to turn friction on every iteration
        friction_status = String()
        moving_motor = String()



        # increment counter
        self.counter += 1

        # switch between left and right wheels

        if self.counter % 2 == 0:
            friction_status.data = '01'
            self.friction.publish(friction_status)
            # right wheel friction on
            self.get_logger().info(f"Turn right friction on")
            # left wheel friction off
            self.get_logger().info(f"Turn left friction off")
            
            # move the right wheel
            self.effort_command.data = [self.effort]
            moving_motor.data = "r"
            self.send_string.publish(moving_motor)
            self.effort_right.publish(self.effort_command)
            time.sleep(self.time_period)
        else:
            friction_status.data = '10'
            self.friction.publish(friction_status)
            # left wheel friction on
            self.get_logger().info(f"Turn left friction on")
            # right wheel friction off
            self.get_logger().info(f"Turn right friction off")

            self.effort_command.data = [-self.effort]
            moving_motor.data = "l"
            self.send_string.publish(moving_motor)
            self.effort_left.publish(self.effort_command)
            time.sleep(self.time_period)




def main(args=None):
    rclpy.init(args=args)

    feed_forward_controller = FeedForward(1.0)
    
    rclpy.spin(feed_forward_controller)

    feed_forward_controller.destroy_node()
    feed_forward_controller.get_logger().info(f"Shutting down the node")
    rclpy.shutdown()


if __name__ == '__main__':
    main()