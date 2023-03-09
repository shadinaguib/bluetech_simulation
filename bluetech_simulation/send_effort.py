import string
import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class EffortController(Node):

    def __init__(self, effort: float):
        super().__init__('send_effort')
        self.publisher_right = self.create_publisher(Float64MultiArray,\
                                            '/right_wheel_effort_controller/commands', 10)
        self.publisher_left = self.create_publisher(Float64MultiArray,\
                                            '/left_wheel_effort_controller/commands', 10)
        timer_period = 0.2
        self.effort = effort
        self.timer = self.create_timer(timer_period, self.timer_callback)        

    def timer_callback(self):
        effort_command = Float64MultiArray()
        effort_command.data = [self.effort]
        self.publisher_right.publish(effort_command)
        # effort_command.data = [-self.effort]
        # self.publisher_left.publish(effort_command)
        self.get_logger().info(f"Effort of {self.effort} applied")


def main(args=None):
    rclpy.init(args=args)

    effort_publisher = EffortController(float(sys.argv[1]))
    
    rclpy.spin(effort_publisher)
    effort_publisher.destroy_node()
    effort_publisher.get_logger().info(f"Shutting down the node")
    rclpy.shutdown()


if __name__ == '__main__':
    main()