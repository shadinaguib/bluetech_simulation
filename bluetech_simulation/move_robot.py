import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool, String


class MoveRobot(Node):

    def __init__(self, effort: float):
        super().__init__('move_robot')

        # effort topic publishers
        self.get_command = self.create_subscription(String, "/cmd_vel", self.vel_callback, 1)

        self.effort_right = self.create_publisher(Float64MultiArray, '/right_wheel_effort_controller/commands', 10)
        self.effort_left = self.create_publisher(Float64MultiArray, '/left_wheel_effort_controller/commands', 10)
        
        # init the friction clients
        self.friction = self.create_publisher(String,'/toggle/friction_status', 10)
        
        self.get_logger().info(f"Frictions of both wheels can now be toggled")

        self.friction_status = String()
       
        self.effort = effort
        
        # initialize messages
        self.effort_command = Float64MultiArray()
        
    def angle_to_time(self, angle):
        pass


    def vel_callback(self, msg: String):

        friction_status = self.friction_status
        self.get_logger().info(f"Previous friction status : {self.friction_status.data}")

        if msg.data == 'a':
            if self.friction_status.data == '01':
                self.friction_status.data = '11'
                self.get_logger().info(f"Both friction on")

            elif self.friction_status.data == '00':
                self.friction_status.data = '10'
                self.get_logger().info(f"Left friction on")
            else: 
                self.get_logger().info(f"Nothing has changed.")
            self.friction.publish(self.friction_status)

        elif msg.data == 's':
            #disengage left magnet
            if self.friction_status.data == '11':
                self.friction_status.data = '01'
                self.get_logger().info(f"Left friction off")

            elif self.friction_status.data == '10':
                self.friction_status.data = '00'
                self.get_logger().info(f"Both friction off")

            else:   
                self.get_logger().info(f"Nothing has changed")
            self.friction.publish(self.friction_status)
        elif msg.data == 'f':
            #disengage right magnet
            if self.friction_status.data == '11':
                self.friction_status.data = '10'
                self.get_logger().info(f"Right friction off")
            elif self.friction_status.data == '01':
                self.friction_status.data = '00'
                self.get_logger().info(f"Both friction off")
            else: 
                self.get_logger().info(f"Nothing has changed")
            self.friction.publish(self.friction_status)

        elif msg.data == 'd':
            if self.friction_status.data == '10':
                self.friction_status.data = '11'
                self.get_logger().info(f"Both friction on")
            elif self.friction_status.data == '00':
                self.friction_status.data = '01'
                self.get_logger().info(f"Right friction on")
            else: 
                self.get_logger().info(f"Nothing has changed")
            self.friction.publish(self.friction_status)

        elif msg.data == "r":
        
            # move the right wheel
            self.effort_command.data = [self.effort]
            self.effort_right.publish(self.effort_command)
            #time_to_sleep = 8 #angle_to_time(90)

            #time.sleep(time_to_sleep)
            #self.effort_command.data = [0.0]
            #self.effort_right.publish(self.effort_command)
            #self.friction_status.data = '11'
            #self.friction.publish(self.friction_status)
            #self.get_logger().info(f"Just stopped the motors!")

        elif msg.data == "l":
            
            self.effort_command.data = [-self.effort]
            self.effort_left.publish(self.effort_command)

        else:
            
            self.friction_status.data = '11'
            self.friction.publish(friction_status)
            # both wheel friction on
            self.get_logger().info(f"Both friction on")

            # stop both the wheels
            self.effort_command.data = [0.0]
            self.effort_left.publish(self.effort_command)
            self.effort_right.publish(self.effort_command)

        self.get_logger().info(f"New friction status : {self.friction_status.data}")




def main(args=None):
    rclpy.init(args=args)

    move_robot = MoveRobot(1.0)
    
    rclpy.spin(move_robot)

    move_robot.destroy_node()
    move_robot.get_logger().info(f"Shutting down the node")
    rclpy.shutdown()


if __name__ == '__main__':
    main()