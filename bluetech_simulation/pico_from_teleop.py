import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32


class PicoSub(Node):

    def __init__(self):
        super().__init__('pico_from_teleop')

        # velocity subscriber
        self.get_command = self.create_subscription(String, "/cmd_vel", self.vel_callback, 1)
        self.send_command = self.create_publisher(Int32, '/send_int32to_pico', 1)
        self.data = Int32()


    def vel_callback(self, msg: String):
        """Send the command to the microcontroller depending on the string received from the /cmd_vel topic

        Args:
            msg (String): string coming from the /cmd_vel topic indicating what command should be sent to the robot.
        """
        if msg.data == "l":
            self.data.data = 1
            self.get_logger().info(f"Sent 1 to PICO, move the LM_A motor")  
            self.send_command.publish(self.data)
        elif msg.data == 'a':
            self.data.data = 3
            self.get_logger().info(f"Sent 3 to PICO, engage left magnet")
            self.send_command.publish(self.data)
        elif msg.data == 's':
            self.data.data = 2
            self.get_logger().info(f"Sent 2 to PICO, disengage left magnet")
            self.send_command.publish(self.data)

        elif msg.data == "r":
            self.data.data = 4
            self.get_logger().info(f"Sent 4 to PICO, move the RM_A motor")  
            self.send_command.publish(self.data)
        elif msg.data == 'f':
            self.data.data = 5
            self.get_logger().info(f"Sent 5 to PICO, disengage right magnet")  
            self.send_command.publish(self.data)
        elif msg.data == 'd':
            self.data.data = 6
            self.get_logger().info(f"Sent 6 to PICO, engage right magnet")  
            self.send_command.publish(self.data)

        else:
            self.data.data = 0
            self.get_logger().info(f"Sent 0 to PICO, stop motors")
            self.send_command.publish(self.data)




def main(args=None):
    rclpy.init(args=args)

    pico_sub = PicoSub()
    
    rclpy.spin(pico_sub)

    pico_sub.destroy_node()
    pico_sub.get_logger().info(f"Shutting down the node")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
