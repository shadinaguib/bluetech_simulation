import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SerialPublisher(Node):
    def __init__(self):
        super().__init__('serial_publisher')
        self.publisher_ = self.create_publisher(String, 'serial_data', 10)
        self.timer = self.create_timer(0.1, self.publish_serial_data)
        self.serial_port = serial.Serial('/dev/ttyACM0', 9600) #replace with your serial port and baud rate

    def publish_serial_data(self):
        if self.serial_port.in_waiting > 0:
            serial_data = self.serial_port.readline().decode('utf-8').rstrip()
            msg = String()
            msg.data = serial_data
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    serial_publisher = SerialPublisher()
    rclpy.spin(serial_publisher)
    serial_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
