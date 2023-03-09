import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SerialSubscriber(Node):
    def __init__(self):
        super().__init__('serial_subscriber')
        self.subscription = self.create_subscription(String, 'serial_data', self.serial_data_callback, 10)
        self.serial_port = serial.Serial('/dev/ttyACM0', 9600) #replace with your serial port and baud rate

    def serial_data_callback(self, msg):
        data = msg.data.encode('utf-8')
        self.serial_port.write(data)

def main(args=None):
    rclpy.init(args=args)
    serial_subscriber = SerialSubscriber()
    rclpy.spin(serial_subscriber)
    serial_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
