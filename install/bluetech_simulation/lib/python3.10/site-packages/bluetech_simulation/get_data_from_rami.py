# wait for rami to finalise the server host and commands from the server




import string
import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32


import socketio

sio = socketio.Client()



@sio.on('connect')
def on_connect():
    print('Connected to the server.')



sio.connect('http://10.250.174.214:5000')



class Int32Publisher(Node):

    def __init__(self):
        super().__init__('int32_publisher')
        self.publisher = self.create_publisher(Int32,'/send_int32to_pico', 10)
        timer_period = 0.0001
        self.counter = 0
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.received_data = "none"
        

    def timer_callback(self):
        send_int32 = Int32()

        @sio.on('message')
        def on_string(data):
            self.received_data = data
            print(f'Received string: {data}')

        if self.received_data == "test":
            print('test')
            send_int32.data = 5
        elif self.received_data == "left":
            send_int32.data = 4
        else:
            send_int32.data = 0

        self.publisher.publish(send_int32)



def main(args=None):
    rclpy.init(args=args)

    velocity_publisher = Int32Publisher()
    
    rclpy.spin(velocity_publisher)
    velocity_publisher.destroy_node()
    velocity_publisher.get_logger().info(f"Shutting down the node")
    rclpy.shutdown()


if __name__ == '__main__':
    main()