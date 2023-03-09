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



# sio.connect('http://10.250.145.31:5000')
try:
    sio.connect('http://192.168.0.102:5000', wait_timeout=10)
except:
    print(f"Server not found at 192.168.0.102:5000")



class GetFromServerSendToPico(Node):

    def __init__(self):
        super().__init__('send_int_to_pico')
        
        # send the integer to the pico
        self.publisher = self.create_publisher(Int32,'/send_int32to_pico', 10)
        timer_period = 0.0001
        self.counter = 0
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.received_data = ""
        

    def timer_callback(self):
        send_int32 = Int32()

        @sio.on('string')
        def on_string(data):
            self.received_data = data
            print(f'Received string: {data}')

        if self.received_data == "right":
            send_int32.data = 1
            
        elif self.received_data == "left":
            send_int32.data = 4

        elif self.received_data == 'right_magnet_engage':
            send_int32.data = 3
        elif self.received_data == 'right_magnet_disengage':
            send_int32.data = 2
        elif self.received_data == 'left_magnet_engage':
            send_int32.data = 6
        else:
            send_int32.data = 0

        self.publisher.publish(send_int32)



def main(args=None):
    rclpy.init(args=args)

    velocity_publisher = GetFromServerSendToPico()
    
    rclpy.spin(velocity_publisher)
    velocity_publisher.destroy_node()
    velocity_publisher.get_logger().info(f"Shutting down the node")
    rclpy.shutdown()


if __name__ == '__main__':
    main()