# wait for rami to finalise the server host and commands from the server


import string
import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32

import socketio

sio = socketio.Client()

home = "Home"

remoteControl = "Remote Control"
rosScripts = "ROS Scripts"

leftDriveEnable = "Left Drive Enable"
rightDriveEnable = "Right Drive Enable"
moveRobot = "Move Robot to Angle"
engageMagnet = "Engage Magnet"
disengageMagnet = "Disengage Magnet"
script1 = "Script 1"
script2 = "Script 2"


htmlChoose  = "choose.html"
htmlRemote = "remoteControl.html"
htmlROS = "rosScripts.html"


@sio.on('connect')
def on_connect():
    print('Connected to the server.')

# sio.connect('http://10.250.145.31:5000')
# sio.connect('http://10.250.174.214:5000')
# sio.connect('http://192.168.0.100:5000/')
# sio.connect('http://10.250.145.31:5000')
sio.connect('http://192.168.0.103:5000')





class Int32Publisher(Node):

    def __init__(self):
        super().__init__('int32_publisher')
        self.publisher = self.create_publisher(Int32,'/send_int32to_pico', 10)
        timer_period = 0.0001
        self.counter = 0
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.received_data = "none"
        self.driveBase=''
        self.counter = 0
        

    def timer_callback(self):
        send_int32 = Int32()
        leftDrive = 'Left Drive'
        rightDrive = 'Right Drive'

        @sio.on('message')
        def on_string(data):
            self.received_data = data
            print(f'Received string: {data}')

        if self.received_data == leftDriveEnable:
            self.driveBase = leftDrive

        elif self.received_data == rightDriveEnable:
            self.driveBase = rightDrive

        if self.driveBase == leftDrive and self.received_data == moveRobot:
            # send_int32.data = f'{leftDrive} and {constants.moveRobot}'
            send_int32.data = 1

        elif self.driveBase == leftDrive and self.received_data == engageMagnet:
            # send_int32.data = f'{leftDrive} and {constants.engageMagnet}'
            send_int32.data = 2

        elif self.driveBase == leftDrive and self.received_data == disengageMagnet:
            # send_int32.data = f'{leftDrive} and {constants.disengageMagnet}'
            send_int32.data = 3

        elif self.driveBase == rightDrive and self.received_data == moveRobot:
            # send_int32.data = f'{rightDrive} and {constants.moveRobot}'
            send_int32.data = 4

        elif self.driveBase == rightDrive and self.received_data == engageMagnet:
            # send_int32.data = f'{rightDrive} and {constants.engageMagnet}'
            send_int32.data = 5

        elif self.driveBase == rightDrive and self.received_data == disengageMagnet:
            # send_int32.data = f'{rightDrive} and {constants.disengageMagnet}'
            send_int32.data = 6

        if self.counter >= 100:
            send_int32.data = 100
            self.received_data = 100
            self.counter = 0

        if send_int32.data <= 6:
            self.counter = self.counter + 1

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
