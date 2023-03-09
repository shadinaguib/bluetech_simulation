from math import pi
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float64, String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point32



class PublishEncoders(Node):

    def __init__(self):
        super().__init__('publish_encoder')

        # self.get_command = self.create_subscription(String, "/cmd_vel", self.set_encoders, 1)
        # self.get_joint_states = self.create_subscription(JointState, 'joint_states', self.get_encoder_values_from_simulation, 10)
        self.get_encoder_clicks = self.create_subscription(Point32, "encoder", self.get_encoder_values_from_pico, 10)

        # self.get_encoder = self.create_subscription(Point32, '/encoder', self.split_topic, 1)
        
        self.get_int = self.create_subscription(Int32, "/send_int32to_pico", self.get_int32_set_encoders, 1)
        
        self.publisher_right = self.create_publisher(Float64,'/right_encoder', 10)
        self.publisher_left = self.create_publisher(Float64,'/left_encoder', 10)
        timer_period = 0.001
        self.counter = 0

        self.prev_right = 0.0
        self.prev_left = 0.0

        self.set_motors = [0,0]
        # self.timer = self.create_timer(timer_period, self.timer_callback) 


    def get_int32_set_encoders(self, int_msg: Int32):
        if int_msg.data == 4:
            # move the left motor
            self.set_motors = [-1,0]
        elif int_msg.data == 5:
            # move the right motor only
            self.set_motors = [0,1]
        else:
            self.set_motors = [0,0]


    def set_encoders(self, msg: String):
        if msg.data == 'a':
            # move the left motor
            self.set_motors = [-1,0]
        elif msg.data == 'c':
            # move the right motor only
            self.set_motors = [0,1]
        else:
            self.set_motors = [0,0]
    
    def get_encoder_values_from_pico(self, encoder_values: Point32):
        encoder_ticks = Float64()

        right_joint = self.set_motors[1]*encoder_values.y
        encoder_ticks.data = right_joint

        # publish over the network or directly to the topic
        self.publisher_right.publish(encoder_ticks)

        left_joint = self.set_motors[0]*encoder_values.x
        encoder_ticks.data = left_joint

        # data = f"{left_joint},{right_joint}"
        # publish over the network or directly to the topic
        self.publisher_left.publish(encoder_ticks)
        # self.conn.send(data.encode())  # send data to the client
        

    def get_encoder_values_from_simulation(self, joint_states: JointState):
        
        encoder_ticks = Float64()

        right_joint = self.set_motors[1]*joint_states.position[0]
        encoder_ticks.data = right_joint
        self.publisher_right.publish(encoder_ticks)

        left_joint = self.set_motors[0]*joint_states.position[1]
        encoder_ticks.data = left_joint
        self.publisher_left.publish(encoder_ticks)

    
    def split_topic(self, msg: Point32):

        self.get_logger().info(f"Left: {msg.x} and Right: {msg.y}")


    def timer_callback(self):
        self.counter += 1
        encoder_ticks = Float64()

        if self.counter < 1000:
            encoder_ticks.data = pi/10000 * self.counter
            self.publisher_right.publish(encoder_ticks)
            encoder_ticks.data = 0.0
            self.publisher_left.publish(encoder_ticks)
        else:
            encoder_ticks.data = 0.0
            self.publisher_right.publish(encoder_ticks)
            encoder_ticks.data = pi/100000 * self.counter
            self.publisher_left.publish(encoder_ticks)


def main(args=None):
    rclpy.init(args=args)

    effort_publisher = PublishEncoders()
    
    rclpy.spin(effort_publisher)
    effort_publisher.destroy_node()
    effort_publisher.get_logger().info(f"Shutting down the node")
    rclpy.shutdown()

if __name__ == '__main__':
    main()