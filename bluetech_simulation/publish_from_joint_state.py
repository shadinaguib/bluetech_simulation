from math import pi
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float64, String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point32



class PublishEncoders(Node):

    def __init__(self):
        super().__init__('publish_from_joint_state')

        self.get_command = self.create_subscription(String, "/cmd_vel", self.set_encoders, 1)
        self.get_joint_states = self.create_subscription(JointState, 'joint_states', self.get_encoder_values_from_simulation, 10)
        
        self.publisher_right = self.create_publisher(Float64,'/right_encoder', 10)
        self.publisher_left = self.create_publisher(Float64,'/left_encoder', 10)

        self.set_motors = [0,0]

    def set_encoders(self, msg: String):
        if msg.data == 'l':
            # move the left motor
            self.set_motors = [-1,0]
        elif msg.data == 'r':
            # move the right motor only
            self.set_motors = [0,1]
        else:
            self.set_motors = [0,0]
    
     
    def get_encoder_values_from_simulation(self, joint_states: JointState):
        
        encoder_ticks = Float64()

        right_joint = self.set_motors[1]*joint_states.position[0]
        encoder_ticks.data = right_joint
        self.publisher_right.publish(encoder_ticks)

        left_joint = self.set_motors[0]*joint_states.position[1]
        encoder_ticks.data = left_joint
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