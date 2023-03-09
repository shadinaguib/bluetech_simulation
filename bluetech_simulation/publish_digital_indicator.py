from math import pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile
from std_msgs.msg import Int32, Float64, String, Bool
from sensor_msgs.msg import JointState, LaserScan
from geometry_msgs.msg import Point32



class PublishIndicator(Node):

    def __init__(self):
        super().__init__('publish_digital_indicator')
        
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        
        self.get_int = self.create_subscription(LaserScan, "/scan", self.compute_weld_line_status, qos_profile=qos_profile)
        
        self.publisher_digital_indicator = self.create_publisher(Bool,'/weld_line_detected', 10)

        self.looking_for_weld_line = True

        self.weld_line_counter = 0

        self.get_logger().info("Started publish_digital_indicator node")
        
    def compute_weld_line_status(self, int_msg):
        """From the sensor value, determine if a weld line was detected or not.

        Args:
            int_msg (LaserScan): displacement sensor values from the middle of the chassis.
        """
        value = Float64()
        value.data = int_msg.intensities[0]
        if self.looking_for_weld_line:
            if value.data == 1.0:
                self.weld_line_counter += 1
                if self.weld_line_counter % 2: #went over weld line but did not come back
                    self.publisher_digital_indicator.publish(Bool(data=True)) #new weld_line detected
                else: # this means we got back over the weld line / over another weld line if there is a corner 
                    self.publisher_digital_indicator.publish(Bool(data=False))

                self.looking_for_weld_line = False

        else: # this means we just completely go over weld line 
            if value.data == 0.0:
                self.looking_for_weld_line = True

def main(args=None):
    rclpy.init(args=args)

    effort_publisher = PublishIndicator()
    
    rclpy.spin(effort_publisher)
    effort_publisher.destroy_node()
    effort_publisher.get_logger().info(f"Shutting down the node")
    rclpy.shutdown()

if __name__ == '__main__':
    main()