import string
import sys
import time
from gazebo_msgs.srv import ApplyLinkWrench
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class LinkWrenchClient(Node):

    def __init__(self):
        super().__init__('link_wrench_client')
        self.cli = self.create_client(ApplyLinkWrench, '/apply_link_wrench')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ApplyLinkWrench.Request()

    def send_request(self, magnetic_force: float, link: string):
        self.req.link_name = "bluetech_robot::"+link+"_wheel_link"
        # self.req.link_name = "ground_plane::link"

        
        
        self.req.reference_frame = "bluetech_robot::"+link+"_wheel_link"
        self.req.reference_point.x = 0.0
        self.req.reference_point.y = 0.0
        self.req.reference_point.z = 0.0

        self.req.start_time.sec = 0
        self.req.start_time.nanosec = 0

        self.req.duration.sec = -1
        self.req.duration.nanosec = 0
        
        # FORCE
        self.req.wrench.force.z = -magnetic_force
        self.req.wrench.force.x = 0.0
        self.req.wrench.force.y = 0.0

        # TORQUE
        self.req.wrench.torque.x = 0.0
        self.req.wrench.torque.y = 0.0
        self.req.wrench.torque.z = 0.0

        if link == "chassis":
            self.req.link_name = "bluetech_robot::chassis_link"
            self.req.wrench.force.z = 0.0
            self.req.wrench.force.x = 0.0
            self.req.wrench.force.y = magnetic_force
            self.req.reference_point.z = -0.2

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    link_client = LinkWrenchClient()
    rate = link_client.create_rate(2)

    response = link_client.send_request(float(sys.argv[1]), str(sys.argv[2]))
    
    
    link_client.get_logger().info(
        f'Force of: {int(sys.argv[1])} appiled to {str(sys.argv[2])} link')

    time.sleep(1)
    if sys.argv[2] == "chassis":
        response = link_client.send_request(0.0, str(sys.argv[2]))
        link_client.get_logger().info(
            f'Force of: {0} appiled to {str(sys.argv[2])} link')

    link_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()