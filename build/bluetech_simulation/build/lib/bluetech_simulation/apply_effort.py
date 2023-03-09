# ApplyLinkWrench (Service) will apply a "wrench force" to the specified link in Gazebo
# (name of the link here must match the name in the URDF)
# You can specify the origin of the force vector relative to the origin of the 
# link as described in the urdf. 

# This was initially used to model the adhesion force in the Gazebo simulation. 
# The link force is implemented as a service that applies a specified force on the right
# link in the negative Z direction
# More info here http://docs.ros.org/en/jade/api/gazebo_msgs/html/srv/ApplyBodyWrench.html

# usage
# make sure the robot is spawned in Gazebo. Make sure you specify the force applied on
# the link as an argument, in the example below, the force applied is 10

# ros2 run bluetech_simulation apply_effort 10

import sys
import time
from gazebo_msgs.srv import ApplyLinkWrench
import rclpy
from rclpy.node import Node


class LinkWrenchClient(Node):

    def __init__(self):
        super().__init__('apply_effort')
        self.cli = self.create_client(ApplyLinkWrench, '/apply_link_wrench')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ApplyLinkWrench.Request()

    def send_request(self, effort: float):

        # here you can change the link on which you want to apply the wrench force
        self.req.link_name = "bluetech_robot::"+"right"+"_wheel_link"
        
        
        self.req.reference_frame = "bluetech_robot::right_wheel_link"

        # setting origin of the wrench force to the origin of the link
        self.req.reference_point.x = 0.0
        self.req.reference_point.y = 0.0
        self.req.reference_point.z = 0.0

        self.req.start_time.sec = 0
        self.req.start_time.nanosec = 0

        self.req.duration.sec = -1
        self.req.duration.nanosec = 0
        
        # FORCE
        self.req.wrench.force.z = -effort
        self.req.wrench.force.x = 0.0
        self.req.wrench.force.y = 0.0

        # TORQUE
        self.req.wrench.torque.x = 0.0
        self.req.wrench.torque.y = 0.0
        self.req.wrench.torque.z = 0.0


        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    link_client = LinkWrenchClient()
    rate = link_client.create_rate(2)

    response = link_client.send_request(effort=float(sys.argv[1]))
    
    link_client.get_logger().info(f'Link Wrench of: {sys.argv[1]} appiled to right link')

    time.sleep(1)

    link_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()