# experimental node to model adhesion

import string
import sys
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class SendTwoVacuumForce(Node):

    def __init__(self):
        super().__init__('send_two_vacuum_forces')
        self.one = self.create_client(SetBool, '/one/switch')
        self.two = self.create_client(SetBool, '/two/switch')

        while not (self.one.wait_for_service(timeout_sec=1.0) or \
                        self.two.wait_for_service(timeout_sec=1.0)):
            self.get_logger().info('service not available, waiting again...')
        
        self.req_one = SetBool.Request()
        self.req_two = SetBool.Request()

    def send_request(self, one: bool, two: bool):
        
        self.req_one.data = one
        self.req_two.data = two

        self.future_one = self.one.call_async(self.req_one)
        self.future_two = self.two.call_async(self.req_two)
        rclpy.spin_until_future_complete(self, self.future_one)
        rclpy.spin_until_future_complete(self, self.future_two)

        return [self.future_one.result(), self.future_two.result()]


def main(args=None):
    rclpy.init(args=args)

    vacuum_client = SendTwoVacuumForce()
    rate = vacuum_client.create_rate(2)

    one: bool = False
    two: bool = False

    if sys.argv[1] == "true":
        one = True
    if sys.argv[2] == "true":
        two = True

    response = vacuum_client.send_request(one=one, two=two)
    
    
    vacuum_client.get_logger().info(
        f'Cube one is: {str(one)}; Cube two is: {str(two)}')


    vacuum_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()