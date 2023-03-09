import sys
import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class JointPositionClient(Node):

    def __init__(self):
        super().__init__('joint_position_client')

    def send_goal(self, joint: str, angle: float):
        goal_msg = FollowJointTrajectory.Goal()

        # Fill in data for trajectory
        if joint == "right":
            joint_names = ["right_wheel_joint"]
            self._action_client = ActionClient(self, FollowJointTrajectory, '/right_wheel_trajectory_controller/follow_joint_trajectory')
        else:
            self._action_client = ActionClient(self, FollowJointTrajectory, '/left_wheel_trajectory_controller/follow_joint_trajectory')
            joint_names = ["left_wheel_joint"]

        points = []
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0]

        point2 = JointTrajectoryPoint()
        point2.time_from_start = Duration(seconds=1, nanoseconds=0).to_msg()
        point2.positions = [angle]

        point3 = JointTrajectoryPoint()
        point3.time_from_start = Duration(seconds=2, nanoseconds=0).to_msg()
        point3.positions = [0.0]

        points.append(point1)
        points.append(point2)
        points.append(point3)

        goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = points


        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: '+str(result))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        #self.get_logger().info('Received feedback:'+str(feedback))

    

def main(args=None):
    
    rclpy.init()

    action_client = JointPositionClient()

    angle = float(sys.argv[2])
    joint = sys.argv[1]
    future = action_client.send_goal(joint, angle)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()