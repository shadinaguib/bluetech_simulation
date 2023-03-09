# Updates the position of the robot based on the current encoder readings. 
# 
# This handles the robot movemet correctly by moving only one motor at 
# a time.
#
#     publisher                      topics                      subscriber        
# publish_encoder ---> /right_encoder and /left_encoder ---> bluetech_localization
#
# Sequence of events:
#
# 1. Update time information (current time, time elapsed)
# 
# 2. Update the robot theta here the robot theta is the total angle moved by both the motors 
#    theta = right_radians - left_radians (-ve sign to handle orientation)
# 
# 3. To get the current position of the chassis_link. Check which motor is rotating
#     3.1 get the current position of the rotating motor wrt the odom frame
#     3.2 rotate the chassis_link about the origin (origin: rotating frame) by the angle moved by the motor
#     3.3 shift the origin of the chassis_link to the position of the current rotating motor
# 
# 4. update the current velocities
# 
# 5. publish tf2 and odom data



import rclpy
from rclpy.node import Node
from math import sin, cos, pi
import numpy as np
import sys
from geometry_msgs.msg import Quaternion, TransformStamped, Point32

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


NS_TO_SEC = 1000000000

def quaternion_from_euler(ai: float, aj: float, ak: float) -> Quaternion:
    """
    This function converts euler angles (roations about xyz) to a quartenion and
    returns a Quartenion object.

    Inputs:
    ai: roll
    aj: pitch
    ak: yaw

    Returns:
    quartenion: Quartenion object (can be fed into the Odometry topic directly)
    """
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = cos(ai)
    si = sin(ai)
    cj = cos(aj)
    sj = sin(aj)
    ck = cos(ak)
    sk = sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    quartenion = Quaternion()
    quartenion.x = cj*sc - sj*cs
    quartenion.y = cj*ss + sj*cc
    quartenion.z = cj*cs - sj*sc
    quartenion.w = cj*cc + sj*ss


    return quartenion

def get_rotated_point(point: np.array, theta: float):
    """
    This function rotates a point about the origin by the angle specified

    Input:
    point: np.array (1x2)
    theta: anticlockwise angle by which the point is to be rotated in radians (of course!)

    Returns
    rotated point: np.array (1x2)
    """
    rot_matrix = np.array([[cos(theta), sin(theta)],
                           [-sin(theta), cos(theta)]])

    return (rot_matrix @ point.T).T

class BluetechLocalization(Node):

    def __init__(self, gazebo_simulation: str="False"):
        super().__init__("bluetech_localization")
        self.nodename = "bluetech_localization"
        self.get_logger().info(f"{self.nodename} started")

        self.gazebo = gazebo_simulation

        # parameters
        self.rate_hz = self.declare_parameter("rate_hz", 10.0).value
        self.base_frame_id = self.declare_parameter('base_frame_id', 'base_footprint').value
        self.odom_frame_id = self.declare_parameter('odom_frame_id', 'odom').value
        self.chassis_frame_id = self.declare_parameter('chassis_frame_id', 'chassis_link').value
        self.left_wheel_frame_id = self.declare_parameter('left_wheel_frame', 'left_wheel_link').value
        self.right_wheel_frame_id = self.declare_parameter('right_wheel_frame', 'right_wheel_link').value
              
        # set left and right encoder ticks
        self.left_radians = 0.0
        self.right_radians = 0.0

        
        # initialise the current and previous x, y and theta values
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.x = 0.0  
        self.y = 0.0
        self.theta = 0.0
        self.prev_theta = 0.0

        # linear velocity of the robot
        self.dx = 0.0
        self.dy = 0.0
        # angular velocity of the robot along the z axis
        self.dr = 0.0

        # motor bools
        self.left_motor_rotating = False
        self.right_motor_rotating = False

        # get the previous time (initialised as the current time)
        self.then = self.get_clock().now()

        # tf transform stuff here
        self.tf_buffer = Buffer() # initialise the buffer
        # initialise a listener, this will allow us to "lookup" transforms between 2 frames at any point of time
        self.tf_listener = TransformListener(self.tf_buffer, self) 

        
        # subscriptions and publishers

        # subscribe to the right and left encoder topics (publish_encoder node is publishing to these topics)
        self.create_subscription(Float64, "left_encoder", self.read_left_encoder, 10)
        self.create_subscription(Float64, "right_encoder", self.read_right_encoder, 10)
        
        # initialise the odom publisher to publish all odom information
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        
        # setup dynamic transforms
        # the following transforms establish the transforms that define the robot. 
        # chassis_link to right_wheel_link
        # chassis to left_wheel_link
        # odom to base_footprint


        # you might suggest that right_wheel_link and left_wheel_link are static as well wrt 
        # chassis_link, but think again, they are rotating about the z axis as they represent the
        # position and orientation of the motor shafts wrt the chassis_link. Hence, the are sent as dynamic transforms
        
        self.odom_broadcaster = TransformBroadcaster(self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.chassis_to_left = TransformStamped()
        self.chassis_to_left.header.frame_id = self.chassis_frame_id
        self.chassis_to_left.child_frame_id = self.left_wheel_frame_id
        self.chassis_to_left.transform.translation.x = -0.75
        self.chassis_to_left.transform.translation.z = -0.375
        
        self.chassis_to_right = TransformStamped()
        self.chassis_to_right.header.frame_id = self.chassis_frame_id
        self.chassis_to_right.child_frame_id = self.right_wheel_frame_id
        self.chassis_to_right.transform.translation.x = 0.75
        self.chassis_to_right.transform.translation.z = -0.375

        self.odom_to_base_footprint = TransformStamped()
        self.odom_to_base_footprint.header.frame_id = self.odom_frame_id
        self.odom_to_base_footprint.child_frame_id = self.base_frame_id


        # base_footprint to chassis_link is a static transform (does not change ove time)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame_id
        t.child_frame_id = self.chassis_frame_id

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.5
        quat = quaternion_from_euler(0.0, 0.0, 0.0)
        t.transform.rotation = quat

        self.tf_static_broadcaster.sendTransform(t)

        # setup odometry information
        self.odom = Odometry()
        self.odom.header.frame_id = self.odom_frame_id
        self.odom.child_frame_id = self.base_frame_id

        # start a timer that will call the update function at the desired frequency
        self.create_timer(1/self.rate_hz, self.update)


    def update(self):
        """
        Updates the position of the robot based on the current encoder readings. 
        This handles the robot movemet correctly by moving only one motor at 
        a time.
        """

        # update the time information
        now = self.get_clock().now()
        elapsed = now - self.then
        self.then = now
        elapsed = elapsed.nanoseconds / NS_TO_SEC

        # update the robot theta/orientation
        self.theta = self.right_radians - self.left_radians

        # try except block is used as the lookup_transform() function can give errors
        # if there is a mismatch in time or if time is not synchronized.
        # the try except will catch the exception but continue the 
        try:
            # check which motor is moving
            if self.left_motor_rotating:

                # get the current position of the rotating frame
                left_wrt_odom = self.tf_listener.buffer.lookup_transform("odom", "left_wheel_link", rclpy.time.Time())
                
                # initialize the chassis frame such that the rotating frame is the origin
                base_in_left = np.array([0.75, 0.0])
                # rotate the chassis by the angle moved by the motor
                base_rotated = get_rotated_point(base_in_left, self.theta)

                # shift the origin of the chassis frame to the current position of the rotating frame
                rotated_base_in_odom = base_rotated + np.array([left_wrt_odom.transform.translation.x, left_wrt_odom.transform.translation.y])

                # update the cjassis frame positions
                self.x = rotated_base_in_odom[0]
                self.y = rotated_base_in_odom[1]

                # update the rotating frame with new rotation
                self.chassis_to_left.transform.rotation = quaternion_from_euler(0.0,0.0,self.theta)

            if self.right_motor_rotating:
                right_wrt_odom = self.tf_listener.buffer.lookup_transform("odom", "right_wheel_link", rclpy.time.Time())               
                base_in_right = np.array([-0.75, 0.0])
                base_rotated = get_rotated_point(base_in_right, self.theta)
                rotated_base_in_odom = base_rotated + np.array([right_wrt_odom.transform.translation.x, right_wrt_odom.transform.translation.y])
                self.x = rotated_base_in_odom[0]
                self.y = rotated_base_in_odom[1]
                self.chassis_to_right.transform.rotation = quaternion_from_euler(0.0,0.0,self.theta)
        except:
            self.get_logger().warn(f"Skipping frames due to time incompatibility")


        # velocity stuff
        if elapsed == 0:
            self.dr = 0.0
            self.dx = 0.0
            self.dy = 0.0
        else:
            self.dx = (self.x - self.prev_x) / elapsed
            self.dy = (self.y - self.prev_y) / elapsed
            self.dr = (self.theta-self.prev_theta) / elapsed
            
            self.prev_x = self.x
            self.prev_y = self.y
            self.prev_theta = self.theta

        # set the timestamps        
        self.odom_to_base_footprint.header.stamp = now.to_msg()
        self.chassis_to_right.header.stamp = now.to_msg()
        self.chassis_to_left.header.stamp = now.to_msg()
        self.odom.header.stamp = now.to_msg()


        # set the odom information        
        # set the tf2 frames
        quaternion = quaternion_from_euler(0.0, 0.0, -self.theta)
        self.odom_to_base_footprint.transform.translation.x = self.x
        self.odom_to_base_footprint.transform.translation.y = self.y
        self.odom_to_base_footprint.transform.rotation = quaternion

        # set the odometry data
        self.odom.pose.pose.position.x = self.x
        self.odom.pose.pose.position.y = self.y
        self.odom.pose.pose.position.z = 0.0
        self.odom.pose.pose.orientation = quaternion
        self.odom.twist.twist.linear.x = self.dx
        self.odom.twist.twist.linear.y = self.dy
        self.odom.twist.twist.angular.z = self.dr

        # publish the tf2 and odometry data
        self.odom_broadcaster.sendTransform(self.odom_to_base_footprint)
        self.odom_broadcaster.sendTransform(self.chassis_to_left)
        self.odom_broadcaster.sendTransform(self.chassis_to_right)
        self.odom_pub.publish(self.odom)


    # TODO: update with correct ratios
    def read_left_encoder(self, msg: Float64):
        # if the /left_encoder topic sends anything other than a 0, set left_motor_rotating to TRUE
        if msg.data != 0.0:
            self.right_motor_rotating = False
            self.left_motor_rotating = True
            if self.gazebo == "True":
                self.left_radians = msg.data
            else:
                self.left_radians = (msg.data / 17.5) * pi / 180
                

    def read_right_encoder(self, msg: Float64):
        # if the /right_encoder topic sends anything other than a 0, set right_motor_rotating to TRUE
        if msg.data != 0.0:
            self.right_motor_rotating = True
            self.left_motor_rotating = False
            if self.gazebo == "True":
                self.right_radians = msg.data
            else:
                self.right_radians = (msg.data / 17.5) * pi / 180






def main(args=None):
    rclpy.init(args=args)
    try:
        if len(sys.argv) > 1:
            diff_tf = BluetechLocalization(sys.argv[1])
        else:
            diff_tf = BluetechLocalization()

        rclpy.spin(diff_tf)
    except rclpy.exceptions.ROSInterruptException:
        pass

    diff_tf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()