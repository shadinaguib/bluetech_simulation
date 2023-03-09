# bluetech simulation


<!-- TODO add the topic/services/actions used -->
# Overview
Contains information about the several ROS2 nodes used to control the bluetech robobt. Run any of these nodes to control the robot after using the ```bluetech_sim.launch.py``` file. 
```
ros2 launch bluetech_simulation bluetech_sim.kaunch.py
```
# Nodes

### apply effort
```
ros2 run bluetech_simulation apply_effort 0.5
```
Applies a specified torque to the right wheel **link**.

### forward_effort
```
ros2 run bluetech_simulation feed_forward 0.5
```
Applies a torque to the left and right wheel joints and toggles friction to move the robot forward. 

### joint_position_client
```
ros2 run bluetech_simulation joint_client right 3.14
```
Moves the specified joint (right or left) to a specific position using the ros2_control ```FollowJointTrajectory``` controller

### link_wrench client
```
ros2 run bluetech_simulation link_client right 50
```
Applies a specified force on the link in the negative z direction of the world frame.  

### send_effort
```
ros2 run bluetech_simulation send_effort 0.5
```
Applies a specified effort on the right wheel joint.


### send_velocity
```
ros2 run bluetech_simulation send_velocity 5
```
Applies a specified velocity on the right wheel joint.


### send_two_vacuum_forces
```
ros2 run bluetech_simulation send_vacuum
```
Turns on the vacuum force on the 2 wheel joints.