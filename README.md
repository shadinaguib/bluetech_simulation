# bluetech_simulation

Contains the ROS2 package for bluetech navigation simulated in Gazebo.  

# Installation

## Requirements
Make sure you have the following installed/running on your machine.  
- git must be working correctly. Since we will be pushing to private remote repositories, make sure you have git setup correctly, use the Git Credential Manager to make sure git commands work correctly without permission errors. 
- [Ubuntu 22.04](https://releases.ubuntu.com/22.04/)
- [ROS2 - Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) (do NOT build from source, the debian packages are what is required for proper installation. In general, for any ROS related packages, do not build from source unless absolutely necessary)
- [colcon](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html#install-colcon)


## Creating a ROS2 workspace
Create a ROS2 workspace called ```ros2_ws``` (in your home directory) - instructions to do so can be found [here](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html). Once you have made a workspace you will have an ```src``` folder, enter this folder.  
```
cd ~/ros2_ws/src
```

## Dependencies
Before running any files, make sure you have to following dependencies installed.  
- [ros2_control](https://control.ros.org/master/doc/getting_started/getting_started.html)
- [custom_gazebo_pkgs](https://github.com/bluetechio/custom_gazebo_plugins)
- [classic gazebo 11](https://classic.gazebosim.org/tutorials?tut=install_ubuntu&cat=install)

## Cloning the package
Inside the ```src``` folder, clone the repository. This is a ROS2 package and has to be built after cloning. 

```
git clone <link_to_repo>
```

## Buidling
You need to source the ROS installation everytime you open a new terminal using the following command.  
```
source /opt/ros/humble/setup.bash
```

If you do not want to source the installation every time add this line to the end of your ```.bashrc``` file in your home directory using
```
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```
It’s good practice to run rosdep in the root of your workspace ```ros2_ws``` to check for missing dependencies before building:
```
rosdep install -i --from-path src --rosdistro humble -y
```

After cloning the repository go to the root of the workspace you just created and build the package using ```colcon```. Any ROS2 package needs to be built by going into the root of the workspace. Use the ```--packages-select``` argument to build only the needed packages.    
```
cd ~/ros2_ws
colcon build --packages-select bluetech_simulation
```

Source the install ```setup.bash``` from the ```ros2_ws``` you just created.  
```
source ~/ros2_ws/install/setup.bash
```
That's it! You have successfully built the ```bluetech_simulation``` package!


# Usage
There are several ROS nodes and launch files in the package which can be used to spawn robots, add forces, toggle parameters.  


To understand what each function/node does please refer to the following [readme](bluetech_simulation/README.md). 

## Structure
The package has the following sub-directories
- ```bluetech_simulation``` contains all the ROS2 nodes
- ```config``` contains all the ```.yaml``` files
- ```description``` contains all the ```urdf.xacro``` files
- ```launch``` contains all the ```launch.py``` files
- ```worlds``` contains the ```sdf``` files that describe the gazebo worlds. 

# Contributing
Anytime a new change is made which involves a new dependency, please make sure to update the ```package.xml``` file so that the package builds correctly.  
Ideally when developing, create your own branch with an appropriate name.  

## Testing
To check if your package has been built and dependencies have been installed correctly run the code for basic robot walking.  
Make sure you have sourced the ```ros2_ws``` and the ROS2 installation.  In the root of your ```ros2_ws``` launch the world file using:
```
cd ~/ros2_ws
ros2 launch bluetech_simulation bluetech_sim.launch.py
```
This should open a gazebo window and RViz to visualize the static and dynamic transforms. In another terminal check to see if the effort controllers have been spawned correctly. 
```
ros2 control list_hardware_interfaces
```
You should see the following output:
```
command interfaces
	left_wheel_joint/effort [available] [claimed]
	right_wheel_joint/effort [available] [claimed]
state interfaces
	left_wheel_joint/effort
	right_wheel_joint/effort
```

In another terminal check for the following ROS2 topics.
```
ros2 topic list
```
or use the ```-t``` argument to check the type of message on the topic.
```
ros2 topic list -t
```
There will be many topics listed, make sure you have the following topics listed (this output is without the ```-t``` argument):
```
/joint_states
/left/friction
/left/friction_status
/left_wheel_effort_controller/commands
/left_wheel_effort_controller/transition_event
/right/friction
/right/friction_status
/right_wheel_effort_controller/commands
/right_wheel_effort_controller/transition_event
/robot_description
/rosout
/tf
/tf_static
```

In another terminal run the ```feed_forward``` node to move the robot using with a specific effort for example:
```
ros2 run bluetech_simulation feed_forward 1
```

# Problems
## Gazebo
Often the gazebo server will crash unexpectedly and needs to be restarted. Make sure to kill the gazebo client and server completely before restarting gazebo using:
```
killall -9 gzserver
killall -9 gzclient
```

## Gazebo + ros2_control
While controlling joints using ros2_control make sure to use ```effort_controllers``` and not ```position_controllers``` or ```trajectory_controller```, they force gazebo to turn the joints to a specified position and may result in unexpected or unrealistic behaviour.  

Ideally use a custom PID controller with the ```effort_controller``` and command a setpoint in position to move the joint to. This is under development [here](https://github.com/ros-controls/ros2_controllers/tree/effort_group_position_controller/effort_controllers)
