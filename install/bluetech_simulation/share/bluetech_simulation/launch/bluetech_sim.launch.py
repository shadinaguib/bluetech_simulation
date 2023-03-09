import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node, SetParameter
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'bluetech_simulation'
    file_subpath = 'description/robot.urdf.xacro'

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    with open(xacro_file, 'r') as infp:
        robot_desc = infp.read()

    # Configure the node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description_raw}],
        arguments=[robot_description_raw]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )

    spawn_robot_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'robot'],
                    output='screen')


    rviz = Node(
        package='rviz2',
        executable='rviz2',
    )


    localization = Node(
        package="bluetech_simulation",
        executable="bluetech_localization"
    )

    publish_from_encoder = Node(
        package="bluetech_simulation",
        executable="publish_from_encoder"
    )

    pico_from_teleop = Node(
        package="bluetech_simulation",
        executable="pico_from_teleop"
    )

    # run this launch file

    # run the bluetech_localization node
    
    # keyboard_teleop node (use the keyboard to see where the robot estimates its position)
    # OR
    # do NOT forget to start the webserver
    # run the send_int_to_pico node (this will connect to the webserver and run based on those commands)
    
    return LaunchDescription([        
        SetParameter(name='use_sim_time', value=True),
        #gazebo,
        #spawn_robot_entity,
        rviz,
        robot_state_publisher,
        publish_from_encoder,
        pico_from_teleop,
    ])


