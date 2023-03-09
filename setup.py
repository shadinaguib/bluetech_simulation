import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'bluetech_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'description'), glob('description/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*'))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bluetech',
    maintainer_email='bluetech@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apply_effort = bluetech_simulation.apply_effort:main',
            'bluetech_localization_over_wifi = bluetech_simulation.bluetech_localization_over_wifi:main',
            'bluetech_localization = bluetech_simulation.bluetech_localization:main',
            'forward_effort = bluetech_simulation.forward_effort:main',
            'avoidance_controller = bluetech_simulation.avoidance_controller:main',
            'get_data_from_rami = bluetech_simulation.get_data_from_rami:main',
            'jetson_client = bluetech_simulation.jetson_client:main'
            'joint_position_client = bluetech_simulation.joint_position_client:main',
            'link_wrench_client = bluetech_simulation.link_wrench_client:main',
            'move_robot = bluetech_simulation.move_robot:main',
            'pico_control = bluetech_simulation.pico_control:main',
            'keyboard_teleop = bluetech_simulation.keyboard_teleop:main',
            'pico_from_teleop = bluetech_simulation.pico_from_teleop:main',
            'position_estimate = bluetech_simulation.position_estimate:main',
            'publish_from_encoder = bluetech_simulation.publish_from_encoder:main',
            'publish_digital_indicator = bluetech_simulation.publish_digital_indicator:main', 
            'publish_from_joint_state = bluetech_simulation.publish_from_joint_state:main',
            'robot_localization = bluetech_simulation.robot_localization:main',
            'send_effort = bluetech_simulation.send_effort:main',
            'send_int_to_pico = bluetech_simulation.send_int_to_pico:main',
            'send_two_vacuum_forces = bluetech_simulation.send_two_vacuum_forces:main',
        ],
    },
)
