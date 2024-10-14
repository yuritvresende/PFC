'''
Author: David Valencia
Date: 15 / 10 /2021

Describer:  
			
			
			This scrip spawn the robot in gazebo and LOAD and START a basic joint_trajectory_controller
			
			The info and configuration of the controller can be found in the config folder:
			
			/src/my_doosan_pkg/config/simple_controller.yaml

			
			Just for testing purposes inside this packages only
			
			- Robot model m1013 color white
			- Robot model a0912 color blue
'''

import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():


    spawn_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    spawn_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
        output="screen",
    )
    

    # create and return launch description object
    return LaunchDescription(
        [
            spawn_controller,
            spawn_arm_controller
        ]
    )