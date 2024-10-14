#!/usr/bin/python3
# -*- coding: utf-8 -*-
from launch_ros.actions import Node
from launch import LaunchDescription


# this is the function launch  system will look for


def generate_launch_description():


    spawn_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    spawn_diff_driver_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_base_controller"],
        remappings=[("/cmd_vel_unstamped", "/cmd_vel")],
        output="screen",
    )

    spawn_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
        output="screen",
    )

    spawn_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
        output="screen",
    )
    

    # create and return launch description object
    return LaunchDescription(
        [
            spawn_controller,
            spawn_diff_driver_controller,
            spawn_arm_controller,
            spawn_gripper_controller
        ]
    )