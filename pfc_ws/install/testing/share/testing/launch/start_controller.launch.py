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
        parameters=[{'use_sim_time': True}],  # Enable simulation time
    )

    spawn_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
        output="screen",
        parameters=[{'use_sim_time': True}],  # Enable simulation time
    )

    spawn_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
        output="screen",
        parameters=[{'use_sim_time': True}],
    )
    

    # create and return launch description object
    return LaunchDescription(
        [
            spawn_controller,
            spawn_arm_controller,
            spawn_gripper_controller,
            
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'publish_rate': 200.0}],  # Set to 200 Hz
        ),
        
        # Launch Gazebo with real-time update rate set
        #Node(
        #    package='gazebo_ros',
        #    executable='gzserver',
        #    output='screen',
        #    parameters=[{'use_sim_time': True}],
        #    arguments=['-s', 'libgazebo_ros_factory.so'],
        #),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_controller'],
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}],  # Enable simulation time
        ),
        
        # Launch robot_state_publisher to broadcast transforms
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        ]
    )