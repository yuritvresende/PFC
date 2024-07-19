import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
import launch_ros.descriptions

import xacro

def generate_launch_description():

    print("Fetching URDF ==>")

    robot_model_path = os.path.join(
        get_package_share_directory('manipulator_ros1_to_ros2'))
    xacro_file = os.path.join(robot_model_path, 'urdf', 'mobile_manipulator.urdf')

    # convert XACRO file into URDF
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}


    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[params],
    )
    
    # arm_controller_spawner_node = Node(
    # 	package='controller_manager',
    # 	args = "spawn arm_controller",
    #     executable='spawn arm_controller',
    #     name='arm_controller_spawner',
    #     output='screen',  
    # )

    # joint_state_controller_node = Node(
    # 	package='controller_manager',
    #     args = "spawn joint_state_controller",
    #     executable='spawn joint_state_controller',
    #     name='joint_state_controller',
    #     output='screen',    
    # )

    # base_controller_spawner_node = Node(
    # 	package='controller_manager',
    # 	args = "robot_base_joint_publisher robot_base_velocity_controller",
    #     executable='robot_state_publisher',
    #     name='base_controller_spawner',
    #     output='screen',
  
    # )
    
    # Position and orientation
    # [X, Y, Z]
    position = [0.0, 0.0, 0.15]
    # [Roll, Pitch, Yaw]
    orientation = [0.0, 0.0, 0.0]
    # Base Name or robot
    robot_base_name = "robot_1"
    entity_name = robot_base_name

    # Spawn ROBOT Set Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity',
                   entity_name,
                   '-x', str(position[0]), '-y', str(position[1]
                                                     ), '-z', str(position[2]),
                   '-R', str(orientation[0]), '-P', str(orientation[1]
                                                        ), '-Y', str(orientation[2]),
                   '-topic', '/robot_description'
                   ]
    )

    # create and return launch description object
    return LaunchDescription(
        [            
            robot_state_publisher_node,
            spawn_robot
            # arm_controller_spawner_node,
            # joint_state_controller_node,
            # base_controller_spawner_node
        ]
    )
