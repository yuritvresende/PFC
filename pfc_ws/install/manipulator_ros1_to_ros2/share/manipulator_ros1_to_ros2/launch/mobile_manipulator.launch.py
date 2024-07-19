import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Arguments
    robot_description_package = 'manipulator_ros1_to_ros2'
    robot_description_file = 'mobile_manipulator.urdf'
    robot_description_path = LaunchConfiguration('robot_description_path', default=os.path.join(
        get_package_share_directory(robot_description_package), 'urdf', robot_description_file))

    arm_control_config = LaunchConfiguration('arm_control_config', default=os.path.join(
        get_package_share_directory(robot_description_package), 'config', 'arm_control.yaml'))

    joint_state_controller_config = LaunchConfiguration('joint_state_controller_config', default=os.path.join(
        get_package_share_directory(robot_description_package), 'config', 'joint_state_controller.yaml'))

    control_config = LaunchConfiguration('control_config', default=os.path.join(
        get_package_share_directory(robot_description_package), 'config', 'control.yaml'))

    # Nodes
    
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

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description_path, arm_control_config, joint_state_controller_config, control_config],
        output='screen'
    )

    arm_controller_spawner_node = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'arm_controller'],
        output='screen'
    )

    joint_state_controller_spawner_node = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_controller'],
        output='screen'
    )

    base_controller_spawner_node = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'robot_base_joint_publisher', 'robot_base_velocity_controller'],
        output='screen'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_path}],
        output='screen'
    )

    # Launch description
    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument('robot_description_package', default_value=robot_description_package))
    ld.add_action(DeclareLaunchArgument('robot_description_file', default_value=robot_description_file))
    ld.add_action(DeclareLaunchArgument('arm_control_config', default_value=arm_control_config))
    ld.add_action(DeclareLaunchArgument('joint_state_controller_config', default_value=joint_state_controller_config))
    ld.add_action(DeclareLaunchArgument('control_config', default_value=control_config))

    # Add nodes and processes to the launch description
    ld.add_action(spawn_robot)
    ld.add_action(controller_manager_node)
    ld.add_action(arm_controller_spawner_node)
    ld.add_action(joint_state_controller_spawner_node)
    ld.add_action(base_controller_spawner_node)
    ld.add_action(robot_state_publisher_node)

    return ld
