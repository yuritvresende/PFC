import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the launch files
    gazebo_launch_file = os.path.join(
        get_package_share_directory('testing'), 'launch', 'start_world.launch.py')
    
    spawn_launch_file = os.path.join(
        get_package_share_directory('testing'), 'launch', 'platform.launch.py')
        
    controller_launch_file = os.path.join(
        get_package_share_directory('testing'), 'launch', 'start_controller.launch.py')
    

    return LaunchDescription([
        # Launch first_launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file),
            launch_arguments={}.items(),
        ),

        # Delay before launching the robot_state_publisher
        TimerAction(
            period=10.0,  # Adjust the delay time as needed
            actions=[
                # Launch second_launch.py
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(spawn_launch_file),
                    launch_arguments={}.items(),
                ),
            ],
        ),

        TimerAction(
            period=10.0,  # Adjust the delay time as needed
            actions=[
                # Launch second_launch.py
               IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(controller_launch_file),
                    launch_arguments={}.items(),
                ),
            ],
        ),
    ])
