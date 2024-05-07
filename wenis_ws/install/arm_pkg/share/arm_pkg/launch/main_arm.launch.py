import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the launch files
    gazebo_launch_file = os.path.join(
        get_package_share_directory('arm_pkg'), 'launch', 'start_world.launch.py')
    
    arm_launch_file = os.path.join(
        get_package_share_directory('arm_pkg'), 'launch', 'start_arm.launch.py')
    
    move_group_launch_file = os.path.join(
        get_package_share_directory('moveit_config'), 'launch', 'move_group.launch.py')
    
    rviz_launch_file = os.path.join(
        get_package_share_directory('moveit_config'), 'launch', 'moveit_rviz.launch.py')
    


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
                    PythonLaunchDescriptionSource(arm_launch_file),
                    launch_arguments={}.items(),
                ),
            ],
        ),
        # Delay before launching the spawner
        TimerAction(
            period=5.0,  
            actions=[
                # Launch second_launch.py
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(move_group_launch_file),
                    launch_arguments={}.items(),
                ),
            ],
        ),

        TimerAction(
            period=5.0, 
            actions=[
                # Launch second_launch.py
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(rviz_launch_file),
                    launch_arguments={}.items(),
                ),
            ],
        ),

    ])
