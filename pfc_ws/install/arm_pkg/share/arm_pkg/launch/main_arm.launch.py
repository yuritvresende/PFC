import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from setuptools import setup

package_name = 'arm_pkg'

# setup(
#     name=package_name,
#     version='0.0.1',
#     packages=[package_name],
#     py_modules=[
#         'joint_trajectory_publisher',
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='your_name',
#     maintainer_email='your_email@example.com',
#     description='Your package description',
#     license='License',
#     tests_require=['pytest'],
#     entry_points={
#         'console_scripts': [
#             'joint_trajectory_publisher = joint_trajectory_publisher:main',  # Adapte conforme necessário
#         ],
#     },
# )


def generate_launch_description():
    # Get the path to the launch files
    gazebo_launch_file = os.path.join(
        get_package_share_directory('arm_pkg'), 'launch', 'start_world.launch.py')
    
    spwan_launch_file = os.path.join(
        get_package_share_directory('arm_pkg'), 'launch', 'spawn_gazebo.launch.py')
    
    controller_launch_file = os.path.join(
        get_package_share_directory('arm_pkg'), 'launch', 'start_controller.launch.py')
    

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
                    PythonLaunchDescriptionSource(spwan_launch_file),
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