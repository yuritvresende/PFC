import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the paths to the other launch files
    launch1_path = get_package_share_directory('arm_pkg') + '/launch/spawn_gazebo.launch.py'
    launch2_path = get_package_share_directory('arm_pkg') + '/launch/start_controller.launch.py'

    # Create actions to include the other launch files
    include_launch1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch1_path)
    )

    include_launch2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch2_path)
    )

    # Create the launch description
    ld = LaunchDescription()

    # Add the actions to include the other launch files
    ld.add_action(include_launch1)
    ld.add_action(include_launch2)

    return ld

if __name__ == '__main__':
    generate_launch_description()