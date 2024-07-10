colcon build
source /opt/ros/humble/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source /usr/share/gazebo/setup.sh
source ~/PFC/wenis_ws/install/setup.bash
killall -9 gzserver
killall gzclient
#ros2 launch platform platform.launch.py
ros2 launch platform aplatform.launch.py
#ros2 launch arm_pkg main_arm.launch.py
