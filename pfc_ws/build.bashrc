colcon build
source ~/PFC/pfc_ws/install/setup.bash
source /usr/share/gazebo/setup.sh
killall -9 gzserver
killall gzclient
cd src
