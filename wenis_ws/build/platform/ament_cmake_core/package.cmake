set(_AMENT_PACKAGE_NAME "platform")
set(platform_VERSION "0.0.0")
set(platform_MAINTAINER "yuritvr <yuritvresende@gmail.com>")
set(platform_BUILD_DEPENDS "urdf" "xacro" "ros2_control" "gazebo_ros2_control" "joint_state_broadcaster" "joint_trajectory_controller" "rviz" "joint_state_publisher" "robot_state_publisher" "velocity_controllers")
set(platform_BUILDTOOL_DEPENDS "ament_cmake")
set(platform_BUILD_EXPORT_DEPENDS "urdf" "xacro" "ros2_control" "gazebo_ros2_control" "joint_state_broadcaster" "joint_trajectory_controller" "rviz" "joint_state_publisher" "robot_state_publisher" "velocity_controllers")
set(platform_BUILDTOOL_EXPORT_DEPENDS )
set(platform_EXEC_DEPENDS "urdf" "xacro" "ros2_control" "gazebo_ros2_control" "joint_state_broadcaster" "joint_trajectory_controller" "rviz" "joint_state_publisher" "robot_state_publisher" "velocity_controllers")
set(platform_TEST_DEPENDS "ament_lint_auto" "ament_lint_common")
set(platform_GROUP_DEPENDS )
set(platform_MEMBER_OF_GROUPS )
set(platform_DEPRECATED "")
set(platform_EXPORT_TAGS)
list(APPEND platform_EXPORT_TAGS "<build_type>ament_cmake</build_type>")
list(APPEND platform_EXPORT_TAGS "<gazebo_ros gazebo_model_path=\"..\"/>")