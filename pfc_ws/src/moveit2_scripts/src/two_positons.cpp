#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");
static const std::string PLANNING_GROUP = "arm";

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("move_group_arm", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  moveit::planning_interface::MoveGroupInterface move_group(move_group_node,
                                                            PLANNING_GROUP);

  const moveit::core::JointModelGroup *joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  RCLCPP_INFO(LOGGER, "Planning frame: %s",
              move_group.getPlanningFrame().c_str());

  RCLCPP_INFO(LOGGER, "End effector link: %s",
              move_group.getEndEffectorLink().c_str());

  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(),
            move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);

  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group,
                                         joint_group_positions);

  joint_group_positions[0] = 0.00;  // Shoulder Pan
  joint_group_positions[1] = -0.57; // Shoulder Lift
  joint_group_positions[2] = 0.00;  // Elbow
  joint_group_positions[3] = 0.00;  // Wrist 1
  joint_group_positions[4] = 0.00;  // Wrist 2
  // joint_group_positions[5] = 0.00;  // Wrist 3
  move_group.setJointValueTarget(joint_group_positions);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success =
      (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  // Execute
  RCLCPP_INFO(LOGGER, "Going to Position 1");
  move_group.execute(my_plan);

  joint_group_positions[0] = -0.57; // Shoulder Pan
  joint_group_positions[1] = -0.57; // Shoulder Lift
  joint_group_positions[2] = 0.57;  // Elbow
  joint_group_positions[3] = 0.00;  // Wrist 1
  joint_group_positions[4] = 0.00;  // Wrist 2
  joint_group_positions[5] = 1.57;  // Wrist 3
  move_group.setJointValueTarget(joint_group_positions);

  success =
      (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  // Execute
  RCLCPP_INFO(LOGGER, "Going to Position 2");
  move_group.execute(my_plan);

  rclcpp::shutdown();
  return 0;
}