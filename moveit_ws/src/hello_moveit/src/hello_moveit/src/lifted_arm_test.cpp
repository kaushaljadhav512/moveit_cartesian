#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "lifted_arm_test",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });


  // Create a ROS logger
  auto const logger = rclcpp::get_logger("lifted_arm");

  // Next step goes here
  // Create the MoveIt MoveGroup Interface
  auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node, "lifted_arm");

  // auto jmg = move_group_interface.getRobotModel()->getJointModelGroup("lifted_arm");
  moveit::core::RobotState start_state(*move_group_interface.getCurrentState());
    
  auto end_effector_name = move_group_interface.getEndEffectorLink();
  
  RCLCPP_INFO(logger, "End Effector name is: %s", end_effector_name.c_str());

  auto current_pose = move_group_interface.getCurrentPose();  
  RCLCPP_INFO_STREAM(logger, "Current Pose is, x=" << current_pose.pose.position.x << ", y=" << current_pose.pose.position.y << ", z="<< current_pose.pose.position.z << ", w=" << current_pose.pose.orientation.w);
  
  std::vector<geometry_msgs::msg::Pose> waypoints;

  geometry_msgs::msg::Pose target_pose = current_pose.pose;
  
  target_pose.position.z -= 0.5;
  target_pose.position.x -= 0.2;
  waypoints.push_back(target_pose);

  target_pose.position.x -= 0.2;
  waypoints.push_back(target_pose);

  target_pose.position.x -= 0.2;
  waypoints.push_back(target_pose);

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  RCLCPP_INFO(logger, "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

  if (fraction>0.9) {
    move_group_interface.execute(trajectory);
  }
  else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }


  // auto current_pose = move_group_interface.getCurrentPose();
  
  // RCLCPP_INFO_STREAM(logger, "Current Pose is, x=" << current_pose.pose.position.x << ", y=" << current_pose.pose.position.y << ", z="<< current_pose.pose.position.z << ", w=" << current_pose.pose.orientation.w);

  // geometry_msgs::msg::Pose target_pose = current_pose.pose;
  
  // target_pose.x += 0.5;
  
  // Set a target Pose
  // auto const target_pose = []{
  // geometry_msgs::msg::Pose msg;
  // msg.orientation.w = 1.0;
  // msg.position.x = 0.28;
  // msg.position.y = -0.2;
  // msg.position.z = 0.5;
  // return msg;
  // }();


  // move_group_interface.setPoseTarget(target_pose);

  // // Create a plan to that target pose
  // auto const [success, plan] = [&move_group_interface]{
  // moveit::planning_interface::MoveGroupInterface::Plan msg;
  // auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  // return std::make_pair(ok, msg);
  // }();

  // // Execute the plan
  // if(success) {
  // move_group_interface.execute(plan);
  // } else {
  // RCLCPP_ERROR(logger, "Planning failed!");
  // }

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}