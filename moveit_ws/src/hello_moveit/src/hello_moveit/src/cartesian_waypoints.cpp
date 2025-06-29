#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "cartesian_waypoints",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("cartesian_waypoints");

  // executors
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node, "lifted_arm");

  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
    node, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC,
    move_group_interface.getRobotModel()};
  
  moveit_visual_tools.deleteAllMarkers();

//   auto const draw_title = [&moveit_visual_tools](auto text) {
//     auto const text_pose = [] {
//         auto msg = Eigen::Isometry3d::Identity();
//         msg.translation().z() = 1.0;  // Place text 1m above the base link
//         return msg;
//     }();
//     moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
//                                     rviz_visual_tools::XLARGE);
//   };
  
  auto jmg = move_group_interface.getRobotModel()->getJointModelGroup("lifted_arm");

  moveit::core::RobotState start_state(*move_group_interface.getCurrentState());

  geometry_msgs::msg::Pose new_start_pose;
  new_start_pose.orientation.w = 1.0;
  new_start_pose.position.x = 0.55;
  new_start_pose.position.y = -0.05;
  new_start_pose.position.z = 0.8;
  start_state.setFromIK(jmg, new_start_pose);
  move_group_interface.setStartState(start_state);




//   std::vector<geometry_msgs::msg::Pose> waypoints;
//   waypoints.push_back(new_start_pose);

//   geometry_msgs::msg::Pose target_pose = new_start_pose;
  
//   target_pose.position.y -= 0.2;
//   waypoints.push_back(target_pose);

//   target_pose.position.x -= 0.1;
//   waypoints.push_back(target_pose);

//   target_pose.position.y -= 0.4;
//   waypoints.push_back(target_pose);

//   moveit_msgs::msg::RobotTrajectory trajectory;
//   const double jump_threshold = 0.0;
//   const double eef_step = 0.01;
//   double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  // RCLCPP_INFO(logger, "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

//   moveit_visual_tools.deleteAllMarkers();
//   draw_title("cartesian path");

//   moveit_visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
//   moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
//   for (std::size_t i = 0; i < waypoints.size(); ++i)
    // visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
//   moveit_visual_tools.trigger();

//   move_group_interface.execute(trajectory);

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
