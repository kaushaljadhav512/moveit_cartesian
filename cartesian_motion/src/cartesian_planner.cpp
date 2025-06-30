#include <memory>
#include <fstream>
#include <iomanip>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <moveit/move_group_interface/move_group_interface.h>

// Function to set the average Cartesian speed of a planned trajectory
void setAvgCartesianSpeed(moveit_msgs::msg::RobotTrajectory &traj, const std::string end_effector, const double speed, moveit::core::RobotStatePtr kinematic_state, const rclcpp::Logger &logger)
{

    kinematic_state->setToDefaultValues();
 
    int num_waypoints = traj.joint_trajectory.points.size();
    const std::vector<std::string> joint_names = traj.joint_trajectory.joint_names;
 
    // Set robot state to the first waypoint's joint positions
    kinematic_state->setVariablePositions(joint_names, traj.joint_trajectory.points.at(0).positions);
 
    // Get the initial end-effector pose
    Eigen::Affine3d current_end_effector_state = kinematic_state->getGlobalLinkTransform(end_effector);
    Eigen::Affine3d next_end_effector_state;

    double euclidean_distance, new_timestamp, old_timestamp, q1, q2, q3, dt1, dt2, v1, v2, a;
    
    trajectory_msgs::msg::JointTrajectoryPoint *prev_waypoint, *curr_waypoint, *next_waypoint;
 
    // Loop through trajectory waypoints to adjust timing for desired speed
    for(int i = 0; i < num_waypoints - 1; i++)
    {
        curr_waypoint = &traj.joint_trajectory.points.at(i);
        next_waypoint = &traj.joint_trajectory.points.at(i+1);

        // Set robot state to next waypoint's joint positions
        kinematic_state->setVariablePositions(joint_names, next_waypoint->positions);
 
        // Get next end-effector pose
        next_end_effector_state = kinematic_state->getGlobalLinkTransform(end_effector);

        // Compute Euclidean distance between current and next end-effector poses
        euclidean_distance = pow(pow(next_end_effector_state.translation()[0] - current_end_effector_state.translation()[0], 2) + 
                            pow(next_end_effector_state.translation()[1] - current_end_effector_state.translation()[1], 2) + 
                            pow(next_end_effector_state.translation()[2] - current_end_effector_state.translation()[2], 2), 0.5);
 
        // Calculate current and new timestamps based on desired speed
        double curr_time = curr_waypoint->time_from_start.sec + curr_waypoint->time_from_start.nanosec * 1e-9;
        new_timestamp = curr_time + (euclidean_distance / speed);
        old_timestamp = next_waypoint->time_from_start.sec + next_waypoint->time_from_start.sec * 1e-9;

        // Update the next waypoint's timestamp if within limits
        if(new_timestamp > old_timestamp) {
            next_waypoint->time_from_start.sec = std::floor(new_timestamp);
            next_waypoint->time_from_start.nanosec = std::floor((new_timestamp - next_waypoint->time_from_start.sec) * 1e9);
        }
        // Warn if speed to fast
        else
        {
            RCLCPP_WARN(logger, "Average speed is too fast. Moving as fast as joint constraints allow.");
        }
        
        // Update current end-effector pose for next iteration
        current_end_effector_state = next_end_effector_state;
    }
    
    // Loop to update velocities and accelerations at each waypoint
    for(int i = 0; i < num_waypoints; i++)
    {
        curr_waypoint = &traj.joint_trajectory.points.at(i);
        if(i > 0)
            prev_waypoint = &traj.joint_trajectory.points.at(i-1);
        if(i < num_waypoints-1)
            next_waypoint = &traj.joint_trajectory.points.at(i+1);
 
        // Compute time intervals before and after current waypoint
        if(i == 0)
            dt1 = dt2 = next_waypoint->time_from_start.sec - curr_waypoint->time_from_start.sec + (next_waypoint->time_from_start.nanosec - curr_waypoint->time_from_start.nanosec) * 1e-9;
        else if(i < num_waypoints-1)
        {
            dt1 = curr_waypoint->time_from_start.sec - prev_waypoint->time_from_start.sec + (curr_waypoint->time_from_start.nanosec - prev_waypoint->time_from_start.nanosec) * 1e-9;
            dt2 = next_waypoint->time_from_start.sec - curr_waypoint->time_from_start.sec + (next_waypoint->time_from_start.nanosec - curr_waypoint->time_from_start.nanosec) * 1e-9;
        }
        else
            dt1 = dt2 = curr_waypoint->time_from_start.sec - prev_waypoint->time_from_start.sec + (curr_waypoint->time_from_start.sec - prev_waypoint->time_from_start.sec) * 1e-9;
 
        // For each joint, compute velocities and accelerations
        for(int j = 0; j < joint_names.size(); j++)
        {
            if(i == 0)
            {
                q1 = next_waypoint->positions.at(j);
                q2 = curr_waypoint->positions.at(j);
                q3 = q1;
            }
            else if(i < num_waypoints-1)
            {
                q1 = prev_waypoint->positions.at(j);
                q2 = curr_waypoint->positions.at(j);
                q3 = next_waypoint->positions.at(j);
            }
            else
            {
                q1 = prev_waypoint->positions.at(j);
                q2 = curr_waypoint->positions.at(j);
                q3 = q1;
            }
 
            if(dt1 == 0.0 || dt2 == 0.0)
                v1 = v2 = a = 0.0;
            else
            {
                v1 = (q2 - q1)/dt1;
                v2 = (q3 - q2)/dt2;
                a = 2.0*(v2 - v1)/(dt1+dt2);
            }
            
            // Set average velocity and acceleration for the joint at this waypoint
            curr_waypoint->velocities.at(j) = (v1+v2)/2;
            curr_waypoint->accelerations.at(j) = a;
        }
    }
}

// Function to save joint velocities from the trajectory to a CSV file
void save_joint_velocities(moveit_msgs::msg::RobotTrajectory &trajectory) {

  std::ofstream file("joint_velocities.csv");
  file << "time";

  // Write joint names as CSV header
  for (const auto& name : trajectory.joint_trajectory.joint_names) {
    file << "," << name;
  }

  file << std::endl;

  // Write velocities for each waypoint
  for (const auto& point : trajectory.joint_trajectory.points) {
    double time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
    file << std::fixed << std::setprecision(6) << time;

    for (const auto& vel : point.velocities) {
        file << "," << vel;
    }
    file << std::endl;
  }

  file.close();
}


int main(int argc, char * argv[])
{

  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "lifted_arm_test",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a single-threaded executor and spin in a separate thread
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  auto const logger = rclcpp::get_logger("lifted_arm");

  // Create MoveGroupInterface for controlling the robot arm
  auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node, "lifted_arm");

  // Get the current robot state
  moveit::core::RobotState start_state(*move_group_interface.getCurrentState());

  // Get the name of the end-effector link
  auto end_effector = move_group_interface.getEndEffectorLink();
  
  // Example code to set z-height manually
    // Set desired height to perform cartesian
    // auto current_pose = move_group_interface.getCurrentPose();  
    // geometry_msgs::msg::Pose target_pose = current_pose.pose;
    // target_pose.position.z = 1.0;
    // target_pose.position.x += 0.2;
    // target_pose.position.y += 0.2;
    // move_group_interface.setPoseTarget(target_pose);
    // moveit::planning_interface::MoveGroupInterface::Plan plan_;
    // // non- cartesian planning and execution
    // if (move_group_interface.plan(plan_)) {
    //   move_group_interface.execute(plan_);
    // } else {
    //   RCLCPP_ERROR(logger, "Planning failed!");
    // }

  // Get and log the current end-effector pose
  auto current_pose = move_group_interface.getCurrentPose();  
  RCLCPP_INFO_STREAM(logger, "Current Pose is, x=" << current_pose.pose.position.x << ", y=" << current_pose.pose.position.y << ", z="<< current_pose.pose.position.z << ", w=" << current_pose.pose.orientation.w);
  
  // Define a series of Cartesian waypoints for the end-effector to follow
  std::vector<geometry_msgs::msg::Pose> waypoints;

  geometry_msgs::msg::Pose cartesian_pose = current_pose.pose;
  waypoints.push_back(cartesian_pose);

  cartesian_pose.position.x -= 0.5;
  cartesian_pose.position.y += 0.7;
  waypoints.push_back(cartesian_pose);

  cartesian_pose.position.y -= 0.4;
  waypoints.push_back(cartesian_pose);

  cartesian_pose.position.x -= 0.3;
  cartesian_pose.position.y -= 0.5;
  waypoints.push_back(cartesian_pose);

  cartesian_pose.position.x += 0.7;
  cartesian_pose.position.y += 0.2;
  waypoints.push_back(cartesian_pose);

  // Plan a Cartesian path through the waypoints
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(move_group_interface.getRobotModel()));
  
  // Adjust the trajectory timing to achieve the desired Cartesian speed of end-effector
  // here the speed is set as 0.015 m/s
  setAvgCartesianSpeed(trajectory, end_effector, 0.015, kinematic_state, logger);

  // Save the computed joint velocities to a CSV file
  save_joint_velocities(trajectory);

  RCLCPP_INFO(logger, "Visualizing Cartesian path (%.2f%% achieved)", fraction * 100.0);

  // Execute the trajectory if the planned path is sufficiently complete
  if (fraction>0.9) {
    move_group_interface.execute(trajectory);
  }
  else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}