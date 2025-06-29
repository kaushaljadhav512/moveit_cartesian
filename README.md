# moveit_cartesian

## Overview

This branch targets ROS 2 `humble`.

## Instructions to run

Launch rviz, move_it and ros2_conrol with "ros2 launch lifted_arm_config demo.launch.py"

In a separate terminal, run "ros2 run cartesian_motion cartesian_planner"

This will launch the planning and executor node.

When launched, first the robot arm will plan and move to a fixed height(z) as defined in cartesian_planner.cpp (z=1.0 in our case)

Then the robot will plan a cartesian trajectory for a list of waypoints (defined in file)

Once the plan is complete, the trajectory is passed to the set_avg_cartesian_speed function along with a specified speed.
It then returns the modified trajectory constraining the end effector at a specified average speed adhering to the joints's kinematic constraints.

This plan is then executed.


References: https://github.com/moveit/moveit/pull/2674
