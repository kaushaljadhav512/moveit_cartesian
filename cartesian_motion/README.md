# Cartesian Motion package

Contains [cartesian_planner](./src/cartesian_planner.cpp) node for path planning with constant end-effector speed in a cartesian plane.

A sequence of waypoints in a 2d cartesian plane are defined for the end-effector to reach.

Using the ```computeCartesianPath()``` function of the ```moveit::planning_interface::MoveGroupInterface```.

The returned trajectory is then passed on to the ```setAvgCartesianSpeed()``` along with an average speed for the end-effector to follow.

The ```setAvgCartesianSpeed()``` function modifies the trajectory for the joints to execute.

### Example:

Robot starts with joints at initial positions, current end-effector pose is (x=0.00113393, y=0.2907, z=1.4848, w=0.70).

```
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

```
Average speed for the end-effector to follow is 0.015 m/s

#### Trajectory executed:

<p align="middle" float="middle">
  <img width="50.0%" src="https://github.com/user-attachments/assets/e8b26957-7d6d-4868-8f8c-2285cc8025e3" alt="Animation of constant velocity cartesian"/>
</p>


#### Joint Velocities:

<p align="middle" float="middle">
  <img width="50.0%" src="https://github.com/user-attachments/assets/d12190a7-4862-4a82-b2aa-d84822978701" alt="joint_velocities"/>
</p>
