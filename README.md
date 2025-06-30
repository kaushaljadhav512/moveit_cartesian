# moveit_cartesian

## Overview

This branch targets ROS 2 `humble`.

Below is an overview of the included packages, with a short description of their purpose.

- [**Universal_Robots_ROS2_Description**](./Universal_Robots_ROS2_Description) – Universal Robots ROS2 Description package with the modified UR10e robot (attached with a vertical prismatic lift)
- [**lifted_arm_config**](./lifted_arm_config) – MoveIt 2 configuration for the modified lifted_arm robot.
- [**cartesian_motion**](./cartesian_motion) – Contains the node for constant-velocity cartesian planning.

## Dependencies

These are the primary dependencies required to use this project.

- ROS 2 [Humble](https://docs.ros.org/en/humble/Installation.html)
- [MoveIt 2](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html)

All additional dependencies are pulled via [rosdep](https://wiki.ros.org/rosdep) during the building process below.

## Building

Clone this repository, import dependencies, install dependencies, and build with [colcon](https://colcon.readthedocs.io).

```bash
# Clone this repository into your moveit_ws ROS 2 Humble workspace
git clone https://github.com/kaushaljadhav512/moveit_cartesian
# Install dependencies
rosdep install -y -r -i --rosdistro humble --from-paths .
# Build
colcon build
```

### Sourcing

Before utilising this package, remember to source the ROS 2 workspace.

```bash
source install/local_setup.bash
```


## Instructions to run

For starting up RViz with the lifted_arm URDF loaded and the MoveIt! Motion Planning plugin run: 

```bash
Rviz, with "ros2 launch lifted_arm_config demo.launch.py"
```


In a separate terminal, run the constant-velocity cartesian path planning node
```bash
ros2 run cartesian_motion cartesian_planner
```


When launched, the robot arm will plan and move to a fixed height(z) as defined in cartesian_planner.cpp (z=1.0 in our case)

Then the robot will plan a cartesian trajectory for a list of waypoints in the x-y plane

The planned trajectory is then retified with the set_avg_cartesian_speed function to constrain the end effector at a specified average speed.


## TODO

1. Add support for Gazebo Fortress
2. Add Compliance Control (maintain a constant force against a
wall (e.g., along X or Z) without direct torque control.)


## References

* https://www.kwesirutledge.info/thoughts/ur_urdfs
* https://www.youtube.com/watch?v=yIVc5Xq0Xm4&t=387s
* https://github.com/AndrejOrsula/panda_gz_moveit2/tree/9e5fabed267db78a91a2cd185ee6a89b799f3be3?tab=readme-ov-file#panda_ign_moveit2
* https://github.com/moveit/moveit/pull/2674
