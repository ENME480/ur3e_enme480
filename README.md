Thi repository is for ENME480 Labs to enable communication between ur_drivers and our custom topics and messages.

This package uses the `joint_trajectory_controller` instead of `scaled_joint_trajectory_controller`. This is a Python based package so it requires the `ur3e_mrc` C++ pacakge for the custom messges.

## Prerequisites

Install `ur3e_mrc` package for custom messages

```bash
git clone https://github.com/MarylandRoboticsCenter/ur3e_mrc.git
```

## Usage

Build and source this package

### Launch

```bash
ros2 launch ur3e_enme480 ur3e_sim_enme480.launch.py 
```

It should publish the following topics
- `/ur3/position` - publishes the pose of end effector (or last link) relative to the world frame in Gazebo
- `/ur3/command` - publishes to `/joint_trajectory_controller` to move the robot given joint angles 

## Robot Motion

For enabling forward kinematics of the robot in Gazebo

```bash
ros2 topic pub --once /ur3/command ur3e_mrc/msg/CommandUR3e "destination: [0, 0, 0, 0, 0, 0]
v: 1.0
a: 1.0
io_0: false" 
```