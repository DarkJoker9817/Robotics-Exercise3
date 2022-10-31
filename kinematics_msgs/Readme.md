# Kinematic Messages Package

## Overview

This package implements messages, in particular services and actions, which are used to compute `forward kinematics` and `inverse kinematics` of a robot.

## Package content

* ```srv``` folder contains the service definition [GetFKSolution.srv](./srv/GetFKSolution.srv) used to compute `forward_kinematics`. It is composed by a request (moveit_msgs/RobotState) that represent the state of the joints of the robot and a response (geometry_msgs/Pose) that represent the pose (position + orientation) of the end-effector of the robot;
* ```action``` folder contains the action definition [GetIKSolutions.srv](./action/GetIKSolutions.action) used to compute `inverse_kinematics`. It is composed by a goal (geometry_msgs/Pose) that represent the pose (position + orientation) of the end-effector of the robot, a result (moveit_msgs/RobotState[]) that represent the multiple solutions to the inverse kinematics calculations and a feedback (moveit_msgs/RobotState) that represent the single solution that are retrieved one by one when they are calculated.
