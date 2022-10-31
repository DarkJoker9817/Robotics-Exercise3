# Ex3 Forward/Inverse Kinematics

## The task

* Implement a service server that computes the forward kinematics of a robot and a service client that uses this service and prints the solution to stdout.
* Verify the result by comparing it with the service /compute_fk of the move_group_node
* Implement an action server that computes all the inverse kinematic solutions of a robot (one by one) and an action client that uses this action and prints the solutions to stdout (one by one, as they are received). The action server does not send the same solution twice and stops when all solutions have been found. At that time, they are returned all together.
* Repeat the experiment at point 3 by neglicting joint limits.
* Visualize the IK solutions in RViz.

---

## Repository content

This repository is compose by three ROS packages:

* ```kinematics_msgs``` package that contains the definition of a service message to compute forward kinematics and an action message used to compute inverse kinematics;
* ```forward_kinematics``` package that contains the nodes used to compute forward kinematics;
* ```inverse_kinematics``` package that contains the nodes used to compute inverse kinematics.

Read the `Readme.md` file in each package for more info.
