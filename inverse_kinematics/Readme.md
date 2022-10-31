# Inverse Kinematics package

## Overview

This package implements an action client and an action server to compute inverse kinematics. The action message used is defined in [kinematics_msgs](../kinematics_msgs) package.

---

## Package content

* ```src``` folder contains:
  * ```client.cpp``` node for requesting the inverse kinematics computation for a robot. Through the use of a callback function the solutions are displayed one by one as they are retrieved and, once the server computes all of them, send in one shot at the end. The design of the node allows to modify from code the coordinates of the end-effector pose that have to be sent to server in order to compute kinematics.
  * ```server.cpp``` node that implements the server side that calculate the inverse kinematics requested by the client. The implementation is done relying on a custom class called ```InverseKinematicsAction``` defined in [inverse_kinematics_action.cpp](./include/inverse_kinematics/inverse_kinematics_action.h).
  * ```inverse_kinematics_action.cpp``` it is the module that instantiate the NodeHandle and all the functions necessary for calculating the inverse kinematics and sending the results to the client. It's the business logic of [server.cpp](./src/server.cpp) file.
* ```include``` folder contains the prorotypes of the functions necessary for calculating the inverse kinematics and the definition of the ```InverseKinematicsAction``` class.
* ```launch``` folder contains the file used to setup the sistem and launch the robot configuration node and the server node. For a correct visualization launch the client node in another terinal.

---

## How to run

Open a new terminal window and run the following command:

```bash
roslaunch inverse_kinematics ik_solver.launch
```

Open a second terminal window and run the following command:

```bash
rosrun inverse_kinematics client_node
```
