# Forward Kinematics package

## Overview

This package implements a service client and a service server to compute forward kinematics. The service message used is defined in [kinematics_msgs](../kinematics_msgs) package.

---

## Package content

* ```src``` folder contains:
  * ```fk_client``` node that implements the client side. It request the computation to the server first and then verify the result obtained with the computation made with the `move_group`'s service `/compute_fk`;
  * ```fk_server``` node that implements the server side. It receive the service request from the client, compute the fk calculation and then send it back to the client.

* ```launch``` folder contains the launch file that bring up the smartsix configuration package and the server node.

---

## How to run

After building the package and its dependencies (it does automatically) open a new shell and run the above command:

```bash
roslaunch forward_kinematics forward_kinematics.launch
```

Open a new shell and run the above command:

```bash
rosrun forward_kinematics fk_client_node
```
