/* 
 * -------------------------------------------------------------------
 * This module has been developed as an exercise for Robotics course
 * @ UniSa.
 *
 * Title:   server.cpp
 * Author:  Ugo Barbato
 * Date:    Oct 29, 2022
 *
 * This node implements an action server for computing the inverse kinematics
 * of a robot simply calling an external constructor specially implemented
 * for this task.
 *
 * -------------------------------------------------------------------
 */

#include <inverse_kinematics/inverse_kinematics_action.h>

int main(int argc, char **argv) {

    ROS_INFO("Bring up the action_server node");
    ros::init(argc, argv, "ik_action_server");

    // Calling the constructor of the InverseKinematicsAction class
    inverse_kinematics::InverseKinematicsAction ik_action;

    ROS_INFO("Started inverse kinematics action server");

    // Keeps the node alive and ready to receive other goals
    ros::spin();

    ros::shutdown();
    return 0;
}