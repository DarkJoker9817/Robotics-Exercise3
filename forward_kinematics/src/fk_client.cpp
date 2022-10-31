/* 
 * -------------------------------------------------------------------
 * This module has been developed as an exercise for Robotics course
 * @ UniSa.
 *
 * Title:   fk_client.cpp
 * Author:  Ugo Barbato
 * Date:    Oct 27, 2022
 *
 * This node implements a client that requests a service for the 
 * computation of the forward kinematics of a robot using the 
 * GetFkSolution.srv service.
 *
 * -------------------------------------------------------------------
 */

#include "ros/ros.h"
#include "kinematics_msgs/GetFKSolution.h"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit_msgs/GetPositionFK.h>

int main(int argc, char **argv) {

    ROS_INFO("Starting \"fk_client\" node");
    ros::init(argc, argv, "fk_client");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<kinematics_msgs::GetFKSolution>("fk_solver");

    // Create service request
    kinematics_msgs::GetFKSolution custom_fk_service;

    // Load robot model
    robot_model_loader::RobotModelLoaderConstPtr robot_model_loader = std::shared_ptr<const robot_model_loader::RobotModelLoader>(new robot_model_loader::RobotModelLoader("robot_description"));

    // Get the robot kinematic model
    robot_model::RobotModelConstPtr kinematic_model = robot_model_loader->getModel();
    
    // Create default robot state from kinematic model
    moveit::core::RobotState robot_state(kinematic_model);

    // Get planning group name stored on parameter server
    std::string planning_group_name;

    if(!nh.getParam("planning_group_name", planning_group_name)) {
        ROS_ERROR("\"planning_group_name\" is not defined on parameter server");
        return false;    
    }

    // Get the planning group
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(planning_group_name);

    // Create a vector of joint positions and set the robot to these positions
    const std::vector<double> joint_positions = {0, 0.80, 0.79, 1.90, 1.05, -3.14159};
    robot_state.setJointGroupPositions(joint_model_group, joint_positions);
    
    // Convert the robot configuration to a RobotState message.
    moveit::core::robotStateToRobotStateMsg(robot_state, custom_fk_service.request.robot_state);

    if(!client.call(custom_fk_service))
        ROS_ERROR("Could not compute forward kinematics");

    // Convert orientation of obtained Pose to RPY
    tf2::Quaternion quaternion;
    tf2::fromMsg(custom_fk_service.response.end_effector_pose.orientation, quaternion);

    tf2::Matrix3x3 matrix(quaternion);
    tf2Scalar roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);

    // Print solution to stdout
    std::ostringstream custom_solution;

    custom_solution << "Computed forward kinematic solution with service: " << std::endl;
    custom_solution << "Position (XYZ): [";
    custom_solution << custom_fk_service.response.end_effector_pose.position.x << ", ";
    custom_solution << custom_fk_service.response.end_effector_pose.position.y << ", ";
    custom_solution << custom_fk_service.response.end_effector_pose.position.z << "]" << std::endl;
    custom_solution << "Orientation (RPY): [" << roll << ", " << pitch << ", " << yaw << "]";

    ROS_INFO_STREAM(custom_solution.str());

    // Compare the result with service /compute_fk of the move_group node
    client = nh.serviceClient<moveit_msgs::GetPositionFK>("compute_fk");

    std::vector<std::string> link_names = joint_model_group->getLinkModelNames();

    // Create and prepare service request
    moveit_msgs::GetPositionFK movegroup_fk_service;
    movegroup_fk_service.request.header.frame_id = link_names[0];
    movegroup_fk_service.request.fk_link_names.push_back(link_names.back());
    moveit::core::robotStateToRobotStateMsg(robot_state, movegroup_fk_service.request.robot_state);

    // Call the service
    if(!client.call(movegroup_fk_service))
        ROS_ERROR("Cannot compute forward kinematics");

    // Convert orientation to RPY
    tf2::fromMsg(movegroup_fk_service.response.pose_stamped[0].pose.orientation, quaternion);

    tf2::Matrix3x3 rotation_matrix(quaternion);
    rotation_matrix.getRPY(roll, pitch, yaw);

    // Print solution to stdout
    std::ostringstream solution;

    solution << "Computed fk solution with move_group solver:" << std::endl;
    solution << "Position (XYZ): [";
    solution << movegroup_fk_service.response.pose_stamped[0].pose.position.x << ", "
             << movegroup_fk_service.response.pose_stamped[0].pose.position.y << ", "
             << movegroup_fk_service.response.pose_stamped[0].pose.position.z << "]"
             << std::endl;
    solution << "Orientation (RPY): [" << roll << ", " << pitch << ", " << yaw << "]";

    ROS_INFO_STREAM(solution.str());

    ros::shutdown();
    return 0;
}