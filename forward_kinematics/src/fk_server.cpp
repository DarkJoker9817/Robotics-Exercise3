/* 
 * -------------------------------------------------------------------
 * This module has been developed as an exercise for Robotics course
 * @ UniSa.
 *
 * Title:   fk_server.cpp
 * Author:  Ugo Barbato
 * Date:    Oct 28, 2022
 *
 * This node implements a server that receive a request for the 
 * computation of the forward kinematics of a robot using the 
 * GetFkSolution.srv service and send the response to the client
 * that requested the service.
 * -------------------------------------------------------------------
 */

#include <ros/ros.h>
#include <kinematics_msgs/GetFKSolution.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <eigen_conversions/eigen_msg.h>


bool computeFKSolution(kinematics_msgs::GetFKSolutionRequest &request, kinematics_msgs::GetFKSolutionResponse &response);

int main(int argc, char **argv) {
    ros::init(argc, argv, "fk_server");
    ros::NodeHandle nh;

    ros::ServiceServer server = nh.advertiseService("fk_solver", computeFKSolution);

    ROS_INFO("Started forward kinematics service");

    ros::spin();

    ros::shutdown();
    return 0;
}

bool computeFKSolution(kinematics_msgs::GetFKSolution::Request &request, kinematics_msgs::GetFKSolution::Response &response) {

    // Load the robot model
    robot_model_loader::RobotModelLoaderConstPtr robot_model_loader = std::shared_ptr<const robot_model_loader::RobotModelLoader>(new robot_model_loader::RobotModelLoader("robot_description"));

    // Get robot kinematic model
    robot_model::RobotModelConstPtr kinematic_model = robot_model_loader->getModel();

    // Create default robot state from kinematic model
    moveit::core::RobotState robot_state(kinematic_model);

    // Convert robot state message to RobotState
    moveit::core::robotStateMsgToRobotState(request.robot_state, robot_state);

    robot_state.updateLinkTransforms();

    // Get the planning group name from parameter server
    ros::NodeHandle nh;
    std::string planning_group_name;

    if(!nh.getParam("planning_group_name", planning_group_name)) {
        ROS_ERROR("'planning_group_name' is not defined on parameter server");
        return false;
    }

    // Get the planning group
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(planning_group_name);

    std::vector<std::string> link_names = joint_model_group->getLinkModelNames();

    // Compute FK and prepare the response message
    Eigen::Isometry3d end_effector_pose = robot_state.getGlobalLinkTransform(link_names.back());

    tf::poseEigenToMsg(end_effector_pose, response.end_effector_pose);

    return true;
}
