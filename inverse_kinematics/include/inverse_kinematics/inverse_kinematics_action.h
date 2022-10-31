/* 
 * -------------------------------------------------------------------
 * This module has been developed as an exercise for Robotics course
 * @ UniSa.
 *
 * Title:   inverse_kinematics_action.h
 * Author:  Ugo Barbato
 * Date:    Oct 29, 2022
 *
 * This header declares the InverseKinematicsAction class and the
 * prototypes of the functions within it and all of its private attributes.
 *
 * -------------------------------------------------------------------
 */

#ifndef INCLUDE_INVERSE_KINEMATICS_INVERSE_KINEMATICS_ACTION_H
#define INCLUDE_INVERSE_KINEMATICS_INVERSE_KINEMATICS_ACTION_H

#include <ros/ros.h>
#include <kinematics_msgs/GetIKSolutionsAction.h>
#include <actionlib/server/simple_action_server.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

namespace inverse_kinematics {

    class InverseKinematicsAction {

        public:
            InverseKinematicsAction();
            ~InverseKinematicsAction();
        
        private:
            void ik_callback_(const kinematics_msgs::GetIKSolutionsGoalConstPtr & goal);
            bool isSolutionNew_(const std::vector<double> & solution) const;
            std::vector<double> generateSeedState_() const;
            void normalizeJointPositions_(std::vector<double> & solution) const;

            ros::NodeHandle nh_;

            actionlib::SimpleActionServer<kinematics_msgs::GetIKSolutionsAction> action_server_;

            std::vector<std::vector<double>> ik_solutions_;
            robot_model_loader::RobotModelLoaderConstPtr robot_model_loader_;
            robot_model::RobotModelConstPtr kinematic_model_;
            const robot_state::JointModelGroup *joint_model_group_;
    };

} // namespace inverse_kinematics

#endif // INCLUDE_INVERSE_KINEMATICS_INVERSE_KINEMATICS_ACTION_H