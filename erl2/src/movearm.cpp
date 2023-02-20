/*************************************************************************************************************************//**
 * \file   movearm.cpp
 *
 * description:
 *    This node implements the ROSPlan MovearmActionInterface: the corresponding behaviour for the movearm action in the PDDl file.
*****************************************************************************************************************************/

#include "erl2/movearm.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/GetPlanningScene.h>
#include<unistd.h>


#define _USE_MATH_DEFINES

//Initialize the /check_state_validity ServiceClient
ros::ServiceClient validity_service;
//Initialize the /get_planning_scene ServiceClient
ros::ServiceClient planning_scene_service;

namespace KCL_rosplan {

	MovearmActionInterface::MovearmActionInterface(ros::NodeHandle &nh) {

	}

	/**
	 * bool MovearmActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	 * 
	 *This is the callback funciton that moves the robotic arm to reach the maeker and back to the iniial pose 
	 **/
	bool MovearmActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) { 

        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
        ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

        moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
        kinematic_state->setToDefaultValues();
        const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");
        moveit::planning_interface::MoveGroupInterface group("arm");
        const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

        group.setNamedTarget("initial_pose");
	    group.move(); 
        sleep(1);

        group.setNamedTarget("reach_target");
	    group.move(); 
        sleep(1);

        group.setNamedTarget("initial_pose");
	    group.move(); 
        sleep(1);
		
		ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
		return true;
	}

}

/** 
* int main(argc, argv)
* 
*This is the main function declare the MovearmActionInterface.
**/
	int main(int argc, char **argv) {
		ros::init(argc, argv, "movearm_action", ros::init_options::AnonymousName);
		ros::NodeHandle nh("~");
        validity_service=nh.serviceClient<moveit_msgs::GetStateValidity>("/check_state_validity");
        planning_scene_service=nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
        ros::AsyncSpinner spinner(1);
        spinner.start();
		KCL_rosplan::MovearmActionInterface my_aci(nh);
		my_aci.runActionInterface();

		return 0;
	}
