/*************************************************************************************************************************//**
 * \file   gotoWaypoint.cpp
 * 
 * description:
 *          This node implements the ROSPlan GoToWaypointActionInterface: the corresponding behaviour for the go_to_waypoint action in the PDDl file.
*****************************************************************************************************************************/

#include "erl2/go_to_waypoint.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cmath>
#include <erl2/MoveAction.h>
#include <erl2/MoveActionResult.h>
#include <math.h>

#define _USE_MATH_DEFINES

//Declaring the x and y co ordinates and orientation
float pose_x;
float pose_y;
float orientation;



namespace KCL_rosplan {

	GoToWaypointActionInterface::GoToWaypointActionInterface(ros::NodeHandle &nh) {

	}

	/**
	 * bool GoToWaypointActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	 * 
	 * This is the callback function that sends a target to reach to the go_to point node via action request and wait for the node to execute.
	 **/
	bool GoToWaypointActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		std::cout << "Going from " << msg->parameters[0].value << " to " << msg->parameters[1].value << std::endl;
		
		actionlib::SimpleActionClient<erl2::MoveAction> ac("go_to_point_action", true);
		erl2::MoveAction goal;
		
		if(msg->parameters[1].value == "w1"){
		pose_x=-2.0;
		pose_y=0.0;
		orientation=M_PI;
		}
		else if (msg->parameters[1].value == "w2"){
		pose_x=2.0;
		pose_y=0.0;
		orientation=0.0;
		}
		else if (msg->parameters[1].value == "w3"){
		pose_x=0.0;
		pose_y=2.0;
		orientation=M_PI_2;
		}
		else if (msg->parameters[1].value == "w4"){
		pose_x=0.0;
		pose_y=-2.0;
		orientation=1.5*M_PI;
		}
		else { //Go to oracle room
		pose_x=0.0;
		pose_y=0.0;
		orientation=0.0;
		}

		goal.action_goal.goal.x_pos=pose_x;
		goal.action_goal.goal.y_pos=pose_y;
		goal.action_goal.goal.theta=orientation;
		
		ac.waitForServer();
		ac.sendGoal(goal.action_goal.goal);
		ac.waitForResult();

		erl2::MoveResultConstPtr res = ac.getResult();

		if (res->reached==true)
		{
			ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
			return true;
		}
		else
		{
			ROS_ERROR("The position reached by the robot is not correct");
			return false;	
		}
	}
}


/** 
* int main(argc, argv)
*
*This is the main function declare the GoToWaypointInterface. 
**/
	int main(int argc, char **argv) {
		ros::init(argc, argv, "goto_waypoint_action", ros::init_options::AnonymousName);
		ros::NodeHandle nh("~");
		
		KCL_rosplan::GoToWaypointActionInterface my_aci(nh);
		my_aci.runActionInterface();
		return 0;
	}
