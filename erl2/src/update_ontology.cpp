/*************************************************************************************************************************//**
 * \file   update_ontology.cpp
 * 
 * description:
 *    This node implements the ROSPlan UpdateOntologyActionInterface: the corresponding behaviour for the update_ontology action in the PDDl file.
 * 
*****************************************************************************************************************************/

#include "erl2/update_ontology.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cmath>
#include <erl2/Update.h>

ros::ServiceClient client_update;

namespace KCL_rosplan {

	UpdateOntologyActionInterface::UpdateOntologyActionInterface(ros::NodeHandle &nh) {
			// here the initialization
	}

	/**
	 * bool UpdateOntologyActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	 * 
	 *This is the callback funciton to perform the update_ontology sends operation in the ontology_interface and waits for the execution.
	 **/
	bool UpdateOntologyActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
			// here the implementation of the action 
		erl2::Update req;
		req.request.req=true;
	    client_update.call(req);
        if (req.response.updated==true)
		{
            return true;
        }
        else
        {
            return false;
        }
	}
}

/** 
* int main(argc, argv)
*This is the main function, declare the UpdateOntologyActionInterface
**/
	int main(int argc, char **argv) {
		ros::init(argc, argv, "update_ontology", ros::init_options::AnonymousName);
		ros::NodeHandle nh("~");
		
        client_update= nh.serviceClient<erl2::Update>("/update_request");
		KCL_rosplan::UpdateOntologyActionInterface my_aci(nh);
		my_aci.runActionInterface();
		return 0;
	}
