#! /usr/bin/env python3

'''
.. module:: ontology:interface
   :platform: Unix
   :synopsis: Node to reset the ROSPLAN knowledge base.
.. moduleauthor:: SUBRAMANI SATHISH KUMAR

This node to reset the ROSPLAN knowledge base.

   This is the node which calls the rosplan_knowledge_base service with the pallner class methods. And it checks the solution whether the solution is concluded or not with the /reset_planning message. Based on the solution is correct or not the problem is defined again and the process continues.
   
Service:
    /reset_planning, solution checking  
'''

from time import sleep
import rospy
from erl2.srv import Reset, ResetResponse
from classes.planner import Planner


solution_correct=False
'''
bool: defines if the solution of the cluedo game was found or not 
'''

def reset(msg):
    '''
    This is the callback function to receive the /reset_planning request message and updates teh solution_correct variable.
    '''

    global solution_correct
    print("Check if simulation finishes")
    solution_correct=msg.finished
    return ResetResponse()


def replan():
    '''
    This is the function that creates the ROSPLAN. It defines the problem and generate the new plan. Then it parse them and dispatch them. It also clear the knowledge base.
    '''

    #1 Clear all
    res=Planner.clear_planner()
    print("Clear planner: "+str(res))

    #2 Add all the instances
    res=Planner.add_instance(name="sherlock", instance_type="robot")
    print("Add instance: "+str(res))
    Planner.add_instance(name="w1", instance_type="waypoint")
    Planner.add_instance(name="w2", instance_type="waypoint")
    Planner.add_instance(name="w3", instance_type="waypoint")
    Planner.add_instance(name="w4", instance_type="waypoint")
    Planner.add_instance(name="oracle_room", instance_type="waypoint")

    #3 Add the attributes
    Planner.add_attribute(attribute_name="not_can_check", key="", value="")
    Planner.add_attribute(attribute_name="ontology_updated", key="", value="")
    Planner.add_attribute(attribute_name="not_get_hint", key=["waypoint"], value=["w1"])
    Planner.add_attribute(attribute_name="not_get_hint", key=["waypoint"], value=["w2"])
    Planner.add_attribute(attribute_name="not_get_hint", key=["waypoint"], value=["w3"])
    Planner.add_attribute(attribute_name="not_get_hint", key=["waypoint"], value=["w4"])
    Planner.add_attribute(attribute_name="not_visited", key=["waypoint"], value=["w1"])
    Planner.add_attribute(attribute_name="not_visited", key=["waypoint"], value= ["w2"])
    Planner.add_attribute(attribute_name="not_visited", key=["waypoint"], value=["w3"])
    Planner.add_attribute(attribute_name="not_visited", key=["waypoint"], value=["w4"])
    Planner.add_attribute(attribute_name="not_visited", key=["waypoint"], value=["oracle_room"])
    print("All attributes are added")

    key=[]
    value=[]
    key.append("waypoint")
    value.append("oracle_room")
    key.append("robot")
    value.append("sherlock")
    Planner.add_attribute(attribute_name="in_position", key=key, value=value)

    print("Now add functions")

    ##4 Add functions 
    Planner.add_function(func_name="waypoints", key=[], value=[], func_value=0)
    Planner.add_function(func_name="cost", key=[], value=[], func_value=0)
    Planner.add_function(func_name="distance", key=["waypoint", "waypoint"], value=["w1", "w3"], func_value=5)
    Planner.add_function(func_name="distance", key=["waypoint", "waypoint"], value=["w3", "w1"], func_value=5)
    Planner.add_function(func_name="distance", key=["waypoint", "waypoint"], value=["w1", "w4"], func_value=5)
    Planner.add_function(func_name="distance", key=["waypoint", "waypoint"], value=["w4", "w1"], func_value=5)
    Planner.add_function(func_name="distance", key=["waypoint", "waypoint"], value=["w2", "w3"], func_value=5)
    Planner.add_function(func_name="distance", key=["waypoint", "waypoint"], value=["w3", "w2"], func_value=5)
    Planner.add_function(func_name="distance", key=["waypoint", "waypoint"], value=["w2", "w4"], func_value=5)
    Planner.add_function(func_name="distance", key=["waypoint", "waypoint"], value=["w4", "w2"], func_value=5)
    Planner.add_function(func_name="distance", key=["waypoint", "waypoint"], value=["w1", "oracle_room"], func_value=3.5)
    Planner.add_function(func_name="distance", key=["waypoint", "waypoint"], value=["w2", "oracle_room"], func_value=3.5)
    Planner.add_function(func_name="distance", key=["waypoint", "waypoint"], value=["w3", "oracle_room"], func_value=3.5)
    Planner.add_function(func_name="distance", key=["waypoint", "waypoint"], value=["w4", "oracle_room"], func_value=3.5)
    Planner.add_function(func_name="distance", key=["waypoint", "waypoint"], value=["oracle_room", "w1"], func_value=3.5)
    Planner.add_function(func_name="distance", key=["waypoint", "waypoint"], value=["oracle_room", "w2"], func_value=3.5)
    Planner.add_function(func_name="distance", key=["waypoint", "waypoint"], value=["oracle_room", "w3"], func_value=3.5)
    Planner.add_function(func_name="distance", key=["waypoint", "waypoint"], value=["oracle_room", "w4"], func_value=3.5)
    Planner.add_function(func_name="distance", key=["waypoint", "waypoint"], value=["w1", "w2"], func_value=7.25)
    Planner.add_function(func_name="distance", key=["waypoint", "waypoint"], value=["w3", "w4"], func_value=7.25)
    Planner.add_function(func_name="distance", key=["waypoint", "waypoint"], value=["w2", "w1"], func_value=7.25)
    Planner.add_function(func_name="distance", key=["waypoint", "waypoint"], value=["w4", "w3"], func_value=7.25)


    print("Add goal")
    #5 Add the goal
    Planner.add_goal(attribute_name="visited", key=["waypoint"], value=["w1"])
    Planner.add_goal(attribute_name="visited", key=["waypoint"], value=["w2"])
    Planner.add_goal(attribute_name="visited", key=["waypoint"], value=["w3"])
    Planner.add_goal(attribute_name="visited", key=["waypoint"], value=["w4"])
    Planner.add_goal(attribute_name="end_game", key=[], value=[])
    key_goal=[]
    value_goal=[]
    key_goal.append("waypoint")
    value_goal.append("oracle_room")
    key_goal.append("robot")
    value_goal.append("sherlock")
    Planner.add_goal(attribute_name="in_position", key=key_goal, value=value_goal)

    print("Generate the problem")

    #6 Generate the problem
    Planner.generate_problem()
    #7 Generate the plan
    Planner.generate_plan()
    #8 Parse the plan
    Planner.parse_plan()
    #9 Dispatch the plan and get solution
    solution=Planner.dispatch_plan()

    if solution.goal_achieved:
        print("A solution was found")
    else:
        print("No solution for the planner")

    return



def main():
    '''
    This is the function to initialise the reset_palnner node and all the function. 
    '''
    global solution_correct
    rospy.init_node('reset_planning')

    rospy.Service("/reset_planning", Reset, reset)

    while solution_correct==False:
        replan()
        sleep(1)

    rospy.spin()



if __name__ == '__main__':
    main()
