#! /usr/bin/env python3

'''
.. module:: myArmor.py
   :platform: Unix
   :synopsis: CLASS to update the ROSPLAN knowledge base.
.. moduleauthor:: SUBRAMANI SATHISH KUMAR

   This class updates the knowledge base. It is a way to interact with the ROSPLAN knowledge base. 

Service: 
    rosplan_knowledge_base/update service to update the knowledge base
    rosplan_plan_dispatcher/dispatch_plan service to dispatch the plan 
    rosplan_knowledge_base/clear service to clear the plan
    rosplan_parsing_interface/parse_plan service to parse the plan
    rosplan_problem_interface/problem_generation_server service to generate the problem
    rosplan_planner_interface/planning_server service to generate the plan 
'''

from rosplan_dispatch_msgs.srv import DispatchService
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService, KnowledgeUpdateServiceRequest
from diagnostic_msgs.msg import KeyValue
import rospy
from std_srvs.srv import Empty

knowledge_update=rospy.ServiceProxy('rosplan_knowledge_base/update', KnowledgeUpdateService)
'''
Initialize the service to update the knowledge base
'''
plan_dispatch=rospy.ServiceProxy('rosplan_plan_dispatcher/dispatch_plan', DispatchService) 
'''
Initialize the service to dispatch the plan 
'''
clear_knowledge=rospy.ServiceProxy('rosplan_knowledge_base/clear', Empty)
'''
Initialize the service to clear the plan
'''
parse_plan=rospy.ServiceProxy('rosplan_parsing_interface/parse_plan', Empty)
'''
Initialize the service to parse the plan
'''
problem_generation=rospy.ServiceProxy('rosplan_problem_interface/problem_generation_server', Empty)
'''
Initialize the service to generate the problem
'''
planning_server=rospy.ServiceProxy('rosplan_planner_interface/planning_server', Empty)
'''
Initialize the service to generate the plan
'''


class Planner(object):
    '''
    This is the class to update the ROSPlan knowledge base
    '''

    def __init__(self):
        '''
        Initialize the class
        '''
        print("\nClass initialized")    

    def add_instance(name, instance_type):
        '''
       Thisi is the function to add a new instance in the knowledge base
        '''

        rospy.wait_for_service('rosplan_knowledge_base/update')
        req=KnowledgeUpdateServiceRequest()
        req.knowledge.instance_name=name
        req.knowledge.instance_type=instance_type
        req.update_type=0
        req.knowledge.knowledge_type=0
        res=knowledge_update(req)
        return res

    def add_attribute(attribute_name, key, value, is_negative=False):
        '''
        This is the function to add a new attribute in the knowledge base
        '''

        rospy.wait_for_service('rosplan_knowledge_base/update')
        req=KnowledgeUpdateServiceRequest()
        req.knowledge.attribute_name=attribute_name
        req.knowledge.is_negative=is_negative
        req.update_type=0
        req.knowledge.knowledge_type=1
        assert len(key)==len(value), "The key elements and the value one do not correspond in add_attribute method"
        for i, j in zip(key, value):
            key_value=KeyValue()
            key_value.key=i
            key_value.value=j
            req.knowledge.values.append(key_value)
        res=knowledge_update(req)
        return res

    def add_goal(attribute_name, key, value, is_negative=False):
        '''
        This is the function to add a new goal in the knowledge base
        '''

        rospy.wait_for_service('rosplan_knowledge_base/update')
        req=KnowledgeUpdateServiceRequest()
        req.knowledge.attribute_name=attribute_name
        req.update_type=1
        req.knowledge.knowledge_type=1
        req.knowledge.is_negative=is_negative
        assert len(key)==len(value), "The key elements and the value one do not correspond in add_goal method"
        for i, j in zip(key, value):
            key_value=KeyValue()
            key_value.key=i
            key_value.value=j
            req.knowledge.values.append(key_value)
        res=knowledge_update(req)
        return res



    def add_function(func_name, key, value, func_value):
        '''
        This is the function to add a new function in the knowledge base
        '''

        req = KnowledgeUpdateServiceRequest()
        req.knowledge.attribute_name = func_name
        req.knowledge.knowledge_type = req.knowledge.FUNCTION
        assert len(key)==len(value), "The key elements and the value one do not correspond in add_goal method"
        for i, j in zip(key, value):
            key_value=KeyValue()
            key_value.key=i
            key_value.value=j
            req.knowledge.values.append(key_value)
        req.knowledge.function_value = func_value
        res=knowledge_update(req)
        return res


        
    def clear_planner():
        '''
        This is the function to remove the current model from the knowledge base
        '''

        rospy.wait_for_service('rosplan_knowledge_base/clear')
        res=clear_knowledge()
        return res

    def generate_problem():
        '''
        This is the function to generate the problem
        '''

        rospy.wait_for_service('rosplan_problem_interface/problem_generation_server')
        res=problem_generation()
        return res

    def generate_plan():
        '''
        This is the function to generate the plan
        '''

        rospy.wait_for_service('rosplan_planner_interface/planning_server')
        res=planning_server()
        return res

    def parse_plan():
        '''
        This is the function to parse the plan
        '''

        rospy.wait_for_service('rosplan_parsing_interface/parse_plan')
        res=parse_plan()
        return res

    def dispatch_plan():
        '''
        This is the function to dispatch the plan
        '''

        rospy.wait_for_service('rosplan_plan_dispatcher/dispatch_plan')
        res=plan_dispatch()
        return res
