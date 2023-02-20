#! /usr/bin/env python3

"""
.. module:: go_to_point
   :platform: Unix
   :synopsis: Node for implementing the robot movement.
.. moduleauthor:: SUBRAMANI SATHISH KUMAR

Node for implementing the robot movement.

    This node implement the movement of the robot as well orient itself to the goal postion. Based on the goal received from the gotowaypoint node via ActionGoal the robot aligns to the direction set the desired velocity and move towards the target and reach the target postion. 
    
Subscriber:
        /odom, subscribe to the odometry 
Publisher:
	/cmd_vel, send the desired velocity to move the robot 
Action:
    action server to receive the coordinates of the place to reach 
"""

import rospy
import actionlib
from erl2.msg import MoveAction, MoveActionResult
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
import math



# robot state variables
position_ = Point()
yaw_ = 0
position_ = 0
state_ = 0
pub_ = None

ang_coef=2
''' 
int: coefficient to manage the angular velocity of the robot during the motion
'''
lin_coef=1.7
''' 
int: coefficient to manage the linear velocity of the robot during the motion
'''

# parameters for control
yaw_precision_ = math.pi / 9  
# +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  
# +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = -3.0 
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6

server=None


## Callback to get the current odom position of the robot 
def clbk_odom(msg):
    '''
    This is the function obtains the odometry of the robot.            
    '''
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def change_state(state):
    '''
    This is the function to change the state of the FSM in the go_to_point function
            
    '''
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)


def normalize_angle(angle):
    '''
    This is the funciton to normalize the angle of the rotation that the robot must perform.
    '''
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle
    

def fix_yaw(des_pos):
    '''
    This is the funciton to move the robot towards the target.
    '''
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    twist_msg.angular.z=twist_msg.angular.z*ang_coef
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(1)


def go_straight_ahead(des_pos):
    '''
    This is the funciton to move the robot in the straight direction based on x and y position.
    '''
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.3
        if twist_msg.linear.x > ub_d:
           twist_msg.linear.x = ub_d

        twist_msg.angular.z = kp_a*err_yaw

        twist_msg.angular.z=twist_msg.angular.z*ang_coef
        twist_msg.linear.x=twist_msg.linear.x*lin_coef
        pub_.publish(twist_msg)
    else: # state change conditions
        #print ('Position error: [%s]' % err_pos)
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(0)

def fix_final_yaw(des_yaw):
    '''
    This is the function to orient robot towards the desired orientation.
    '''

    err_yaw = normalize_angle(des_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a

    twist_msg.angular.z=twist_msg.angular.z*ang_coef
    twist_msg.linear.x=twist_msg.linear.x*lin_coef
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(3)
        
def done():
    '''
    This is the funciton to bring the robot to halt.
    '''

    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub_.publish(twist_msg)


def go_to_point(goal):
    """
    This is the function which receives the goal to reach, and the corresponding function are called in this function to move the robot to goal postion.  
    """
    global server, state_
    result=MoveActionResult()
    desired_position = Point()
    desired_position.x = goal.x_pos
    desired_position.y = goal.y_pos
    des_yaw = goal.theta

    change_state(0)
    while True:
        
        if server.is_preempt_requested():
            state_=3

        if state_ == 0:
            fix_yaw(desired_position)
        elif state_ == 1:
            go_straight_ahead(desired_position)
        elif state_ == 2:
            fix_final_yaw(des_yaw)
        elif state_ == 3:
            done()
            break
    result.result.reached=True
    server.set_succeeded(result.result)
    


## In the main there are the initialization of the publisher, th subscriber and the SimpleActionServer
def main():
    '''
    This is the main fucniton of the go_to_point module intializes the module. 
    '''
    global pub_, server
    rospy.init_node('go_to_point')
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    server = actionlib.SimpleActionServer('go_to_point_action', MoveAction, execute_cb = go_to_point, auto_start=False)
    server.start()
    
    rospy.spin()


if __name__ == '__main__':
    main()
