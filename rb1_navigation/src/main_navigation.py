#! /usr/bin/env python

import rospy
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback
from rb1_navigation.srv import GoToPoint, GoToPointResponse
from geometry_msgs.msg import Point, Quaternion



PENDING = 0
ACTIVE = 1
DONE = 2
WARN = 3
ERROR = 4


def GoToPointCb(gotopointreq):
    global client
    
    response = GoToPointResponse()
    pos = Point()
    ori = Quaternion()
    goal = MoveBaseGoal()


    lbls = ['initial_position', 'loading_position']


    label = gotopointreq.label    
    response.message = 'NOK'
    for i in range(2):
        if label == lbls[i]:
            new_pose = rospy.get_param('/' + lbls[i])
            pos = new_pose['position']
            ori = new_pose['orientation']  
            goal = MoveBaseGoal()       
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.pose.position.x = pos['x']
            goal.target_pose.pose.position.y = pos['y']
            goal.target_pose.pose.position.z = pos['z']            
            goal.target_pose.pose.orientation.w = ori['w']            
            goal.target_pose.pose.orientation.x = ori['x']
            goal.target_pose.pose.orientation.y = ori['y']
            goal.target_pose.pose.orientation.z = ori['z']            
            client.send_goal(goal, feedback_cb=None)
            status = client.get_state()
            rospy.loginfo('Got new plan')
            while status < DONE:
                rate.sleep()
                status = client.get_state()
            rospy.loginfo('Goal reached')
            response.message = 'OK'
            break
    return response


if __name__ == '__main__':
    nodename = 'go_to_point_serviceserver'
    serviceservername = '/go_to_point'
    actionservername = '/move_base'

    rospy.init_node(nodename) 
    # waits until the service server is up and running
    rospy.loginfo('STARTING SERVICE SERVER ' + serviceservername)
    serviceserver = rospy.Service(serviceservername, GoToPoint , GoToPointCb)
    rospy.loginfo('SERVICE SERVER ' + serviceservername + ' STARTED')    

    # waits until the action server is up and running

    client = actionlib.SimpleActionClient(actionservername, MoveBaseAction)    
    client.wait_for_server()
    rospy.loginfo('ACTION SERVER ' + actionservername + ' STARTED')
    
    # wait system rate    
    rate = rospy.Rate(10)
    rate.sleep()
    
    # spin and wait events
    rospy.spin()
