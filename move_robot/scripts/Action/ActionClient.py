#! /usr/bin/env python

import rospy
import actionlib

class ActionClient(object):

    def __init__(self, ac_name, ac_action):
        self._client = actionlib.SimpleActionClient(ac_name, ac_action)
        self._goal = None
        self._fb_callback = None
        self._client.wait_for_server()
        rospy.loginfo('ACTION CLIENT INSTANTIATED')

    def send_goal(self):  
        rospy.loginfo('SENDING GOAL')
        self._client.send_goal(self._goal, feedback_cb=self._fb_callback)
        rospy.loginfo('WAITING FOR RESULT')
        self._client.wait_for_result()
        rospy.loginfo('ACTION CLIENT FINISHED')        
        return self._client.get_state()

