#! /usr/bin/env python

import rospy
from Action.ActionClient import ActionClient
from move_robot.msg import OdomRecordAction, OdomRecordGoal
from time import sleep

def actionClientFbCb(feedback):
    pass

rospy.init_node('record_odom_ac')
sleep(90)
ac_rb1 = ActionClient(ac_name='/record_odom_as', ac_action=OdomRecordAction)
ac_rb1._goal = OdomRecordGoal()
ac_rb1._fb_callback = None #actionClientFbCb
response = ac_rb1.send_goal()
print('[Result] State: %d'%(response))