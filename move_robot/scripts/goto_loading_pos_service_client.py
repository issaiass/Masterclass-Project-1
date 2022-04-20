#! /usr/bin/env python

import rospy
from  move_robot.srv import GoToLoading, GoToLoadingRequest
from Service.ServiceClient import ServiceClient
from time import sleep

rospy.init_node('goto_loadin_pos_service_client')
sleep(90)
rate = rospy.Rate(10)
rate.sleep()
try:
    client = ServiceClient(servicename='/goto_loading_pos', timeout=5, service=GoToLoading)
    client.service_req = GoToLoadingRequest()
    #client.service_req.<req> = <value>
    response = client.call()
    rospy.loginfo(response)
    rate.sleep()
except (rospy.ServiceException, rospy.ROSException):
    pass