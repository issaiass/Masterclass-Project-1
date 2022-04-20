#! /usr/bin/env python

import rospy
from Service.ServiceClient import ServiceClient
from geometry_msgs.msg import Pose
from robotnik_base_hw_sim.srv import Pick, PickRequest
from std_msgs.msg import Int32

def stateCb(msg):
    global state

    global_sate = Int32()
    global_state = msg
    state = global_state.data

rospy.init_node('robot_pick_request')
rate = rospy.Rate(10)
rate.sleep()
global_state = Int32()
state = 0
sub = rospy.Subscriber('/global_state', Int32, stateCb)
while not rospy.is_shutdown():
    if state == 3:
        break
    rate.sleep()


try:
    client = ServiceClient(servicename='/elevator_fake_pickup_gazebo/pick', timeout=5, service=Pick)
    client.service_req = PickRequest()
    client.service_req.object_model = 'rb2_simple_cart'
    client.service_req.object_link = 'link_0'
    client.service_req.robot_model = 'robot'
    client.service_req.robot_link = 'robot_base_footprint'
    link_pose = Pose()
    link_pose.position.x = 0
    link_pose.position.y = 0
    link_pose.position.z = 0.2
    link_pose.orientation.x = 0
    link_pose.orientation.y = 0
    link_pose.orientation.z = 0 
    link_pose.orientation.w = 1.0  
    client.service_req.pose = link_pose
    response = client.call()
    rospy.loginfo(response)
    rate.sleep()
except (rospy.ServiceException, rospy.ROSException):
    exit()