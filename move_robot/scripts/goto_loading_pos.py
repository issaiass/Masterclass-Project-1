#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def LaserCb(laser_topic):
    global state

    laser_front = laser_topic.ranges[360]

    msg = Twist()
    if state == 0:  # advance up to the obstacle
        if laser_front < 0.3:
            rospy.loginfo('ROBOT STOP')
            msg.linear.x = 0
            msg.angular.z = 0
            pub.publish(msg)
            state = 1
            rospy.loginfo('goal_state = %d' % (state))
        else:
            rospy.loginfo_once('ROBOT MOVE')
            msg.linear.x = 0.3
            msg.angular.z = 0
            pub.publish(msg)

    if state == 1:
        msg.linear.x = 0
        msg.angular.z = -0.2
        rospy.loginfo('ROBOT ROTATE')
        pub.publish(msg)
        for i in range(100):
            rate.sleep()
        rospy.loginfo('ROBOT STOP')
        msg.angular.z = 0
        msg.linear.x = 0
        pub.publish(msg)
        rate.sleep()
        state = 2
        rospy.loginfo('goal_state = %d' % (state))

    if state == 2:
        rospy.loginfo('ROBOT MOVE TO LOADING CART')
        msg.linear.x = 0.3
        msg.angular.z = 0
        pub.publish(msg)
        for i in range(50):
            rate.sleep()
        rospy.loginfo('ROBOT STOP')
        msg.linear.x = 0
        msg.angular.z = 0
        pub.publish(msg)
        state = 3
        rospy.loginfo('goal_state = %d' % (state))


rospy.init_node('robot_pubsub')

state = 0
laser_front = 0

pub = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=1)
sub = rospy.Subscriber('/scan', LaserScan, LaserCb)

rospy.loginfo('goal_state = %d' % (state))
rate = rospy.Rate(10)
rospy.spin()
