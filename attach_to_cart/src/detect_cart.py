#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from more_itertools import locate
import numpy as np
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry


def getLegsIx(index_list):
    if len(index_list) <= 4:
        return    
    for i, val in enumerate(index_list):
        current = index_list[i]
        next = index_list[i+1]
        if abs(current - next) > 10: # if is over 10 we are on the next leg
            ix = i + 1 # get the index of the array to separate in two parts
            break
    leg1 = index_list[:ix] # leg1 array from 0 to the found index
    leg2 = index_list[ix:] # leg2 array from the found index to the end

    # get the mid values of the array
    leg1ix = len(leg1)//2
    leg1ix = index_list[leg1ix]
    leg2ix = len(leg2)//2+ix
    leg2ix = index_list[leg2ix]
    return np.array((leg1ix, leg2ix))

def getRanges(ranges, indexes):
    ix1 = indexes[0]
    ix2 = indexes[1]
    leg1d = ranges[ix1] 
    leg2d = ranges[ix2]
    return leg1d, leg2d

def getAngles(orientation_list, index):
    orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
    robot_yaw = euler_from_quaternion(orientation_list)[2]
    angle_min, angle_increment = -2.3561999797821045, 0.004363333340734243
    leg1ix, leg2ix = index
    angle1 = angle_min + leg1ix*angle_increment
    angle2 = angle_min + leg2ix*angle_increment
    return robot_yaw, angle1, angle2

def getCoords(legsranges, angles):
    robot_yaw, angle1, angle2 = angles
    leftd, rightd = legsranges
    d1, d2 = legsranges
    
    rads1 = abs(robot_yaw) - abs(angle1)
    p1 = d1*np.sin(rads1), -d1*np.cos(rads1)
    p1 = p1 + (0.0,)

    rads2 = abs(robot_yaw) + abs(angle2) - np.pi/2
    p2 = d2*np.cos(rads2), d2*np.sin(rads2)
    p2 = p2 + (0.0,)
    
    pmid = (np.array(p1) + np.array(p2))/2
    return tuple(pmid)


def OdometryCb(msg):
    global orientation
    orientation = msg.pose.pose.orientation
    

def LaserCb(laser_topic):
    global br, tf_state, p_laser_cart, p_odom_laser, p
    

    if tf_state == "wait_transform":
        frame2 = "robot_odom"
        frame1 = "robot_front_laser_base_link"
        if tfl.frameExists(frame1) and tfl.frameExists(frame2):
            t = tfl.getLatestCommonTime(frame2, frame1)
            p_odom_laser, q = tfl.lookupTransform(frame2, frame1, t)
            rospy.loginfo("transform_found, calculate robot_odom to cart_frame")
            tf_state = "pos_odom_cart"    
    if tf_state == "pos_odom_cart":
        ranges = laser_topic.ranges
        intensities = laser_topic.intensities
        index_list = list(locate(intensities, lambda a: a > 7000 and a != float('inf')))
        if len(index_list) > 4:
            legsix = getLegsIx(index_list) # indexes of leg1 and leg2
            legsranges = getRanges(ranges, legsix) # distances between base_laser to points
            orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w] # robot orientation
            angles = getAngles(orientation_list, legsix)
            p_laser_cart = getCoords(legsranges, angles)
            px = abs(p_odom_laser[0]) + abs(p_laser_cart[1])
            py = -(abs(p_odom_laser[1]) + abs(p_laser_cart[0]))
            pz = 0.0
            p = (px, py, pz)
            rospy.loginfo("cart_frame pose calculated, broadcasting TF")
            tf_state = "tf_broadcast"
    if tf_state == "tf_broadcast":
        #quat = quaternion_from_euler(0, 0, -1.5707)
        quat = quaternion_from_euler(0, 0, 0)        
        br.sendTransform(p, quat, rospy.Time.now(), "cart_frame", "robot_odom")
    rate.sleep()



rospy.init_node('cart_to_odom_node')
sub = rospy.Subscriber('/scan', LaserScan, LaserCb)
odom = rospy.Subscriber('/odom', Odometry, OdometryCb)
rate = rospy.Rate(10)
odometry = Odometry()
position = odometry.pose.pose.position
orientation = odometry.pose.pose.orientation
br = tf.TransformBroadcaster()
tfl = tf.TransformListener()    

tf_state = "wait_transform"
rospy.loginfo("waiting transform from robot_odom to robot_front_laser_base_link")            
while not rospy.is_shutdown():
    rate.sleep()
