#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from more_itertools import locate
import numpy as np
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from math import atan2
from geometry_msgs.msg import Pose
from robotnik_base_hw_sim.srv import Pick, PickRequest
from std_msgs.msg import Int32



def OdometryCb(msg):
    global x, y, theta

    rot = msg.pose.pose.orientation
    pos = msg.pose.pose.position
    
    x = pos.x
    y = pos.y
    theta = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])[2]


# Main Program
x = 0.0
y = 0.0
theta = 0.0


rospy.init_node('attach_cart_node')
odom = rospy.Subscriber('/odom', Odometry, OdometryCb)
cmd = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=1)
vel = Twist()
odometry = Odometry()
br = tf.TransformBroadcaster()
tfl = tf.TransformListener() 



rospy.loginfo("starting robot movement sequence")   
goal = Point()       
rate = rospy.Rate(10)

state = "tf_robot_odom_cart_frame"
move_to_goal = True
rotate_90 = False
move_backwards = False

while not rospy.is_shutdown():
    if state == "tf_robot_odom_cart_frame":
        frame1 = "robot_odom"
        frame2 = "cart_frame"
        if tfl.frameExists(frame1) and tfl.frameExists(frame2):
            t = tfl.getLatestCommonTime(frame1, frame2)
            goal_pos, goal_q = tfl.lookupTransform(frame1, frame2, t)
            #rospy.loginfo("%s->%s = %s" % (frame1, frame2, goal_pos))
            state = "publish_robot_odom_static_frame"

    if state == "publish_robot_odom_static_frame":
        frame1 = "robot_odom"
        frame2_static = "static_cart"
        br.sendTransform(goal_pos, goal_q, rospy.Time.now(), frame2_static, frame1)
        #rospy.loginfo("%s->%s = %s" %(frame1, frame2_static, goal_pos))
        state = "nav_goal_robot_base_footprint_static_frame"

    if state == "nav_goal_robot_base_footprint_static_frame":
        frame2_static = "static_cart"
        frame3 = "robot_base_footprint"    
        if tfl.frameExists(frame2_static) and tfl.frameExists(frame3):
            t = tfl.getLatestCommonTime(frame2_static, frame3)
            goal_pos, goal_q = tfl.lookupTransform(frame3, frame2_static, t)
            goal_theta = euler_from_quaternion(goal_q)[2]
            #rospy.loginfo("%s->%s = %s" %(frame3, frame2_static, goal_pos))
            state = "move_to_goal_static_cart"
            
    if state == "move_to_goal_static_cart":  
        if goal_pos[0] > 0.02 and move_to_goal:
            vel.linear.x = 0.1
            vel.angular.z = 0.0
            state = "tf_robot_odom_cart_frame"
        else:
            move_to_goal = False
            rotate_90 = True
            vel.linear.x = 0.0
            vel.angular.z = 0.0
            state = "rotate_90"
        cmd.publish(vel)
        
    if state == "rotate_90":
        if goal_theta + np.pi > 1.5707 and rotate_90:
            vel.linear.x = 0.0
            vel.angular.z = 0.3
            state = "tf_robot_odom_cart_frame"            
        else:
            rotate_90 = False
            move_backwards = True
            vel.linear.x = 0.0
            vel.angular.z = 0.0
            state = "move_backwards"
        cmd.publish(vel)
        
    if state == "move_backwards":
        print(goal_pos[0])
        if goal_pos[0] < 0.3 and move_backwards:
            vel.linear.x = -0.1
            vel.angular.z = 0.0
            state = "tf_robot_odom_cart_frame"
        else:
            move_backwards = False
            vel.linear.x = 0.0
            vel.angular.z = 0.0
            break
        cmd.publish(vel)

    rate.sleep()
    
    
try:
    servicename = '/elevator_fake_pickup_gazebo/pick'
    rospy.wait_for_service(servicename, 5)
    client = rospy.ServiceProxy(servicename, Pick)
    req = PickRequest()
    req.object_model = 'rb2_simple_cart'
    req.object_link = 'link_0'
    req.robot_model = 'robot'
    req.robot_link = 'robot_base_footprint'
    link_pose = Pose()
    link_pose.position.x = 0
    link_pose.position.y = 0
    link_pose.position.z = 0.2
    link_pose.orientation.x = 0
    link_pose.orientation.y = 0
    link_pose.orientation.z = 0 
    link_pose.orientation.w = 1.0  
    req.pose = link_pose
    response = client(req)
    rospy.loginfo(response)
    rate.sleep()
except (rospy.ServiceException, rospy.ROSException):
    exit()    
