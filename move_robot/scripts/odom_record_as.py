#! /usr/bin/env python

from tables import Float32Atom
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Point32
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from move_robot.msg import OdomRecordFeedback, OdomRecordResult, OdomRecordAction
import actionlib
from random import random

class ActionServer(object):
    
  _feedback = OdomRecordFeedback()
  _result   = OdomRecordResult()

  def __init__(self):
    # creates the action server
    self._as = actionlib.SimpleActionServer("/record_odom_as", OdomRecordAction, self.goal_callback, False)
    self._as.start()

  def odomCb(self, navmsgs_odometry):
    global odom

    odom = navmsgs_odometry

  def stateCb(self, stdmsgs_int32):
    global state

    global_sate = Int32()
    global_state = stdmsgs_int32
    state = global_state.data

  def goal_callback(self, goal):
    rospy.loginfo('called goal from action server /record_odom_as')
    success = True
    r = rospy.Rate(1)
    
    self._sub = rospy.Subscriber('/global_state', Int32, self.stateCb)        
    self._sub = rospy.Subscriber('/odom', Odometry, self.odomCb)        

    self._feedback.current_pos = [float,float]
    position     = Point32()
    position_arr = [Point32()]
    while True:

      # cancel operation
      if self._as.is_preempt_requested():
        rospy.loginfo('The goal has been cancelled/preempted')
        self._as.set_preempted()
        success = False
        break
    
      # Odometry Logic
      if state >= 0 or state < 3:
        self._feedback.status = "Going to the loading position"

      if state == 3:
        self._feedback.status = "Arrived to the loading position"
        break
       
      # action server will record the position of RB1 as a Point32 message each secod
      position = odom.pose.pose.position
      #position.z = odom.twist.twist.angular.z
      position_arr.append(position)      

      self._feedback.current_pos[0] = position.x
      self._feedback.current_pos[1] = position.y
      #self._feedback.current_vel = random()
      self._feedback.current_vel = odom.twist.twist.angular.z
      self._as.publish_feedback(self._feedback)
      r.sleep()

    # when the robot stops in the loading position the action returns all of the odometry values recorded on the terminal
    for i, pos in enumerate(position_arr):
        rospy.loginfo(f"pose[{i}] = {pos}")

    if success:
      self._result = Empty()
      self._as.set_succeeded(self._result)
      rospy.loginfo('Action Servier finished Successfully')
      
      
if __name__ == '__main__':
  state = -1
  odom = Odometry()
  rospy.init_node('odom_record_as')
  rospy.loginfo('STARTED ACTION SERVER NODE /odom_record_as')
  rospy.loginfo('PLEASE CALL THE SERVICE AS rosptopic call /record_odom_as/goal [TAB][TAB]')  
  ActionServer()
  rospy.spin()