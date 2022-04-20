#! /usr/bin/env python

# References
# https://github.com/gbiggs/crane_plus_arm/blob/master/crane_plus_gripper/script/command_gripper.py
# https://answers.ros.org/question/240723/fail-aborted-no-motion-plan-found-no-execution-attempted/
# http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
# https://answers.ros.org/question/255792/change-planner-in-moveit-python/
# https://github.com/ros-planning/moveit_tutorials/blob/kinetic-devel/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py
# http://docs.ros.org/en/jade/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
# https://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/steps/index.htm
# https://www.theconstructsim.com/ros-qa-how-to-convert-quaternions-to-euler-angles/
# https://github.com/mikeferguson/moveit_python/blob/ros1/src/moveit_python/pick_place_interface.py


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from math import pi

#import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal



class Grippermover:
  def __init__(self, group_name='gripper', display_path_topic='/move_group/display_planned_path'):
    self.scene         = moveit_commander.PlanningSceneInterface()
    self.group         = moveit_commander.MoveGroupCommander(group_name)
    self.pose          = geometry_msgs.msg.Pose() 
    self.display_traj_pub = rospy.Publisher(display_path_topic, moveit_msgs.msg.DisplayTrajectory, queue_size=1)
    self.group.clear_pose_targets()    
    
    #self.client = actionlib.SimpleActionClient('/gripper_controller/gripper_cmd', GripperCommandAction)
    #self.client.wait_for_server()

 # def GripperCb(feedback):
 #   print('[INFO] Position: {}'.format(feedback.position))
 #   print('[INFO] Effort: {}'.format(feedback.effort))
 #   print('[INFO] Stalled: {}'.format(feedback.stalled))
 #   print('[INFO] Reached goal: {}'.format(feedback.reached_goal))
    
  def operate(self, goal=[0,0,0,0], wait=True):
    group = self.group
        
    group.set_joint_value_target([goal, -goal, -goal, goal])
    group.plan()
    rospy.sleep(2)
    group.go(wait=wait)
    
#  def sendGoal(goal):
#    self.client.send_goal(goal, feedback_cb=self.GripperCb)


  def getJointValues(self):
    return self.group.get_current_joint_values()    
    
    

class ARMmover:

  def __init__(self, group_name='arm', display_path_topic='/move_group/display_planned_path'):
    self.robot         = moveit_commander.RobotCommander()
    self.scene         = moveit_commander.PlanningSceneInterface()
    self.group         = moveit_commander.MoveGroupCommander(group_name)
    self.pose          = geometry_msgs.msg.Pose() 
    self.display_traj_pub = rospy.Publisher(display_path_topic, moveit_msgs.msg.DisplayTrajectory, queue_size=1)
    self.group.clear_pose_targets()    
        
  def getPlanningFrame(self):
    return self.group.get_planning_frame()
  
  def getEELink(self):
    return self.group.get_end_effector_link()

  def getGroupNames(self):
    return self.robot.get_group_names()
  
  def getJointValues(self):
    return self.group.get_current_joint_values()
  
  def getPose(self):
    return self.group.get_current_pose()
    
  def getState(self):
    return self.robot.get_current_state()    
    
  def display_trajectory(self, plan):
    traj_pub = self.display_traj_pub
    disp_traj = moveit_msgs.msg.DisplayTrajectory()
    disp_traj.trajectory_start = self.getState()
    disp_traj.trajectory.append(plan)
    traj_pub.publish(disp_traj);
    

  def goPosName(self, name, wait=True):
    group = self.group
    
    group.set_named_target(name)
    group.plan()
    rospy.sleep(3)
    group.go(wait=wait)
    group.stop()
    group.clear_pose_targets()    

  def goJointValues(self, joint_values, wait=True):
    group = self.group
    
    group.set_joint_value_target(joint_values)    
    group.plan()
    rospy.sleep(3)
    group.go(wait=wait)
    group.stop()
    group.clear_pose_targets()
    
  def goPose(self, pose, wait=True):
    group = self.group
    
    group.set_pose_target(pose)
    group.plan()
    rospy.sleep(3)
    group.go(wait=wait)
    group.stop()
    group.clear_pose_targets()        

  def grab(self, object_name):
    self.group.pick(object_name)

  def release(self, object_name):
    self.group.place(object_name)

  def shutdown(self):
    moveit_commander.roscpp_shutdown()
    
def main():
  rospy.init_node('arm_mover')
  rate = rospy.Rate(10)  
  rospy.Time(0)
  mover = ARMmover()
  gripper = Grippermover()
  #print()
  #print("Reference frame: %s" % (mover.getPlanningFrame()))
  #print("End effector: %s" % (mover.getEELink()))
  #print("Robot Groups: %s" % (mover.getGroupNames()))
  #print("Current Joint Values: %s" % (mover.getJointValues()))
  #print("Current Pose: %s" % (mover.getPose()))
  #print("Robot State: %s" % (mover.getState()))
  #print() 

  mover.goPosName('home')

  
  #rospy.Time(0)  
  #joint_values = mover.getJointValues()
  #joint_values[0] = 0.25
  #joint_values[1] = -1.13
  #joint_values[2] = 0.76
  #joint_values[3] = -1.13
  #joint_values[4] = -1.76
  #joint_values[5] = 1.53
  #mover.goJointValues(joint_values)
  #goal = geometry_msgs.msg.Pose()
  #goal.orientation.w = 1.0
  #goal.position.x = 0.4
  #goal.position.y = 0.1
  #goal.position.z = 0.4
  #mover.goPose(goal)
  
  # 1 - Go to the position of the object to grasp, with an offset of 5-10cm in the z axis.
  roll, pitch, yaw = -pi/2, 0, -pi/2
  quat = quaternion_from_euler (roll, pitch,yaw)
  goal = Pose(Point(0.38, 0.155, 0.4), Quaternion(quat[0], quat[1], quat[2], quat[3]))
  mover.goPose(goal)
  rate.sleep()
  
  # 2 - Open gripper.
#  goal = GripperCommandGoal()
#  goal.command.position = 0.00 #-0.57
  gripper.operate(0.0)
  rate.sleep()
  
  # 3 - Approach / Move down.
  goal = mover.getPose().pose
  goal.position.z -= 0.05
  mover.goPose(goal)  
  rate.sleep()
  
  # 4. Close gripper.
#  goal.command.position = -0.57
#  gripper.sendGoal(goal)
  gripper.operate(-0.6)
  rate.sleep()
  mover.grab('demo_cube')

  # 5. Retreat / Move back up.  
  mover.goPosName('end')  
  rate.sleep()

  # 6. Release
  gripper.operate(0.0)
  rate.sleep()
  mover.release('demo_cube')  

  rospy.sleep(7)
#  rospy.spin()
  mover.shutdown()
  
if __name__ == '__main__':
  sys.exit(main())
  
  
  
