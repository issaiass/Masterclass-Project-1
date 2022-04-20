#! /usr/bin/env python

import rospy
from Robot.Robot import Robot
from Service.ServiceServer import ServiceServer
from move_robot.srv import GoToLoading, GoToLoadingResponse
from std_msgs.msg import Int32

def RobotCb(laser_topic):
    global laser_front
    #rospy.loginfo(laser_topic.ranges[360])
    laser_front = laser_topic.ranges[360]
    pass

def GoToLoadingCb(request):
    rospy.loginfo('Application Service Called')
    # YOUR CODE HERE
    global state

    response = GoToLoadingResponse()
    response.complete = False

    while True:
        if state == 0:  # advance up to the obstacle
#            if laser_front < 0.3:    
            if laser_front < 0.35:    
                robot.stop()
                state = 1
                rospy.loginfo('goal_state = %d' % (state))
            else:
                robot.move_linear(0.3)

        if state == 1:
            robot.rotate(-0.1)
#            for i in range(100):
            for i in range(235): # 230
                rate.sleep()
            robot.stop()
            rate.sleep()
            state = 2
            rospy.loginfo('goal_state = %d' % (state))

        if state == 2:
            rospy.loginfo('ROBOT MOVE TO LOADING CART')
            robot.move_linear(0.3)
            for i in range(50):
#            for i in range(50): #48
                rate.sleep()
            robot.stop()
            state = 3
            rospy.loginfo('goal_state = %d' % (state))

        if state == 3:
            rospy.loginfo('Service Completion - Exiting')            
            break

    # END OF YOUR CODE
    response.complete = True
    return response


rospy.init_node('goto_loading_pos_service_server')
state = 0
laser_front = float('inf')
rate = rospy.Rate(10)
robot = Robot(debug=True, linear=0, angular=0, callback=RobotCb)  
service_server = ServiceServer(servicename='/goto_loading_pos', service=GoToLoading, callback=GoToLoadingCb)
state_pub = rospy.Publisher('/global_state', Int32, queue_size=1)
while not rospy.is_shutdown():
    state_pub.publish(state)
    rate.sleep()