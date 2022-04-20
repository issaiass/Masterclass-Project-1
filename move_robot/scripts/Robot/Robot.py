#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from time import sleep


class Robot(object):

    def LaserCb(self, laser_topic):
        self.laser = laser_topic.ranges
        self.laser_front = self.laser[360]
        self.laser_lt = self.laser[719]
        self.laser_rt = self.laser[0]
        rospy.loginfo('FRONT LASER = %.3f' %
                      (self.laser_front)) if self.__debug else _

    def __init__(self, linear=0.2, angular=0.0,
                 pub_cmd_vel_topic='/robot/cmd_vel', sub_laser_topic='/scan', debug=False, callback=None):
        self.__linear_x = linear
        self.__angular_z = angular
        self.__debug = debug
        self.__msg = Twist()
        self.__pub = rospy.Publisher(pub_cmd_vel_topic, Twist, queue_size=1)
        if callback == None:
            self.__sub = rospy.Subscriber(
                sub_laser_topic, LaserScan, self.LaserCb)
        else:
            self.__sub = rospy.Subscriber(sub_laser_topic, LaserScan, callback)

        rospy.Publisher(pub_cmd_vel_topic, Twist, queue_size=1)
        self.laser = 720*[float('inf')]
        self.laser_front = float('inf')
        self.laser_lt = float('inf')
        self.laser_rt = float('inf')
        rospy.loginfo("ROBOT CLASS INSTANTIATED") if self.__debug else _

    def move(self, linear, angular):
        rospy.loginfo('ROBOT MOVING AT LINEAR = %.3f, ANGULAR = %.3f' % (
            linear, angular)) if self.__debug else _
        self.__linear_x = linear
        self.__angular_z = angular
        self.__msg.linear.x = self.__linear_x
        self.__msg.angular.z = self.__angular_z
        self.__pub.publish(self.__msg)

    def stop(self):
        rospy.loginfo('ROBOT STOP') if self.__debug else _
        self.move(0, 0)

    def move_linear(self, linear):
        rospy.loginfo_once('ROBOT MOVE') if self.__debug else _
        self.move(linear, 0)

    def rotate(self, angular):
        rospy.loginfo('ROBOT ROTATE') if self.__debug else _
        self.move(0.0, angular)


if __name__ == '__main__':
    rospy.init_node('robot_class_test')
    rate = rospy.Rate(10)
    robot = Robot(debug=True)
    robot.stop()
    sleep(2)
    robot.move(0.1, 0.2)
    sleep(2)
    robot.move_linear(0.3)
    sleep(2)
    robot.rotate(0.4)
    sleep(2)
    rospy.loginfo('ROS SPIN')
    rospy.spin()
