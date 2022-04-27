#!/usr/bin/env python

import re
import json
import rospy
from get_objects_position import SurfaceExtractor
from geometry_msgs.msg import Pose, Point

if __name__ == '__main__':
    sub_name, obj_names = '/surface_objects', ['surface_1_object_0_axes', 'surface_1_object_1_axes', 'surface_1_object_2_axes']
    se = SurfaceExtractor(sub_name=sub_name, obj_names=obj_names)
    pose = Pose()
    p = Point()
    pub = rospy.Publisher('/graspable_object_pose', Point, queue_size=10)
#    pub = rospy.Publisher('/graspable_object_pose', Pose, queue_size=10)
    while not rospy.is_shutdown():
      if se.msg[0] != str():
        data = json.loads(se.msg[0])
               
        pose.position.x = data['position']['x']
        pose.position.y = data['position']['y']
        pose.position.z = data['position']['z']
       
        pose.orientation.w = data['orientation']['w']
        pose.orientation.x = data['orientation']['x']
        pose.orientation.y = data['orientation']['y']
        pose.orientation.z = data['orientation']['z']        

        p = pose.position

        pub.publish(p)
#        pub.publish(pose)
