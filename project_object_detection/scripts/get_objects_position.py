#!/usr/bin/env python

import rospy
import json
from visualization_msgs.msg import Marker

class SurfaceExtractor():
    def __init__(self, sub_name='/surface_objects', obj_names=[]):
      rospy.init_node('surface_extractor')
      self.sub = rospy.Subscriber(sub_name, Marker, self.MarkerCb)
      self.obj_names = obj_names
      self.msg = 3*[str()]
      self.string = 3*[str()]
      self.rate = rospy.Rate(10)


    def MarkerCb(self, marker):
#      rospy.loginfo(marker)    
      data = {"marker"      : marker.ns, 
              "position"    : {"x": marker.pose.position.x, "y": marker.pose.position.y, "z": marker.pose.position.z},
              "orientation" : {"w": marker.pose.orientation.w, "x": marker.pose.orientation.x, "y": marker.pose.orientation.y, "z": marker.pose.orientation.z}
             }
      data = json.dumps(data)

      message = "'" + marker.ns + "'" + ": " + str(marker.pose)


      if marker.ns == self.obj_names[0] and marker.pose.position.x > 0:
        self.msg[0] = data
        self.string[0] = "{" + message + ","
        
      if marker.ns == self.obj_names[1] and marker.pose.position.x > 0:
        self.msg[1] = data
        self.string[1] = message + ","
        
      if marker.ns == self.obj_names[2] and marker.pose.position.x > 0:
        self.msg[2] = data
        self.string[2] = message + "}"
       
      if str() not in self.msg:
        #rospy.loginfo(self.msg[0])
        #rospy.loginfo(self.msg[1])
        #rospy.loginfo(self.msg[2])
        rospy.loginfo(self.string[0]+self.string[1]+self.string[2])
        
        self.msg = 3*[str()]
        self.string = 3*[str()]
        self.valid = 3*[False]
        rospy.loginfo(' ')



        
        
       



if __name__ == '__main__':
    sub_name, obj_names = '/surface_objects', ['surface_1_object_0_axes', 'surface_1_object_1_axes', 'surface_1_object_2_axes']
    se = SurfaceExtractor(sub_name=sub_name, obj_names=obj_names)
    while not rospy.is_shutdown():
        se.rate.sleep()
