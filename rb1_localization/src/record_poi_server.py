#! /usr/bin/env python

import yaml
import rospy
from rb1_localization.srv import SavePOI, SavePOIResponse
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

def OdometryCb(nav_msgs_odom):
    global pose
    pose = nav_msgs_odom.pose.pose


def SavePOICb(stringreq):
    global pose1, pose2
    
    print("Request Data==> command = " + stringreq.label)
    label = stringreq.label
    response = SavePOIResponse()
    response.success = False
    px = pose.position.x
    py = pose.position.y
    pz = pose.position.z
    ox = pose.orientation.x
    oy = pose.orientation.y
    oz = pose.orientation.z
    ow = pose.orientation.w    
    if label == "initial_position":
        rospy.loginfo('INIT_POSITION CALLED')
        data = {"orientation":{"x":ox,"y":oy,"z":oz,"w":ow}, "position":{"x":px, "y":py, "z":pz}}
        pose1 = {"initial_position": data}   
        rospy.loginfo(pose1)  
        response.success = True
    if label == "loading_position":
        rospy.loginfo('LOADING POSITION CALLED')
        data = {"orientation":{"x":ox,"y":oy,"z":oz,"w":ow}, "position":{"x":px, "y":py, "z":pz}}
        pose2 = {"loading_position": data}  
        rospy.loginfo(pose2)       
        response.success = True        
    if label == "end":
        rospy.loginfo('END LOGGING DESTINATIONS')        
        file = open("poi.yaml", "w")
        yaml.dump(pose1, file)
        yaml.dump(pose2, file)
        file.close()
        response.success = True
    return response


if __name__ == "__main__":
    HZ = 10
    serviceservername = '/save_poi'
    subtopicname = '/odom'
    rospy.init_node('poi_server')
    pose = Pose()
    pose1 = Pose()
    pose2 = Pose()
    rate = rospy.Rate(HZ)
    rospy.loginfo('STARTING SERVER ' + serviceservername)
    service_server = rospy.Service(serviceservername, SavePOI, SavePOICb)
    sub = rospy.Subscriber(subtopicname, Odometry, OdometryCb)
    rospy.spin()  # maintain the service open.
