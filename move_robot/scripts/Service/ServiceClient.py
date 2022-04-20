#! /usr/bin/env python

import rospy

class ServiceClient(object):

    def __init__(self, servicename, timeout, service):
        rospy.loginfo('Waiting for service ' + servicename + ' Ready')
        try:
          rospy.wait_for_service(servicename, timeout)
          self.__service_client = rospy.ServiceProxy(servicename, service)
          self.service_req = None
        except (rospy.ServiceException, rospy.ROSException):
          rospy.logerr("Service call failed")
          return

    def call(self):
        response = self.__service_client(self.service_req)
        return response