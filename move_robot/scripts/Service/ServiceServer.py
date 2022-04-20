#! /usr/bin/env python

import rospy
from std_srvs.srv import EmptyResponse

class ServiceServer(object):

    def ServiceServerCb(self, request):
        rospy.loginfo('Internal class Service Called')
        # YOUR CODE HERE

        # END OF YOUR CODE
        response = EmptyResponse()
        return response

    def __init__(self, servicename, service, callback):
        if callback == None:
            self.__service_server = rospy.Service(servicename, service, self.ServiceServerCb)
        else:
            self.__service_server = rospy.Service(servicename, service, callback)
        rospy.loginfo('Service ' + servicename + ' Ready')