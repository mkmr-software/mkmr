#!/usr/bin/env python
# coding=utf-8
import rospy
import os
import roslaunch.rlutil, roslaunch.parent
from mkmr_msgs.msg import *
from mkmr_srvs.srv import *
from std_msgs.msg import *

from mkmr_scripts.mkmr_base import MkmrBase

class LocationModule(MkmrBase):
    def __init__(self):
        super().__init__()
        rospy.init_node('location')

        self.location_server = rospy.Service("location", Location, self.execute)

    def execute(self, req:LocationResponse):
        try:
            while not rospy.is_shutdown():

                return True

        except Exception as e:
            self.consoleError("Error" + str(e))
            return False

def main():
    lm = LocationModule()
    try:
        rospy.spin()
    except rospy.exceptions.ROSInterruptException:
        print("")
    print("=== Shutting down launcher completed ===")

if __name__ == '__main__':
    main()
