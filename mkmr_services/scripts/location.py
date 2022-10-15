#!/usr/bin/env python
# coding=utf-8
import rospy
import os
import roslaunch.rlutil, roslaunch.parent
from mkmr_msgs.msg import *
from mkmr_srvs.srv import *
from std_msgs.msg import *

from mkmr_scripts.mkmr_base import MkmrBase
from rospy_message_converter import json_message_converter, message_converter

class LocationModule(MkmrBase):
    def __init__(self):
        super().__init__()
        rospy.init_node('location')
        self.locations_file_path =  self.config_folder + "/" + "locations.yaml"
        
        self.loadParamsFromYaml(self.locations_file_path, self.locations_prefix)
        
        self.location_server = rospy.Service("location", Location, self.execute)

        self.updateCFG()
        self.config_pub.publish(Bool(data = True))

    def execute(self, req:LocationRequest):
        try:
            while not rospy.is_shutdown():
                self.cur_locs = rospy.get_param(self.locations_prefix, [])
                if req.process == "add":
                    self.cur_locs.append(json_message_converter.convert_ros_message_to_json(req.location))
                    self.update()

                if req.process == "delete":
                    self.cur_locs.remove(json_message_converter.convert_ros_message_to_json(req.location))
                    self.update()

                return True

        except Exception as e:
            self.consoleError("Error " + str(e))
            return False

    def update(self):
        rospy.set_param(self.locations_prefix, self.cur_locs)
        self.saveParamsToYaml(self.locations_file_path, self.locations_prefix)
        self.updateCFG()
        self.config_pub.publish(Bool(data = True))

def main():
    lm = LocationModule()
    try:
        rospy.spin()
    except rospy.exceptions.ROSInterruptException:
        print("")
    print("=== Shutting down launcher completed ===")

if __name__ == '__main__':
    main()
