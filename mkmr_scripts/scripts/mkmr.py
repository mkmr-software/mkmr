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
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

class MkmrModule(MkmrBase):
    def __init__(self):
        super().__init__()
        rospy.init_node('mkmr')

        self.mkmr_msg = Mkmr()

        self.mkmr_pub = rospy.Publisher("mkmr" , Mkmr, queue_size=10)

        self.loc_pos_sub = rospy.Subscriber(
            "pose", PoseWithCovarianceStamped, self.localizationPoseCb)

        self.mapping_active_sub = rospy.Subscriber(
            "mapping_active", Bool, self.mappingActiveCb)

        self.localization_active_sub = rospy.Subscriber(
            "localization_active", Bool, self.localizationActiveCb)

        self.current_map_sub = rospy.Subscriber(
            "current_map", String, self.currentMapCb)

        self.publish_mkmr_timer = rospy.Timer(rospy.Duration(1.0 / 5.0), self.publishMkmrTimerCb)

    def publishMkmrTimerCb(self, timer):
        self.mkmr_pub.publish(self.mkmr_msg)

    def localizationPoseCb(self, msg: PoseWithCovarianceStamped):
        self.mkmr_msg.px = round(msg.pose.pose.position.x, 2)
        self.mkmr_msg.py = round(msg.pose.pose.position.y, 2)
        self.mkmr_msg.yaw = round(self.getYawDegFromQuatZw(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w), 2)

    def mappingActiveCb(self, msg: Bool):
        if msg.data:
            self.mkmr_msg.mapping_active = True
            self.mkmr_msg.localization_active = False
            self.mkmr_msg.map = ""
            self.mkmr_msg.floor = ""
            self.mkmr_msg.px = 0
            self.mkmr_msg.py = 0
            self.mkmr_msg.yaw = 0

    def localizationActiveCb(self, msg: Bool):
        if msg.data:
            self.mkmr_msg.mapping_active = False
            self.mkmr_msg.localization_active = True
            self.mkmr_msg.px = 0
            self.mkmr_msg.py = 0
            self.mkmr_msg.yaw = 0


    def currentMapCb(self, msg: String):
        try:
            map, floor = msg.data.split("=")[1].split("_")
        except Exception as e:
            self.consoleError("Error " + str(e))
            return

        self.mkmr_msg.map = map
        self.mkmr_msg.floor = floor







def main():
    mkmr = MkmrModule()
    try:
        rospy.spin()
    except rospy.exceptions.ROSInterruptException:
        print("")
    print("=== Shutting down mkmr completed ===")

if __name__ == '__main__':
    main()
