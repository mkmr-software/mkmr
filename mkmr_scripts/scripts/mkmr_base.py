#!/usr/bin/env python
# coding=utf-8

import os
import sys
import time
import rospy
import json
from mkmr_msgs.msg import *
from mkmr_srvs.srv import *
from std_msgs.msg import *


class MkmrBase:
    def __init__(self):

        self.RID = os.getenv('SMR_ROBOT_ID')
        if self.RID is None:
            self.RID = "NOTSET"

        self.CFG_TOPIC = "mkmr_config"

        self.CFG = None

        self.enabled_bool_msg = Bool()
        self.enabled_bool_msg.data = True

        self.disabled_bool_msg = Bool()
        self.disabled_bool_msg.data = False

    def callRosService(self, name: str, msg_type: type, msg, print_info: bool = True) -> bool:
        rospy.wait_for_service(name)
        data_str = str(msg)
        resp = False
        try:
            ros_service_proxy = rospy.ServiceProxy(name, msg_type)
            resp = ros_service_proxy(msg)
            if print_info:
                self.consoleCyan("Calling " + name + " with data: " + data_str)
        except rospy.ServiceException as e:
            self.consoleError(name + " call failed: " + str(e.args))
        return resp

    def consoleCyan(self, s: str):
        color = self.getCyanStr(s)
        pfx = self.getDateTime()
        if rospy.core.is_initialized():
            rospy.loginfo(pfx + color)
        else:
            print(pfx + color)

    def consoleError(self, s: str):
        color = self.getCyanStr(s)
        pfx = self.getDateTime()
        if rospy.core.is_initialized():
            rospy.loginfo(pfx + color)
        else:
            print(pfx + color)

    def getLocalTimeString(self) -> str:
        # (tm_year,tm_mon,tm_mday,tm_hour,tm_min, tm_sec,tm_wday,tm_yday,tm_isdst)
        self.l_time = time.localtime()
        return str(self.l_time[0]) + "/" + \
               str(self.l_time[1]).zfill(2) + "/" + \
               str(self.l_time[2]).zfill(2) + " " + \
               str(self.l_time[3]).zfill(2) + ":" + \
               str(self.l_time[4]).zfill(2) + ":" + \
               str(self.l_time[5]).zfill(2)

    def getDateTime(self) -> str:
        return "[" + self.getLocalTimeString() + "]"

    @staticmethod
    def getCyanStr(s: str) -> str:
        return "\x1B[36m" + s.strip("\n") + "\x1B[m"

    @staticmethod
    def getRedStr(s: str) -> str:
        return "\x1B[31m" + s.strip("\n") + "\x1B[m"

    @staticmethod
    def bytesToStr(val: bytes) -> str:
        return val.decode("utf-8")

    @staticmethod
    def strToBytes(val: str) -> bytes:
        return bytes(val, encoding="utf-8")

    @staticmethod
    def strToJson(val: str) -> json:
        return json.loads(val)

    @staticmethod
    def jsonStrToDict(val: str) -> dict:
        return json.loads(val)

    @staticmethod
    def createDirectoryIfNotExists(dirName):
        if not os.path.exists(dirName):
            os.mkdir(dirName)

    @staticmethod
    def createFile(fileName):
        os.system("touch " +  fileName)           
        