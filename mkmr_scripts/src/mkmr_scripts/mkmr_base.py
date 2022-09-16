#!/usr/bin/env python
# coding=utf-8

import os
import sys
import time
import rospy
import json
import rosparam
import yaml
import rospkg

from mkmr_msgs.msg import *
from mkmr_srvs.srv import *
from std_msgs.msg import *
from rospy_message_converter import json_message_converter, message_converter

class MkmrBase:
    def __init__(self):

        self.RID = os.getenv('MKMR_ROBOT_ID')
        if self.RID is None:
            self.RID = "NOTSET"

        self.CFG_TOPIC = "mkmr_config"

        self.CFG = None

        self.enabled_bool_msg = Bool()
        self.enabled_bool_msg.data = True

        self.disabled_bool_msg = Bool()
        self.disabled_bool_msg.data = False

        self.config_pub = rospy.Publisher(self.CFG_TOPIC, String, queue_size=10, latch=True)

        self.rospack = rospkg.RosPack()
        self.config_folder = self.rospack.get_path(os.getenv('MKMR_CONFIG_PKG')) 
        self.file_path =  self.config_folder + "/" + "config.yaml"

        self.loadParamsFromYaml(self.file_path)
        self.updateCFG()


    def loadParamsFromYaml(self, file_path):
        try:
            f = open(file_path, 'r')
            yamlfile = yaml.load(f)
            f.close()
            rosparam.upload_params('/' + self.RID + "/config/main", yamlfile)
        except Exception as e:
            self.consoleError("Error" + str(e))

    def updateCFG(self):
        self.CFG = rospy.get_param('/' + self.RID + "/config/main")
        self.setStaticConfigs()
        print(self.CFG)

    def getCFG(self):
        return self.CFG

    def setStaticConfigs(self):
        self.CFG["launch_directory"] = self.rospack.get_path("mkmr_navigation") + "/" + "launch"
        self.CFG["config_directory"] = self.config_folder

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
        color = self.getBoldStr(self.getCyanStr(s))
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
    def getBoldStr(s: str) -> str:
        return "\x1B[1m" + s.strip("\n") + "\x1B[m"

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
