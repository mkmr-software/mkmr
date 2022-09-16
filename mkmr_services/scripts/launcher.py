#!/usr/bin/env python
# coding=utf-8
import rospy
import os
import math
import roslaunch.rlutil, roslaunch.parent
from mkmr_msgs.msg import *
from mkmr_srvs.srv import *
from std_msgs.msg import *

from mkmr_scripts.mkmr_base import MkmrBase

class LauncherModule(MkmrBase):
    def __init__(self):
        super().__init__()
        rospy.init_node('launcher')
  
        self.launcher_server = rospy.Service("nav_launcher", Launcher, self.execute)

        req = LauncherRequest()
        req.map = self.CFG["map"]
        req.floor = self.CFG["floor"]
        self.setCurrentArgs(req)

        self.mapping_active_pub = rospy.Publisher("mapping_active", Bool, queue_size=1, latch=True)
        self.localization_active_pub = rospy.Publisher("localization_active", Bool, queue_size=1, latch=True)

        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

        self.launchLocalization()  

    def execute(self, req:LauncherRequest):
        try:
            while not rospy.is_shutdown():
                self.setCurrentArgs(req)

                if req.mapping:
                    self.parent.shutdown()
                    self.launchMapping()
                    return True

                else:
                    self.parent.shutdown()

                    if req.map == "" or req.floor == "" :
                        return False

                    self.launchLocalization()
                    return True

        except Exception as e:
            self.consoleError("Error" + str(e))
            return False


    def setCurrentArgs(self, req):
        self.robot_id_arg = 'robot_id:=' + self.RID
        self.is_sim_arg = "is_sim:=" + str(self.CFG["is_sim"])

        self.cur_px_arg = "initial_pose_x:=" + str(req.init_px)
        self.cur_py_arg = "initial_pose_y:=" + str(req.init_py)
        self.cur_yaw_arg = "initial_yaw:=" + str(math.radians(req.init_yaw_deg))
        self.default_map_arg = "default_map:=" + req.map + "_" + req.floor
        self.map_arg = "map_file:=" + self.CFG["config_directory"] + "/maps/" + req.map + "_" + req.floor + ".yaml"
        self.pbstream_file_arg = "pbstream_file:=" + self.CFG["config_directory"] + "/maps/pbstreams/" \
                                                        + req.map + "_" + req.floor + ".pbstream"

    def initRosLaunch(self):
        self.roslaunch_args = self.cli_args[1:]
        self.roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(self.cli_args)[0], self.roslaunch_args)]
        self.parent = roslaunch.parent.ROSLaunchParent(self.uuid, self.roslaunch_file)
        rospy.logwarn("=======")
        rospy.logwarn(self.cli_args)
        rospy.logwarn("=======")
        self.parent.start()

    def launchMapping(self):
        self.cli_args = [self.getMappingLaunchFile(),
                         self.robot_id_arg, self.is_sim_arg,
                         self.cur_px_arg, self.cur_py_arg, self.cur_yaw_arg,
                         self.map_arg, self.default_map_arg]

        self.initRosLaunch()
        self.mapping_active_pub.publish(self.enabled_bool_msg)
        self.localization_active_pub.publish(self.disabled_bool_msg)

    def launchLocalization(self):
        self.cli_args = [self.getLocalizationLaunchFile(),
                         self.robot_id_arg, self.is_sim_arg,
                         self.cur_px_arg, self.cur_py_arg, self.cur_yaw_arg,
                         self.map_arg, self.default_map_arg, self.pbstream_file_arg]

        self.initRosLaunch()
        self.mapping_active_pub.publish(self.disabled_bool_msg)
        self.localization_active_pub.publish(self.enabled_bool_msg)

    def getMappingLaunchFile(self) -> str:
        return self.CFG["launch_directory"] + "/" + "cartographer_mapping_bringup.launch"

    def getLocalizationLaunchFile(self) -> str:
        return self.CFG["launch_directory"] + "/" + "cartographer_localization_bringup.launch"


def main():
    lm = LauncherModule()
    try:
        rospy.spin()
    except rospy.exceptions.ROSInterruptException:
        print("")
    print("=== Shutting down launcher completed ===")

if __name__ == '__main__':
    main()
