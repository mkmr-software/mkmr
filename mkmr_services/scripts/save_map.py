#!/usr/bin/env python
# coding=utf-8
import rospy
import os
import roslaunch.rlutil, roslaunch.parent
from mkmr_msgs.msg import *
from mkmr_srvs.srv import *
from std_msgs.msg import *
from cartographer_ros_msgs.srv import *

from mkmr_scripts.mkmr_base import MkmrBase

class SaveMapModule(MkmrBase):
    def __init__(self):
        super().__init__()
        rospy.init_node('save_map')

        self.CONFIG_DIR = rospy.get_param('~config_dir', "/UNKNOWN/") 

        self.consoleCyan(self.CONFIG_DIR)

        self.createDirectoryIfNotExists(self.CONFIG_DIR)
        self.createDirectoryIfNotExists(self.CONFIG_DIR + "/maps")
        self.createDirectoryIfNotExists(self.CONFIG_DIR + "/maps/pbstreams")

        self.save_map_server = rospy.Service("save_map", SaveMap, self.execute)

        self.RID = os.getenv('MKMR_ROBOT_ID')
        
    def execute(self, req:SaveMapResponse):
        try:
            while not rospy.is_shutdown():
                self.mapSaver(req.map + "_" + req.floor)
                return True

        except Exception as e:
            self.consoleError("Error" + str(e))
            return False

    def mapSaver(self, site_floor):
             
        os.system("rosrun map_server map_saver map:=/" + self.RID + "/map -f " +  
                                            self.CONFIG_DIR + "/maps/" + site_floor)

        req = FinishTrajectoryRequest()
        req.trajectory_id = 0
        self.callRosService( "/" + self.RID + "/" + "finish_trajectory", FinishTrajectory, req, print_info=True)

        req = WriteStateRequest()
        req.filename = self.CONFIG_DIR + "/maps/pbstreams/" + site_floor + ".pbstream"
        req.include_unfinished_submaps = True
        self.callRosService( "/" + self.RID + "/" + "write_state", WriteState, req, print_info=True)

def main():
    smm = SaveMapModule()
    try:
        rospy.spin()
    except rospy.exceptions.ROSInterruptException:
        print("")
    print("=== Shutting down launcher completed ===")

if __name__ == '__main__':
    main()
