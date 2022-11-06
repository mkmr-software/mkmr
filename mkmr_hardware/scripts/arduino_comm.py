#!/usr/bin/env python
# coding=utf-8
import os , sys 
import rospy, tf

import math
from mkmr_msgs.msg import ArduinoInput
from sensor_msgs.msg import JointState

from mkmr_scripts.mkmr_base import MkmrBase

# 330 rpm - radius 0.04 - max 1.38 m/s

class ArduinoComm(MkmrBase):
    def __init__(self):
        super().__init__()
        rospy.init_node('arduino_comm')
        self.initParams()
        self.initObj()


    def initParams(self):
        self.rads_to_rpm = 60/(2 * math.pi)

        self.velocity_1 = 0
        self.velocity_2 = 0
        self.velocity_3 = 0
        self.velocity_4 = 0

        self.pubdata = ArduinoInput()


    def initObj(self):

        self.obj_pub = rospy.Publisher('arduino_input', ArduinoInput, queue_size=1000)

        rospy.Subscriber("joint_states", JointState, self.jointStatesCb)


    def jointStatesCb(self, data:JointState):

        self.velocity_1 = data.velocity[0]*self.rads_to_rpm
        self.velocity_2 = data.velocity[1]*self.rads_to_rpm
        self.velocity_3 = data.velocity[2]*self.rads_to_rpm
        self.velocity_4 = data.velocity[3]*self.rads_to_rpm


        # ROS --> FL FR BL BR
        # HARDWARE --> CLOCKWISE --> DC1 FRONT LEFT - DC2 FRONT RIGHT - DC3 BACK RIGHT - DC4 BACK LEFT 
        
        self.pubdata.velocity_1 = round(self.velocity_1) # FL
        self.pubdata.velocity_2 = round(self.velocity_2) # FR
        self.pubdata.velocity_3 = round(self.velocity_4) # BR
        self.pubdata.velocity_4 = round(self.velocity_3) # BL

        # self.scale=1

        # if  self.velocity_1 != 0:  
        #     self.pubdata.velocity_1 = round(self.velocity_1 *self.scale ) 

        # if  self.velocity_2 != 0: 
        #     self.pubdata.velocity_2 = round(self.velocity_2 *self.scale ) 

        # if  self.velocity_4 != 0: 
        #     self.pubdata.velocity_3 = round(self.velocity_4 *self.scale )

        # if  self.velocity_3 != 0: 
        #     self.pubdata.velocity_4 = round(self.velocity_3 *self.scale )

        
        self.obj_pub.publish(self.pubdata)
          

def main():
    ac = ArduinoComm()
    try:
        rospy.spin()
    except rospy.exceptions.ROSInterruptException:
        print("")
    print("=== Shutting arduino_comm completed ===")

if __name__ == '__main__':
    main()