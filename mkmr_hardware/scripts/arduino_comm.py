#!/usr/bin/env python
# coding=utf-8
import os , sys 
import rospy, tf

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
        self.rads_to_rpm = 9.5793

        self.velocity_1 = 0
        self.velocity_2 = 0
        self.velocity_3 = 0
        self.velocity_4 = 0


    def initObj(self):
        rospy.Subscriber("joint_states", JointState, self.jointStatesCb)
        self.obj_pub = rospy.Publisher('/arduino_input', ArduinoInput, queue_size=1000)


    def jointStatesCb(self, data):

        self.velocity_1 = data.velocity[0]*self.rads_to_rpm
        self.velocity_2 = data.velocity[1]*self.rads_to_rpm
        self.velocity_3 = data.velocity[2]*self.rads_to_rpm
        self.velocity_4 = data.velocity[3]*self.rads_to_rpm


        pubdata = ArduinoInput()
        pubdata.velocity_1 = round(self.velocity_1)
        pubdata.velocity_2 = round(self.velocity_2)
        pubdata.velocity_3 = round(self.velocity_4)
        pubdata.velocity_4 = round(self.velocity_3)

       
        """
        self.scale=1


        if  self.velocity_1 != 0:  
            pubdata.velocity_1 = round(self.velocity_1 *self.scale)

        if  self.velocity_2 != 0:
            pubdata.velocity_2 = round(self.velocity_2 *self.scale)

        if  self.velocity_4 != 0:
            pubdata.velocity_3 = round(self.velocity_4 *self.scale)

        if  self.velocity_3 != 0:
            pubdata.velocity_4 = round(self.velocity_3 *self.scale)

        """


        self.obj_pub.publish(pubdata)
          

def main():
    ac = ArduinoComm()
    try:
        rospy.spin()
    except rospy.exceptions.ROSInterruptException:
        print("")
    print("=== Shutting arduino_comm completed ===")

if __name__ == '__main__':
    main()