#!/usr/bin/env python
# coding=utf-8
import rospy
import os
import yaml
import tf
import actionlib
from mkmr_msgs.msg import *
from mkmr_srvs.srv import *
from std_msgs.msg import *

from rospy_message_converter import json_message_converter, message_converter

from mkmr_srvs.msg import TaskBaseAction, TaskBaseActionFeedback, TaskBaseActionGoal, TaskBaseActionResult \
        ,TaskBaseFeedback, TaskBaseGoal,TaskBaseResult
from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray

from actionlib_msgs.msg import GoalID, GoalStatusArray
from geometry_msgs.msg import PoseStamped 

from mkmr_scripts.mkmr_base import MkmrBase

class TaskBaseModule(MkmrBase):
    _feedback = TaskBaseFeedback()
    _result = TaskBaseResult()

    def __init__(self):
        super().__init__()
        rospy.init_node('task_base')

        self.CONFIG_DIR = rospy.get_param('~config_dir', "/UNKNOWN/") 

        self.RUN = False



        self.STATE_COUNTER_TIME = {
            State.STARTUP : 0,
            State.READY_FOR_TASK : 0,
            State.GOAL : 0,
        }

        self.STATE_SET_TIME = {
            State.STARTUP : 2,
            State.READY_FOR_TASK : 2,
            State.GOAL : 0.5,
        }

        self.move_base_updated = False 
        self.last_goal_req = dict()
        self.status_move_base = 0


        self.goal_cancel_pub = rospy.Publisher("/" + self.RID + "/move_base/cancel", GoalID, queue_size=10)
        self.goal_pub = rospy.Publisher("/" + self.RID + "/move_base_simple/goal", PoseStamped, queue_size=10)

        self.move_base_status_sub = rospy.Subscriber("move_base/status", GoalStatusArray, self.moveBaseStatusCb)

        self.mkmr_config_sub = rospy.Subscriber("mkmr_config", Bool, self.mkmrConfigCb)

        self._action_name = "task_base"
        self._as = actionlib.SimpleActionServer(self._action_name,TaskBaseAction, 
                                    execute_cb=  self.execute_cb, auto_start = False)

        self._as.start()

    # ROS Callbacks ----------------------------------------------------------------------------------------------------

    def moveBaseStatusCb(self, msg:GoalStatusArray):
        # uint8 PENDING=0
        # uint8 ACTIVE=1
        # uint8 PREEMPTED=2
        # uint8 SUCCEEDED=3
        # uint8 ABORTED=4
        # uint8 REJECTED=5
        # uint8 PREEMPTING=6
        # uint8 RECALLING=7
        # uint8 RECALLED=8
        # uint8 LOST=9

        if len(msg.status_list) != 0:
            if len(msg.status_list) == 2:
                # 1 go 2 nothing 3 goal reached 4 error
                self.status_move_base = msg.status_list[1].status
            else:
                self.status_move_base = msg.status_list[0].status

        else:
            self.status_move_base = 0  # nothing


    def mkmrConfigCb(self, msg: Bool):
        self.updateCFG()

    def publishGoal(self, x , y , z):
        data = PoseStamped()
        data.header.stamp = rospy.Time.now()
        data.header.frame_id = self.RID + "/map"

        quaternion = tf.transformations.quaternion_from_euler(0, 0, float(z) * (3.141592 / 180.0))

        data.pose.position.x = float(x)
        data.pose.position.y = float(y)
        data.pose.orientation.z = quaternion[2]
        data.pose.orientation.w = quaternion[3]

        self.goal_pub.publish(data)

    def publishGoalCancel(self):
        data = GoalID()
        data.stamp = rospy.Time.now()
        data.id = ''
        self.goal_cancel_pub.publish(data)

    def STARTUP_P(self):
        pass

    def STARTUP_N(self):
        pass

    def READY_FOR_TASK_P(self):
        pass

    def READY_FOR_TASK_N(self):
        pass

    def GOAL_P(self):
        pass

    def GOAL_N(self):
        pass

    def execute_cb(self, goal:TaskBaseActionGoal):


        if goal.RunTaskBase:
            self.consoleInfo('%s: ACTION START TRIGGER' % self._action_name)

            self.RUN = True 

            self.consoleInfo('%s: STARTUP ' % self._action_name)
            self._feedback.STATE = State.STARTUP

        if goal.Start and self._feedback.STATE != State.STARTUP:
            for i in range(0,len(self.CFG["locs"])):
                if goal.TargetName == (self.CFG["locs"][i])["name"]:
                    self.last_goal_req = self.CFG["locs"][i]
                    self.consoleInfo('%s: NEW TASK TRIGGER' % self._action_name)

                    self._feedback.STATE = State.GOAL

                    self._feedback.TargetName = self.last_goal_req["name"]
        
                    self.publishGoal(self.last_goal_req["px"],
                                self.last_goal_req["py"],
                                self.last_goal_req["yaw"])
                    
                    break
                    
            self.RUN = True  
            
            # self.consoleInfo('%s: error unknown loc' % self._action_name) TODO

        # if goal.Pause: 
        #     self.publishGoalCancel()   
        #     self.RUN = True


        r = rospy.Rate(10)
       
        while self.RUN:

            if self._as.is_preempt_requested():
                self.publishGoalCancel()
                self.RUN = False
                self.consoleInfo('%s: ACTION CANCEL TRIGGER '%self._action_name)
                self._as.set_preempted()


            # STARTUP --------------------------------------------------------------------------------------------------
            if self._feedback.STATE == State.STARTUP:
                self.STATE_COUNTER_TIME[State.STARTUP] += 0.1

                # STARTUP Transitions ----------------------------------------------------------------------------------
                if int(self.STATE_COUNTER_TIME[State.STARTUP]) == self.STATE_SET_TIME[State.STARTUP]:
                    self.consoleInfo('%s: STARTUP to READY_FOR_TASK' % self._action_name)
                    self._feedback.STATE = State.READY_FOR_TASK
                               
            else:
                self.STATE_COUNTER_TIME[State.STARTUP] = 0


            # GOAL --------------------------------------------------------------------------------------------------
            if self._feedback.STATE == State.GOAL:
                # waiting time move_base status changing x to 1
                if self.status_move_base == 1:
                    self.move_base_updated = True


                if float(self.STATE_COUNTER_TIME[State.GOAL]) >= self.STATE_SET_TIME[State.GOAL] \
                                                    and self.move_base_updated == True:

                    # GOAL Transitions ---------------------------------------------------------------------------------
                    
                    if self.status_move_base == GoalStatus.SUCCEEDED:
                        self.move_base_updated = False   
                        rospy.loginfo('%s: GOAL to SIGNAL' % self._action_name)
                        self._feedback.STATE = State.READY_FOR_TASK
                        self._feedback.LastDoneTargetName = self.last_goal_req["name"]
            
                     
                    if self.status_move_base == GoalStatus.ABORTED:
                        self.move_base_updated = False   
                        self._result.sequence = [1]  # id:1 goal_error
                        self._as.set_aborted(self._result)                    
                    

                else:
                    self.STATE_COUNTER_TIME[State.GOAL] += 0.1

            else:
                self.STATE_COUNTER_TIME[State.GOAL] = 0


            self._as.publish_feedback(self._feedback)
            r.sleep()



def main():
    smm = TaskBaseModule()
    try:
        rospy.spin()
    except rospy.exceptions.ROSInterruptException:
        print("")
    print("=== Shutting down launcher completed ===")

if __name__ == '__main__':
    main()
