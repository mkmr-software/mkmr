#!/usr/bin/env python3
# coding=utf-8
import rospy
import os
import time
import logging
import threading
import json
from std_msgs.msg import *
from sensor_msgs.msg import *
from mkmr_msgs.msg import *
from mkmr_srvs.srv import *
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist

from websocket_server import WebsocketServer
from rospy_message_converter import json_message_converter, message_converter

from mkmr_scripts.mkmr_base import MkmrBase

from mkmr_srvs.msg import TaskBaseAction, TaskBaseActionFeedback, TaskBaseActionGoal, TaskBaseActionResult \
        ,TaskBaseFeedback, TaskBaseGoal,TaskBaseResult

class UIModule(MkmrBase):
    def __init__(self):
        super().__init__()
        rospy.init_node('ui')

        self.STATUS_COMM = False
        self.CLOUD_VEL_TIMEOUT = 1.5
        
        self.initialize()
        self.defineProcessFunctions()
        self.initWebSocketServer()

    # Initialize -------------------------------------------------------------------------------------------------------

    def initialize(self):
        self.currents = {
            'latest_received_heartbeat_time' : time.time(),
            'mkmr': Mkmr(),
            'compressed_map': String(),
            'enable_manual_control': False,            
            'manual_control': Twist(),
            'latest_received_speed_time'  : time.time(),        
        }
        
    # Define Process Functions -----------------------------------------------------------------------------------------

    def defineProcessFunctions(self):
        self.process_functions = {
            "heartbeat_ui" : self.processHearbeatUi,
            "add_loc" : self.processAddLoc, # {"project_id":0,"robot_id":"mkmr0","topic":"add_loc","message":{"type":"target","name":"loc1"}}
            "start_map" : self.processStartMap, # {"project_id":0,"robot_id":"mkmr0","topic":"start_map","message":{"enable":"True"}}
            "save_map" : self.processSaveMap, # {"project_id":0,"robot_id":"mkmr0","topic":"save_map","message":{"map":"test","floor":"1"}}
            "start_nav" : self.processStartNav, # {"project_id":0,"robot_id":"mkmr0","topic":"start_nav","message":{"map":"test","floor":"1"}}
            "run_task_base" : self.processRunTaskBase, # {"project_id":0,"robot_id":"mkmr0","topic":"start_map","message":{"enable":"True"}}
            "start_task" : self.processStartTask, # {"project_id":0,"robot_id":"mkmr0","topic":"start_map","message":{"loc_name":"locX"}}
            "pause_task" : self.processPauseTask, # {"project_id":0,"robot_id":"mkmr0","topic":"start_map","message":{"enable":"True"}}
            "continue_task" : self.processContinueTask, # {"project_id":0,"robot_id":"mkmr0","topic":"start_map","message":{"enable":"True"}}
            "enable_manual_control" : self.processEnableManualControl, # {"project_id":0,"robot_id":"mkmr0","topic":"enable_manual_control","message":{"enable":"True"}}
            "manual_control" : self.processManualControl, # {"project_id":0,"robot_id":"mkmr0","topic":"manual_control","message":{"cmd":"x,z"}}
        }

    # Initialize Websocket Server  -------------------------------------------------------------------------------------

    def initWebSocketServer(self):
        if not int(os.getenv('MKMR_UI_PORT')) == 0:
                os.system("fuser -k " + str(int(os.getenv('MKMR_UI_PORT'),)) + "/tcp ")
                time.sleep(2)

                try:
                    self.server = WebsocketServer(
                        host = os.getenv('MKMR_UI_IP'),  
                        port = int(os.getenv('MKMR_UI_PORT'),),
                        loglevel = logging.INFO)
                except OSError as error:
                    self.consoleError("Something went wrong Websocket server error -- " + str(error))

                try:
                    getattr(self, "server")
                    self.server.set_fn_new_client(self.newClient)
                    self.server.set_fn_client_left(self.clientLeft)
                    self.server.set_fn_message_received(self.wsMessageReceivedCallback)

                    self.ui_status_pub = rospy.Publisher("ui_status", Bool, queue_size=1, latch=True)

                    self.task_base_goal_pub = rospy.Publisher("task_base/goal", TaskBaseActionGoal, queue_size=1)

                    self.enable_manual_control_pub = rospy.Publisher("enable_manual_control", Bool, 
                                                                                        queue_size=1, latch=True)

                    self.manual_control_pub = rospy.Publisher("manual_ctrl_vel", Twist, queue_size=1)

                    self.mkmr_config_sub = rospy.Subscriber("mkmr_config", Bool, self.mkmrConfigCb)

                    self.map_sub = rospy.Subscriber("map", OccupancyGrid, self.mapCb)

                    self.heartbeat_timer = rospy.Timer(rospy.Duration(2.0), self.heartbeatTimerCb)

                    self.mkmr_timer = rospy.Timer(rospy.Duration(1 / 5), self.mkmrTimerCb)

                    self.manual_control_vel_reset_timer = rospy.Timer(rospy.Duration(1 / 2), 
                                                                    self.manualControlVelResetTimerCb)
                    self.manual_control_vel_publish_timer = rospy.Timer(rospy.Duration(1 / 10), 
                                                                    self.manualControlVelPublishTimerCb)

                    # must be last line
                    # self.server.run_forever()
                    self.server.keep_running = True 
                    self.wst = threading.Thread(target=self.server.run_forever)
                    self.wst.daemon = True
                    self.wst.start()
                    rate = rospy.Rate(1)
                    while not rospy.is_shutdown():          
                        rate.sleep()
                    self.server.keep_running = False

                except AttributeError as error:
                    self.consoleError("ui node failed to initialize -- Check your ip adress" + str(error))

    # ROS Timer Callbacks ----------------------------------------------------------------------------------------------

    def heartbeatTimerCb(self, timer):
        if (time.time()- self.currents['latest_received_heartbeat_time']) > 8:
            self.publishUiStatus(False)
        else:
            self.publishUiStatus(True)
        self.publishStrToUi("heartbeat_ui", "")

    def mkmrTimerCb(self, timer):
        if self.currents["mkmr"] != self.cur_mkmr_msg:
            self.publishJsonToUi("mkmr", json_message_converter.convert_ros_message_to_json(self.cur_mkmr_msg))
        self.currents["mkmr"] = self.cur_mkmr_msg

    def manualControlVelResetTimerCb(self, timer):
        if self.currents["enable_manual_control"]:
            if (time.time()- self.currents['latest_received_speed_time']) > self.CLOUD_VEL_TIMEOUT:
                self.resetCloudVel("manual_control")
            
    def manualControlVelPublishTimerCb(self, timer):
        if self.currents["enable_manual_control"]:
            self.publishCmdVel("manual_control") 

    # Websocket Functions ----------------------------------------------------------------------------------------------

    def newClient(self, client, server):
        self.consoleInfo(
            "New Client connected id:" + str(client["id"]) + 
            " Count of Active Client:" + str(len(self.server.clients)))

        self.server.send_message(client, self.jsonToStr(self.addTopicToJson(
            "client_connected", "success", self.CFG["project_id"], self.RID)))

        self.publishJsonToUi("CFG", self.CFG)
        self.publishJsonToUi("mkmr", json_message_converter.convert_ros_message_to_json(self.currents["mkmr"]))

        self.publishJsonToUi("compressed_map", json_message_converter.convert_ros_message_to_json(self.currents["compressed_map"]))


    def clientLeft(self, client, server):
        self.consoleInfo(
            "Client disconnected id:" + str(client["id"]) + 
            " Count of Active Client:" + str(len(self.server.clients)))

    def publishJsonToUi(self, topic: str, data: dict):
        """ Publish directly if data is dict """
        if len(self.server.clients) > 0:
            try:
                self.server.send_message_to_all(self.jsonToStr(self.addTopicToJson(
                        topic, data, self.CFG["project_id"], self.RID)))
            except Exception as e:
                self.consoleError("Error publishJsonToUi" + str(e))
            
    def publishJsonStrToUi(self, topic: str, data: str):
        """ Convert string before pub if data is json str """
        if len(self.server.clients) > 0:
            try:
                self.server.send_message_to_all(self.jsonToStr(self.addTopicToJson(
                        topic, self.strToJson(data), self.CFG["project_id"], self.RID)))
            except Exception as e:
                self.consoleError("Error publishJsonStrToUi" + str(e))

    def publishStrToUi(self, topic: str, data: str):
        """ Publish directly if data is str """
        if len(self.server.clients) > 0:
            try:
                self.server.send_message_to_all(self.jsonToStr(self.addTopicToJson(
                        topic, data, self.CFG["project_id"], self.RID)))
            except Exception as e:
                self.consoleError("Error publishStrToUi" + str(e))

    def wsMessageReceivedCallback(self, client, server, message):
        msg_json = self.strToJson(message)
        if "topic" in msg_json and "message" in msg_json:
            topic = msg_json["topic"]
            data = msg_json["message"]
            if not topic == "":
                if self.CFG["ui_debug"] and topic != "heartbeat_ui":
                    self.consoleInfo("Client " + str(client["id"]) + " *topic::: " + str(topic) + " *data: " + str(data))

                if topic in self.process_functions:
                    self.process_functions[topic](topic, data)
                    return


    # Publish Functions ------------------------------------------------------------------------------------------------

    def publishUiStatus(self, status: bool):
        if self.STATUS_COMM != status:
            self.STATUS_COMM = status
            msg = Bool(data = status)
            self.ui_status_pub.publish(msg)
            if status:
               self.consoleCyan("UI Status: " + str(status))     
            else:
                self.consoleError("UI Status: " + str(status))  

    def publishCmdVel(self, topic):
        self.manual_control_pub.publish(self.currents[topic])

    # Process Functions ------------------------------------------------------------------------------------------------     

    def processHearbeatUi(self, topic, message):
        self.currents['latest_received_heartbeat_time'] = time.time()

    def processAddLoc(self, topic, message):
        req = LocationRequest()
        req.process = "add"

        try:
            req.location.type = message["type"]
            req.location.name = message["name"]
            req.location.map = self.cur_mkmr_msg.map
            req.location.floor = self.cur_mkmr_msg.floor
            req.location.px = str(self.cur_mkmr_msg.px)
            req.location.py = str(self.cur_mkmr_msg.py)
            req.location.yaw = str(self.cur_mkmr_msg.yaw)

        except Exception as e:
            self.consoleError("Error UI processAddLoc " + str(e))
            return False
        
        self.callRosService( "/" + self.RID + "/" + "location", Location, req, print_info=True)

    def processStartMap(self, topic, message):
        if self.getBool(str(message["enable"])):
            req = LauncherRequest()
            req.mapping = True
            self.callRosService( "/" + self.RID + "/" + "launcher", Launcher, req, print_info=True)

    def processSaveMap(self, topic, message):
        req = SaveMapRequest()
        try:
            req.map = message["map"]
            req.floor = message["floor"]

        except Exception as e:
            self.consoleError("Error UI processSaveMap " + str(e))
            return False

        self.callRosService( "/" + self.RID + "/" + "save_map", SaveMap, req, print_info=True)

    def processStartNav(self, topic, message):
        req = LauncherRequest()
        try:
            req.map = message["map"]
            req.floor = message["floor"]

        except Exception as e:
            self.consoleError("Error UI processStartNav " + str(e))
            return False

        self.callRosService( "/" + self.RID + "/" + "launcher", Launcher, req, print_info=True)

    def processRunTaskBase(self, topic, message):
        if self.getBool(str(message["enable"])):
            msg = TaskBaseActionGoal()
            msg.goal.RunTaskBase = True
            self.task_base_goal_pub.publish(msg)

    def processPauseTask(self, topic, message):
        if self.getBool(str(message["enable"])):
            msg = TaskBaseActionGoal()
            msg.goal.Pause = True
            self.task_base_goal_pub.publish(msg)

    def processContinueTask(self, topic, message):
        if self.getBool(str(message["enable"])):
            msg = TaskBaseActionGoal()
            msg.goal.Continue = True
            self.task_base_goal_pub.publish(msg)

    def processStartTask(self, topic, message):
        if message["loc_name"] != "":
            msg = TaskBaseActionGoal()
            msg.goal.Start = True
            msg.goal.TargetName = message["loc_name"]
            self.task_base_goal_pub.publish(msg)

    def processEnableManualControl(self, topic, message):
        self.currents[topic] = self.getBool(str(message["enable"]))
        if self.getBool(str(message["enable"])):
            self.enable_manual_control_pub.publish(self.enabled_bool_msg)
        else:
            self.enable_manual_control_pub.publish(self.disabled_bool_msg)


    def processManualControl(self, topic, message):
        if self.currents["enable_" + topic]:
            speed_str = message['cmd'].split(",")
            speed_str_size = len(speed_str)
            speed_str_all_float = True
            try:
                self.speed_x = float(speed_str[0])
            except ValueError:
                speed_str_all_float = False
            try:
                self.speed_z = float(speed_str[1])
            except ValueError:
                speed_str_all_float = False

            if speed_str_size != 2 or not speed_str_all_float:
                self.consoleError("processManualControl syntax error: " + message)
                return

            self.currents['latest_received_speed_time'] = time.time()
            self.currents[topic].linear.x = self.speed_x
            self.currents[topic].angular.z = self.speed_z


    # ROS Callbacks ----------------------------------------------------------------------------------------------------

    def mkmrConfigCb(self, msg: Bool):
        self.updateCFG()
        self.publishJsonToUi("CFG", self.CFG)

    def mapCb(self, msg: OccupancyGrid):
        compressed_map_str = ""
        compressed_map_str += str(msg.info.width) + ","
        compressed_map_str += str(msg.info.height) + ","
        compressed_map_str += str(msg.info.origin.position.x) + ","
        compressed_map_str += str(msg.info.origin.position.y) + ","
        list_of_tuples = self.shrinkMapArrayToListOfTuples(msg.data)
        for i in range(3000): # TODO BUG len(list_of_tuples)
            compressed_map_str += str(list_of_tuples[i][1]) + "x" + str(list_of_tuples[i][0]) + ","
        compressed_map_str = compressed_map_str[:-1]  # remove last ","

        if self.currents["compressed_map"].data != compressed_map_str:
            self.currents["compressed_map"].data = compressed_map_str
            self.publishJsonToUi("compressed_map", json_message_converter.convert_ros_message_to_json(self.currents["compressed_map"]))

        self.currents["compressed_map"].data = compressed_map_str


    # Other Functions ------------------------------------------------------------------------------------------------

    def resetCloudVel(self, topic):
        self.currents[topic] = Twist() 
        self.publishCmdVel(topic) 


def main():
    uim = UIModule()
    try:
        rospy.spin()
    except rospy.exceptions.ROSInterruptException:
        print("")
    print("=== Shutting down ui completed ===")

if __name__ == '__main__':
    main()
