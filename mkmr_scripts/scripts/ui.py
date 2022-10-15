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
from websocket_server import WebsocketServer
from rospy_message_converter import json_message_converter, message_converter

from mkmr_scripts.mkmr_base import MkmrBase


class UIModule(MkmrBase):
    def __init__(self):
        super().__init__()
        rospy.init_node('ui')

        self.STATUS_COMM = False
        
        self.initialize()
        self.defineProcessFunctions()
        self.initWebSocketServer()

    # Initialize -------------------------------------------------------------------------------------------------------

    def initialize(self):
        self.currents = {
            'latest_received_heartbeat_time' : time.time(),         
        }
        
    # Define Process Functions -----------------------------------------------------------------------------------------

    def defineProcessFunctions(self):
        self.process_functions = {
            "heartbeat_ui" : self.processHearbeatUi,
            "add_loc" : self.processAddLoc # {"project_id":0,"robot_id":"mkmr0","topic":"add_loc","message":{"type":"target","name":"loc1"}}
        }

    # Initialize Websocket Server  -------------------------------------------------------------------------------------

    def initWebSocketServer(self):
        if not self.CFG["ui_port"] == 0:
                os.system("fuser -k " + str(self.CFG["ui_port"]) + "/tcp ")
                time.sleep(2)

                try:
                    self.server = WebsocketServer(
                        host=self.CFG["ui_ip"],
                        port=int(self.CFG["ui_port"]),
                        loglevel=logging.INFO)
                except OSError as error:
                    self.consoleError("Something went wrong Websocket server error -- " + str(error))

                try:
                    getattr(self, "server")
                    self.server.set_fn_new_client(self.newClient)
                    self.server.set_fn_client_left(self.clientLeft)
                    self.server.set_fn_message_received(self.wsMessageReceivedCallback)

                    self.ui_status_pub = rospy.Publisher("ui_status", Bool, queue_size=1, latch=True)

                    self.heartbeat_timer = rospy.Timer(rospy.Duration(2.0), self.heartbeatTimerCb)

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


    def heartbeatTimerCb(self, timer):
        if (time.time()- self.currents['latest_received_heartbeat_time']) > 8:
            self.publishUiStatus(False)
        else:
            self.publishUiStatus(True)
        self.publishStrToUi("heartbeat_ui", "")

    # Websocket Functions ----------------------------------------------------------------------------------------------

    def newClient(self, client, server):
        self.consoleInfo(
            "New Client connected id:" + str(client["id"]) + 
            " Count of Active Client:" + str(len(self.server.clients)))

        self.server.send_message(client, self.jsonToStr(self.addTopicToJson(
            "client_connected", "success", self.CFG["project_id"], self.RID)))

        self.publishJsonToUi("CFG", self.CFG)

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
                self.consoleError("Error" + str(e))
            
    def publishJsonStrToUi(self, topic: str, data: str):
        """ Convert string before pub if data is json str """
        if len(self.server.clients) > 0:
            try:
                self.server.send_message_to_all(self.jsonToStr(self.addTopicToJson(
                        topic, self.strToJson(data), self.CFG["project_id"], self.RID)))
            except Exception as e:
                self.consoleError("Error" + str(e))

    def publishStrToUi(self, topic: str, data: str):
        """ Publish directly if data is str """
        if len(self.server.clients) > 0:
            try:
                self.server.send_message_to_all(self.jsonToStr(self.addTopicToJson(
                        topic, data, self.CFG["project_id"], self.RID)))
            except Exception as e:
                self.consoleError("Error" + str(e))

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

def main():
    uim = UIModule()
    try:
        rospy.spin()
    except rospy.exceptions.ROSInterruptException:
        print("")
    print("=== Shutting down ui completed ===")

if __name__ == '__main__':
    main()
