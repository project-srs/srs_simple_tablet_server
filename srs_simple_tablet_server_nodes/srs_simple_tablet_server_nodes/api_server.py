#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
function
* select
    * drawer_select (action: ~/select/drawer_task - post /select_drawer, websocket /status select/drawer)
    * action_select (action: ~/select/action_task - post /select_action, websocket /status select/action)
* display
    * header_name (parameter: header.name - websocket /status display/header/name)
    * header_color (parameter: header.color - websocket /status display/header/color)
    * board_icon (parameter: board.icon - websocket /status display/board/icon)
    * board_message (parameter: board.message - websocket /status display/board/message)
* note (service: ~/set_note - websocket /status note)
'''

import sys
import fastapi
import uvicorn
import pydantic
import uuid
import os
from starlette.middleware.cors import CORSMiddleware
import time
import threading
import json
import asyncio

# ROS2
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from rclpy.action import ActionServer, CancelResponse


from srs_simple_tablet_server_msgs.action  import SelectTask
from srs_simple_tablet_server_msgs.srv import SetNote

####### Data Type #######

class NoteItem:
    def __init__(self, level, message):
        self.level = level
        self.message = message

class OptionItem:
    def __init__(self, name, endpoint, post_content):
        self.name = name
        self.endpoint = endpoint
        self.post_content = post_content

class ModePost(pydantic.BaseModel):
    name: str
    comment: str

class TagPost(pydantic.BaseModel):
    category: str
    serial: int

class ActionPost(pydantic.BaseModel):
    name: str
    tag: TagPost

class VehicleStatus:
    def __init__(self):
        self.mode_name ='unknown'
        self.mode_color = 'gray'
        self.board_icon = "stop"
        self.board_message = "stop"

        self.action_list = []
        self.mode_list = []
        self.note_dict = {}

    def get_json_string(self):
        data_dic = {
            "mode_name":self.mode_name,
            "mode_color":self.mode_color,
            "board_icon":self.board_icon,
            "board_message":self.board_message,
            "modes": [],
            "actions":[],
            "notes":[],
            "modes":[]
        }
        for mode_item in self.mode_list:
            data_dic["modes"].append(vars(mode_item))
        for action_item in self.action_list:
            data_dic["actions"].append(vars(action_item))
        for note_value in self.note_dict.values():
            data_dic["notes"].append(vars(NoteItem(note_value[1], note_value[2])))

        return json.dumps(data_dic)

app = fastapi.FastAPI(debug=True)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class Ros2ApiServer(Node):
    def __init__(self):
        super().__init__('api_server')
        self.declare_parameter('mode_name', 'manual')
        self.declare_parameter('mode_color', 'gray')
        self.declare_parameter('mode_list', ["HOLD", "MANUAL", "ROUND", 'GOTO', 'Transport', "FOLLOW"])
        self.declare_parameter('board_icon', 'pause')
        self.declare_parameter('board_message', 'gray')

        self.last_vehicle_status = VehicleStatus()
        # self.last_vehicle_status.note_list.append(NoteItem("info", "this is normal message."))
        # self.last_vehicle_status.note_list.append(NoteItem("warning", "this is warning message."))
        # self.last_vehicle_status.note_list.append(NoteItem("error", "this is error message."))

        # ROS2 API
        self.mode_pub = self.create_publisher(String, "~/mode_event", 1)
        self.note_srv = self.create_service(SetNote, "~/set_note", self._set_note_callback)
        self.interval_timer = self.create_timer(0.5, self._timerCallback)
        self._action_server = ActionServer(
            self,
            SelectTask,
            '~/select_task',
            self._selectExecuteCallback,
            cancel_callback=self.cancel_callback)
        self._action_result = None
        self._action_forcus_id = None

        ####### Mode #######
        @app.get("/mode")
        async def get_mode():
            return self.last_vehicle_status.mode_name

        @app.post("/mode")
        async def post_mode(mode: ModePost):
            self.get_logger().info("post_mode: " + str(mode))
            self.publishMode(mode.name)
            return mode

        ####### action #######
        @app.post("/action")
        async def post_action(action: ActionPost):
            self.get_logger().info('/action ' + str(action))
            self._action_result = action
            return action

        ####### status #######
        @app.websocket("/status")
        async def websocket_endpoint(websocket: fastapi.WebSocket):
            try:
                await websocket.accept()
                self.get_logger().info("connetct: " + websocket.headers.get("sec-websocket-key"))
                while True:
                    self.last_vehicle_status.mode_name = self.get_parameter('mode_name').get_parameter_value().string_value
                    self.last_vehicle_status.mode_color = self.get_parameter('mode_color').get_parameter_value().string_value
                    self.last_vehicle_status.board_icon = self.get_parameter('board_icon').get_parameter_value().string_value
                    self.last_vehicle_status.board_message = self.get_parameter('mode_color').get_parameter_value().string_value
                    self.last_vehicle_status.mode_list = []
                    for mode in self.get_parameter('mode_list').get_parameter_value().string_array_value:
                        self.last_vehicle_status.mode_list.append(OptionItem(mode, "/mode", json.dumps({"name":mode, "comment":""})))
                    await websocket.send_text(self.last_vehicle_status.get_json_string())
                    await asyncio.sleep(0.5)
            except Exception as e:
                self.get_logger().info("disconnected: " + str(e))

    def publishMode(self, mode):
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)

    def _timerCallback(self):
        ros_now = self.get_clock().now()
        delet_keys = [key for key, value in self.last_vehicle_status.note_dict.items() if 5.0 * (10 ** 9) < (ros_now - value[0]).nanoseconds]
        for key in delet_keys:
            del self.last_vehicle_status.note_dict[key]

    def _set_note_callback(self, request, response):
        # print(request)
        ros_now = self.get_clock().now()
        tag_str = request.tag.category + "/" + str(request.tag.serial)
        level_str = "info"
        if request.level == SetNote.Request.LEVEL_WARNING:
            level_str = "warning"
        elif request.level == SetNote.Request.LEVEL_ERROR:
            level_str = "error"
        self.last_vehicle_status.note_dict[tag_str] = [ros_now, level_str, request.content]
        response.success = True
        return response

    def _selectExecuteCallback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        self.last_vehicle_status.action_list = []
        for option in goal_handle.request.option_list:
            content = {
                "name": option.name,
                "tag" : {
                    "category" : option.tag.category,
                    "serial" : option.tag.serial
                }
            }
            self.last_vehicle_status.action_list.append(
                OptionItem(option.name, "/action", json.dumps(content))
            )
        self._action_result = None
        self._action_forcus_id = goal_handle.goal_id

        feedback_msg = SelectTask.Feedback()
        while goal_handle.is_active and goal_handle.goal_id == self._action_forcus_id:
            if goal_handle.is_cancel_requested:
                # canceled
                self.get_logger().info('cancel goal...')
                result = SelectTask.Result()
                self._action_result = None
                self.last_vehicle_status.action_list = []
                goal_handle.canceled()
                print("not active")
                return result
            elif self._action_result is not None:     
                # goal
                result = SelectTask.Result()
                result.target_tag.category = self._action_result.tag.category
                result.target_tag.serial = self._action_result.tag.serial
                self._action_result = None
                self.last_vehicle_status.action_list = []
                goal_handle.succeed()
                print("goal")
                return result

            # print("sleep", goal_handle.goal_id)
            time.sleep(0.5)

        goal_handle.abort()
        result = SelectTask.Result()
        self.get_logger().error('focus to another goal')
        return result

    def cancel_callback(self, goal):
        """Accepts or rejects a client request to cancel an action."""
        print("cancel_callback")
        return CancelResponse.ACCEPT

####### main ######
def main(argv=sys.argv):
    # uvi_thread = threading.Thread(target=uvicorn.run, args=(app,), kwargs={"host":"", "port":8010, "log_level":"warning"}, daemon=True)
    # uvi_thread.start()

    rclpy.init()
    node = Ros2ApiServer()

    executor = MultiThreadedExecutor()
    ros2_thread = threading.Thread(target=rclpy.spin, args=(node,), kwargs={"executor":executor}, daemon=True)
    ros2_thread.start()
    uvicorn.run(app, host="", port=8010, log_level="warning")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main(sys.argv)
