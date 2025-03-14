#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import threading
import json
import sys

# FastAPI
import fastapi
import uvicorn
import pydantic
from starlette.middleware.cors import CORSMiddleware
import asyncio

# ROS2
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, CancelResponse
from srs_simple_tablet_server_msgs.action  import SelectTask
from srs_simple_tablet_server_msgs.srv import SetNote

####### Data Type #######
class SelectPostData(pydantic.BaseModel):
    goal_id: str
    key: str

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

        # drawer_select
        self.drawer_select_action_server = ActionServer(self, SelectTask, '~/select/drawer', self.drawer_select_callback, cancel_callback=self.cancel_accept_callback)
        self.drawer_select_goal_handle_dic = {}
        self.drawer_select_post_data_dic = {}

        # action_select
        self.action_select_action_server = ActionServer(self, SelectTask, '~/select/action', self.action_select_callback, cancel_callback=self.cancel_accept_callback)
        self.action_select_goal_handle_dic = {}
        self.action_select_post_data_dic = {}

        # display
        self.declare_parameter('display.header.name', 'manual')
        self.declare_parameter('display.header.color', 'gray')
        self.declare_parameter('display.board.icon', 'pause')
        self.declare_parameter('display.board.message', 'none')

        # note
        self.note_srv = self.create_service(SetNote, "~/set_note", self.set_note_callback)
        self.note_data_dic = {}
        self.interval_timer = self.create_timer(0.5, self.timer_callback)

        @app.post("/select/drawer")
        async def post_drawer_select(post_data: SelectPostData):
            self.get_logger().info("post_drawer_select: " + str(post_data))
            self.drawer_select_post_data_dic[post_data.goal_id] = post_data
            return fastapi.responses.JSONResponse(status_code=fastapi.status.HTTP_200_OK)

        @app.post("/select/action")
        async def post_action_select(post_data: SelectPostData):
            self.get_logger().info("post_action_select: " + str(post_data))
            self.action_select_post_data_dic[post_data.goal_id] = post_data
            return fastapi.responses.JSONResponse(status_code=fastapi.status.HTTP_200_OK)

        @app.websocket("/status")
        async def websocket_endpoint(websocket: fastapi.WebSocket):
            try:
                await websocket.accept()
                self.get_logger().info("connetct: " + websocket.headers.get("sec-websocket-key"))
                while True:
                    await websocket.send_text(self.generate_status())
                    await asyncio.sleep(0.5)
            except Exception as e:
                self.get_logger().info("disconnected: " + str(e))

    def drawer_select_callback(self, goal_handle):
        goal_id_str = str(bytes(goal_handle.goal_id.uuid).hex())
        self.get_logger().info(f'drawer_select_callback: {goal_id_str}')
        self.drawer_select_goal_handle_dic[goal_id_str] = goal_handle.request

        result = SelectTask.Result()
        while goal_handle.is_active:
            if goal_handle.is_cancel_requested:
                # canceled
                self.get_logger().info('drawer_select: cancel request')
                goal_handle.canceled()
                break

            elif goal_id_str in self.drawer_select_post_data_dic.keys():     
                # receive post data
                self.get_logger().info('drawer_select: get post data')
                post_data = self.drawer_select_post_data_dic[goal_id_str]
                goal_handle.succeed()

                result.selected_key = post_data.key
                del self.drawer_select_post_data_dic[goal_id_str]
                break

            feedback_msg = SelectTask.Feedback()
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)
        else:
            self.get_logger().info('drawer_select: error')
            goal_handle.abort()

        del self.drawer_select_goal_handle_dic[goal_id_str]
        return result

    def action_select_callback(self, goal_handle):
        goal_id_str = str(bytes(goal_handle.goal_id.uuid).hex())
        self.get_logger().info(f'action_select_callback: {goal_id_str}')
        self.action_select_goal_handle_dic[goal_id_str] = goal_handle.request

        result = SelectTask.Result()
        while goal_handle.is_active:
            if goal_handle.is_cancel_requested:
                # canceled
                self.get_logger().info('action_select: cancel request')
                goal_handle.canceled()
                break

            elif goal_id_str in self.action_select_post_data_dic.keys():     
                # receive post data
                self.get_logger().info('action_select: get post data')
                post_data = self.action_select_post_data_dic[goal_id_str]
                goal_handle.succeed()

                result.selected_key = post_data.key
                del self.action_select_post_data_dic[goal_id_str]
                break

            feedback_msg = SelectTask.Feedback()
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)
        else:
            self.get_logger().info('action_select: error')
            goal_handle.abort()

        del self.action_select_goal_handle_dic[goal_id_str]
        return result

    def cancel_accept_callback(self, goal_handle):
        goal_id_str = str(bytes(goal_handle.goal_id.uuid).hex())
        self.get_logger().info(f'cancel_accept_callback: {goal_id_str}')
        return CancelResponse.ACCEPT

    def set_note_callback(self, request, response):
        ros_now = self.get_clock().now()
        self.note_data_dic[request.key] = [ros_now, request]
        response.success = True
        return response

    def timer_callback(self):
        ros_now = self.get_clock().now()
        delete_duration = 5.0
        delet_keys = [key for key, value in self.note_data_dic.items() if delete_duration * (10 ** 9) < (ros_now - value[0]).nanoseconds]
        for key in delet_keys:
            del self.note_data_dic[key]

    def generate_status(self):
        def generate_options(goal_handle_dic):
            output_list = []
            for goal_id, request in goal_handle_dic.items():
                for option in request.option_list:
                    output_list.append({"goal_id":goal_id, "key":option.key, "display_name":option.display_name})
            return output_list

        # drawer_select
        drawer_select_data = {}
        drawer_select_data["options"] = generate_options(self.drawer_select_goal_handle_dic)

        # action_select
        action_select_data = {}
        action_select_data["options"] = generate_options(self.action_select_goal_handle_dic)

        # display
        display_data = {}
        display_data["header"] = {}
        display_data["header"]["name"] = self.get_parameter('display.header.name').get_parameter_value().string_value
        display_data["header"]["color"] = self.get_parameter('display.header.color').get_parameter_value().string_value
        display_data["board"] = {}
        display_data["board"]["icon"] = self.get_parameter('display.board.icon').get_parameter_value().string_value
        display_data["board"]["message"] = self.get_parameter('display.board.message').get_parameter_value().string_value

        # note
        note_data = {}
        note_data["contents"] = []
        for item in self.note_data_dic.values():
            level = "none"
            if item[1].level == SetNote.Request.LEVEL_INFO:
                level = "info"
            elif item[1].level == SetNote.Request.LEVEL_WARNING:
                level_str = "warning"
            elif item[1].level == SetNote.Request.LEVEL_ERROR:
                level_str = "error"
            note_data["contents"].append({"level":level, "message":item[1].message})

        # output
        send_data = {}
        send_data["drawer_select"] = drawer_select_data
        send_data["action_select"] = action_select_data
        send_data["display"] = display_data
        send_data["note"] = note_data
        return json.dumps(send_data)

def main(argv=sys.argv):
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
