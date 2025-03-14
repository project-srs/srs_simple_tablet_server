# srs_simple_tablet_server

ROS2-WebAPI bridge

## select

| name | WebAPI(websocket) | WebAPI(Post) | ROS(action server) | comment |
|:--|:--|:--|:--|:--|
|drawer_select|/status select/drawer|/select/drawer|~/select/drawer| set options, and get result |
|action_select|/status select/action|/select/action|~/select/action| set options, and get result |

## display

| name | WebAPI(websocket) | ROS(parameter) | comment |
|:--|:--|:--|:--|
|header_name|/status display/header/name|header.name|header name|
|header_color|/status display/header/color|header.color|header color|
|board_icon|/status display/board/icon|board.icon|bottom board icon|
|board_message|/status display/board/message|board.message|bottom board message|

## node
| name | WebAPI(websocket) | ROS(service) | comment |
|:--|:--|:--|:--|
|note|/status note|~/set_note|note|
