![](../images/HelloRobotLogoBar.png)

## Overview

This package is in active development. Proceed with caution.

stretch_gui is meant to utilize all aspects of stretch to navigate and grasps objects. This package utilizes stretch_rtabmap to map. 

# Requirements Gazebo

```shell
cd ~/catkin_ws/src
git clone https://github.com/hello-robot/stretch_ros -b dev/noetic
git clone https://github.com/NateWright/stretch_gui.git
git clone https://github.com/pal-robotics/realsense_gazebo_plugin
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin build
```

# Usage Gazebo

```shell
# Shell 1
roslaunch stretch_rtabmap gazebo.launch
# Shell 2
roslaunch stretch_rtabmap start_rtab.launch sim:=true localization:=false move_base_config:=3d_unknown
# Shell 3
roslaunch stretch_moveit_config move_group.launch
# Shell 4
rosrun web_video_server web_video_server
# Shell 5
roslaunch stretch_gui_server stretch_gui_server.launch sim:=true
```

# Requirements Stretch RE1

```shell
cd ~/catkin_ws/src
git clone https://github.com/hello-robot/stretch_ros -b dev/noetic
git clone https://github.com/NateWright/stretch_gui.git
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin build
```

# Usage Stretch

```shell
# Shell 1
roslaunch stretch_rtabmap start_rtab.launch sim:=false localization:=false move_base_config:=3d_unknown
# Shell 2
rosrun web_video_server web_video_server
# Shell 3
roslaunch stretch_gui_server stretch_gui_server.launch sim:=false
```

# Client app

App located at [GitHub - NateWright/stretch_gui_client](https://github.com/NateWright/stretch_gui_client) 

Release located at [Release Full Android App · NateWright/stretch_gui_client · GitHub](https://github.com/NateWright/stretch_gui_client/releases/tag/v1.0)

IP and Port to connect to server will be in server terminal
