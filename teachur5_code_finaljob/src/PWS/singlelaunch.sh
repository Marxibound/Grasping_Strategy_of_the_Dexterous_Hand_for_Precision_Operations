#!/bin/bash

# 启动 roscore
gnome-terminal --tab -- bash -c  "roscore; exec bash"
sleep 2

# 启动UR机器人驱动
# gnome-terminal --tab -- bash -c "roslaunch realsense2_camera rs_camera.launch enable_pointcloud:=true align_depth:=true tf_prefix:=realsense color_width:=1920 color_height:=1080 color_fps:=30
# ; exec bash"
gnome-terminal --tab -- bash -c "roslaunch realsense2_camera rs_camera.launch enable_pointcloud:=true align_depth:=true tf_prefix:=realsense ; exec bash"

# sleep 2
# gnome-terminal --tab -- bash -c "rosrun tf static_transform_publisher 0.014692886492470, 0.000329219834960, 0.000243275234087 0 -1.5708 1.5708 camera_color_optical_frame realsense_link 100; exec bash"
gnome-terminal --tab -- bash -c "roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.60.101; exec bash"
sleep 5

# 启动双机器人MoveIt配置
gnome-terminal --tab -- bash -c "roslaunch single_robot_moveit_config ur5e_moveit_planning_execution.launch; exec bash"

# # 启动RViz
gnome-terminal --tab -- bash -c "roslaunch manipulator single_arm_rviz.launch; exec bash"
# sleep 5

# # 启动夹爪服务
gnome-terminal --tab -- bash -c "roslaunch gripper_modbus Gripper_ModbusControl.launch type:=RGI port:=/dev/ttyUSB0 prefix:=RGI; exec bash"
gnome-terminal --tab -- bash -c "roslaunch gripper_modbus Gripper_ModbusControl.launch type:=PGI port:=/dev/ttyUSB1 prefix:=PGI; exec bash"

# 启动定位服务
# gnome-terminal --tab -- bash -c "roslaunch locatornew location_service.launch; exec bash"
gnome-terminal --tab -- bash -c "roslaunch locatornew location_service.launch camera_image_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info camera_gTc_flag:=false; exec bash"

# gnome-terminal --tab -- bash -c "roslaunch locatornew location_service.launch camera_image_topic:=/stereo_inertial_publisher/color/image camera_info_topic:=/stereo_inertial_publisher/color/camera_info camera_gTc_flag:=true; exec bash"
# gnome-terminal --tab -- bash -c "roslaunch locatornew location_service.launch camera_image_topic:=/stereo_inertial_publisher/color/image; exec bash"

#sudo udevadm trigger
#ll /dev/ |grep ttyUSB

# 启动A_gel_start
# gnome-terminal --tab -- bash -c "roslaunch manipulator A_robot_start.launch Dual_arm_flag:=false Dashboard_flag:=true Locator_flag:=true Load_JPfile:=robot; exec bash"
gnome-terminal --tab -- bash -c "roslaunch manipulator A_robot_start.launch Dual_arm_flag:=false Dashboard_flag:=true Locator_flag:=false Load_JPfile:=robot; exec bash"
sleep 8

# gnome-terminal --tab -- bash -c "roslaunch manipulator B_JP_record.launch Record_JPfile:=robot; exec bash"
# gnome-terminal --tab -- bash -c "rosrun manipulator obstacle_sampler.py; exec bash"

roslaunch js_control js_control.launch switch_mode:=1 control_mode:=0


# OAK驱动和手眼标定
# gnome-terminal --tab -- bash -c "source ~/C_library/dai_ws/devel/setup.bash; roslaunch depthai_examples stereo_inertial_node.launch enableSpatialDetection:=False syncNN:=False enableRviz:=False; exec bash"
# gnome-terminal --tab -- bash -c "roslaunch locatornew location_service.launch camera_image_topic:=/stereo_inertial_publisher/color/image camera_info_topic:=/stereo_inertial_publisher/color/camera_info camera_gTc_flag:=true; exec bash"