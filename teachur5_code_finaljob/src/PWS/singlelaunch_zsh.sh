#!/bin/zsh

# 启动 roscore
gnome-terminal --tab -- zsh -c  "roscore; exec zsh"
sleep 2

# 启动UR机器人驱动
gnome-terminal --tab -- zsh -c "roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.131.11; exec zsh"
sleep 5

# 启动双机器人MoveIt配置
gnome-terminal --tab -- zsh -c "roslaunch single_robot_moveit_config ur5e_moveit_planning_execution.launch; exec zsh"

# # 启动RViz
gnome-terminal --tab -- zsh -c "roslaunch manipulator single_arm_rviz.launch; exec zsh"
# sleep 5


# # 启动夹爪服务
gnome-terminal --tab -- zsh -c "roslaunch gripper_modbus Gripper_ModbusControl.launch type:=RGI port:=/dev/ttyUSB0 prefix:=RGI; exec zsh"
gnome-terminal --tab -- zsh -c "roslaunch gripper_modbus Gripper_ModbusControl.launch type:=PGI port:=/dev/ttyUSB1 prefix:=PGI; exec zsh"

#sudo udevadm trigger
#ll /dev/ |grep ttyUSB

# 启动定位服务
gnome-terminal --tab -- zsh -c "rosrun locatornew location_service.py; exec zsh"
# 启动A_gel_start
gnome-terminal --tab -- zsh -c "roslaunch manipulator A_robot_start.launch Dual_arm_flag:=false Dashboard_flag:=true Locator_flag:=true Load_JPfile:=robot; exec zsh"
sleep 8

gnome-terminal --tab -- zsh -c "roslaunch manipulator B_JP_record.launch Record_JPfile:=robot; exec zsh"
# gnome-terminal --tab -- zsh -c "rosrun manipulator obstacle_sampler.py; exec zsh"

roslaunch js_control js_control.launch switch_mode:=1 control_mode:=0


