#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import rospy
import actionlib
import moveit_commander
import geometry_msgs.msg
from moveit_commander import PlanningSceneInterface
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class AuboMotionPlanner:
    def __init__(self):
        # 初始化 MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("arm")  # 根据实际规划组名调整

        # 设置与底层控制器一致的关节名称
        self.joint_names = [
            'shoulder_joint', 'upperArm_joint', 'foreArm_joint',
            'wrist1_joint', 'wrist2_joint', 'wrist3_joint'
        ]

        # 连接到已有的 follow_joint_trajectory action server
        self.traj_client = actionlib.SimpleActionClient(
            '/aubo_i5_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        rospy.loginfo("等待轨迹执行 action server...")
        self.traj_client.wait_for_server()
        rospy.loginfo("已连接。")

    def add_obstacles_from_bboxes(self, bboxes):
        """
        根据感知模块给出的包围盒列表，将障碍物加入规划场景
        bboxes: 列表，每个元素为 {'center': (x,y,z), 'size': (lx,ly,lz), 'frame': 'base_link'}
        """
        for i, box in enumerate(bboxes):
            name = f"obstacle_{i}"
            self.scene.add_box(
                name,
                pose=self._make_pose(box['center']),
                size=box['size']
            )
        rospy.sleep(0.5)  # 等待场景更新

    def _make_pose(self, pos):
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = "base_link"
        p.pose.position.x = pos[0]
        p.pose.position.y = pos[1]
        p.pose.position.z = pos[2]
        p.pose.orientation.w = 1.0
        return p

    def plan_to_pose(self, target_pose, velocity_scaling=0.2, accel_scaling=0.2):
        """
        规划从当前状态到 target_pose (腕部基座) 的无碰轨迹
        返回 RobotTrajectory 或 None
        """
        self.group.set_pose_target(target_pose)
        self.group.set_max_velocity_scaling_factor(velocity_scaling)
        self.group.set_max_acceleration_scaling_factor(accel_scaling)
        # 允许规划时间
        self.group.set_planning_time(5.0)

        plan = self.group.plan()
        if plan and len(plan.joint_trajectory.points) > 0:
            return plan
        else:
            rospy.logwarn("规划失败")
            return None

    def execute_trajectory(self, plan):
        """将 MoveIt 规划结果转为 FollowJointTrajectoryGoal 并执行"""
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = self.joint_names
        goal.trajectory.points = plan.joint_trajectory.points

        rospy.loginfo("发送避障轨迹，共 %d 个点", len(goal.trajectory.points))
        self.traj_client.send_goal(goal)
        finished = self.traj_client.wait_for_result(timeout=rospy.Duration(20.0))
        if finished:
            state = self.traj_client.get_state()
            rospy.loginfo("执行完成，状态码: %d", state)
            return state == 3
        else:
            rospy.logwarn("超时")
            self.traj_client.cancel_goal()
            return False

    def go_home(self):
        """回到预设的安全关节角（可根据实际情况修改）"""
        home_joints = [0.0, -0.5, 0.0, -1.0, 0.0, 0.0]
        self.group.set_joint_value_target(home_joints)
        plan = self.group.plan()
        if plan:
            self.execute_trajectory(plan)

if __name__ == '__main__':
    rospy.init_node('aubo_motion_planner_with_obstacle')
    planner = AuboMotionPlanner()

    # 示例：添加桌面和障碍物（应根据实际感知话题动态获取）
    obstacles = [
        {'center': (0.6, 0.0, -0.05), 'size': (0.8, 1.2, 0.02)},  # 桌面
        {'center': (0.5, 0.2, 0.15), 'size': (0.1, 0.1, 0.3)}    # 瓶子等
    ]
    planner.add_obstacles_from_bboxes(obstacles)

    # 抓取策略给出的腕部目标位姿（示例）
    target_pose = geometry_msgs.msg.PoseStamped()
    target_pose.header.frame_id = "base_link"
    target_pose.pose.position.x = 0.45
    target_pose.pose.position.y = 0.1
    target_pose.pose.position.z = 0.25
    target_pose.pose.orientation.w = 1.0  # 实际应从抓取模块获取

    plan = planner.plan_to_pose(target_pose)
    if plan:
        planner.execute_trajectory(plan)
    else:
        rospy.logerr("无法生成无碰轨迹")