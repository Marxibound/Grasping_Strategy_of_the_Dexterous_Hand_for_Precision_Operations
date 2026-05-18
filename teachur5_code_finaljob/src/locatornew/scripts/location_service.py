#!/usr/bin/env python3
import rospy
import json
from std_msgs.msg import String
import numpy as np
import os
import sys
import tf  # ROS 1 的 tf 模块
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import cv2
import pyrealsense2 as rs2
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Float32MultiArray
# 导入自定义服务类型
from locatornew.srv import Location, LocationRequest, LocationResponse
import capture
from analyze import *
import shutil
import traceback
import re
from icecream import ic

# 设备名称与配置文件的映射关系
deviceNameToConfigFile = {
    '121table': 'config/121光学平台.json',
    '102table': 'config/102桌面.json',
}

class Locator:
    """定位器类，同时支持Topic和Service两种请求方式"""
    def __init__(self):
        self.current_tf = {}
        self.last_tf = {}
        self.alpha = 0.3
        self.tf_timer_running = False
        self.tf_timer_thread = None
        self.start_tf_timer()  # 启动定时器

        self.config = None
        self.pose = None
        self.moveAndCapture = None
        self.pattern = None

        # 结果缓存变量
        self.cached_bTt = None  # 4x4变换矩阵
        self.cached_station = None
        self.publish_timer = None  # 10Hz发布定时器

        # 初始化ROS节点
        rospy.init_node('location_service', anonymous=False)

        print("尝试初始化节点：location_service（支持Topic和Service）")
        self.tf_broadcaster = tf.TransformBroadcaster()
        # ROS 1 中无需手动创建 Buffer，直接初始化 TransformListener
        self.tf_listener = tf.TransformListener()  # 内部已包含缓存机制
        # Topic相关
        self.obj_bTt_pub = rospy.Publisher('/obj_to_robot_holdon', PoseStamped, queue_size=10)
        self.obj_name_pub = rospy.Publisher('/class_order_holdon', String, queue_size=10)
        self.locator_sub = rospy.Subscriber('/locator_topic', String, self.locator_topic_callback)

        # Service相关
        self.locator_service = rospy.Service(
            '/locator_service',
            Location,
            self.locator_service_callback
        )

        print("节点初始化完成：location_service（支持Topic和Service）")
        print("订阅话题：/locator_topic")
        print("提供服务：/locator_service")
class Locator:
    """定位器类，同时支持Topic和Service两种请求方式"""
    def __init__(self):
        print("尝试初始化节点：location_service（支持Topic和Service）")
        self.current_tf = {}
        self.last_tf = {}
        self.alpha = 0.3
        self.tf_timer_running = False
        self.tf_timer_thread = None
        self.start_tf_timer()  # 启动定时器

        self.config = None
        self.pose = None
        self.moveAndCapture = None
        self.pattern = None

        # 结果缓存变量
        self.cached_bTt = None  # 4x4变换矩阵
        self.cached_station = None
        self.publish_timer = None  # 10Hz发布定时器

        # 初始化ROS节点
        rospy.init_node('location_service', anonymous=False)

        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()  # 内部已包含缓存机制
        
        self.camera=None
        self.camera_gTc_flag = None
        self.camera_info_topic = None
        self.camera_image_topic = None
        self.camera_info = None  # 存储CameraInfo消息
        self.gTc = None  # 相机外参（从TF获取）
        self.camera_matrix = None  # 等效于内参矩阵K
        self.dist_coeffs = None  # 等效于畸变系数D

        # Topic相关
        self.obj_bTt_pub = rospy.Publisher('/obj_to_robot_holdon', PoseStamped, queue_size=10)
        self.obj_name_pub = rospy.Publisher('/class_order_holdon', String, queue_size=10)
        self.locator_sub = rospy.Subscriber('/locator_topic', String, self.locator_topic_callback)

        # Service相关
        self.locator_service = rospy.Service(
            '/locator_service',
            Location,
            self.locator_service_callback
        )

        print("节点初始化完成：location_service（支持Topic和Service）")
        print("订阅话题：/locator_topic")
        print("提供服务：/locator_service")


    def camera_info_callback(self, msg):
        """相机信息回调：更新内参和畸变矩阵"""
        self.camera_info = msg
        # 赋值给对外暴露的camera_matrix和dist_coeffs
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.dist_coeffs = np.array(msg.D)
        rospy.logdebug("已更新相机内参和畸变矩阵")


    def get_tf_transform(self):
        """根据arm配置获取对应的TF变换（返回4x4变换矩阵）"""
        if not self.config or 'arm' not in self.config:
            rospy.logerr("未配置arm参数（left/right/single）")
            return None

        arm_type = self.config['arm']
        # 定义父坐标系和子坐标系（根据实际TF树调整）
        if arm_type == 'left':
            parent_frame = 'left_tool0'
            child_frame = 'left_camera_color_optical_frame'
        elif arm_type == 'right':
            parent_frame = 'right_tool0'
            child_frame = 'right_camera_color_optical_frame'
        elif arm_type == 'single':
            parent_frame = 'tool0'
            child_frame = 'camera_color_optical_frame'
        else:
            rospy.logerr(f"无效的arm类型: {arm_type}（应为left/right/single）")
            return None

        try:
            # 等待TF变换可用（超时5秒）
            self.tf_listener.waitForTransform(
                parent_frame,
                child_frame,
                rospy.Time(0),  # 获取最新的变换
                rospy.Duration(5.0)
            )
            
            # 获取变换（平移+旋转）
            (trans, rot) = self.tf_listener.lookupTransform(
                parent_frame,
                child_frame,
                rospy.Time(0)
            )
            trans=[t*1000 for t in trans]
            # 转换为4x4变换矩阵
            return self.trans_rot_to_matrix(trans, rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"获取TF变换失败 ({parent_frame} -> {child_frame}): {str(e)}")
            return None

    def trans_rot_to_matrix(self, trans, rot):
        """将平移向量和旋转四元数转换为4x4变换矩阵"""
        # 生成旋转矩阵（3x3）
        rotation_matrix = tf.transformations.quaternion_matrix(rot)[:3, :3]
        # 生成4x4变换矩阵
        transform_matrix = tf.transformations.identity_matrix()
        transform_matrix[:3, :3] = rotation_matrix  # 旋转部分
        transform_matrix[:3, 3] = trans  # 平移部分
        return transform_matrix


    def wait_for_camera_params(self):
        """等待相机参数和TF变换就绪（10秒超时），并以普通浮点数格式写入文件"""
        # 确保存储目录存在
        if not os.path.exists("matrix"):
            os.makedirs("matrix")

        start_time = time.time()
        timeout = 10.0  # 超时时间10秒

        # 处理相机内参
        if self.camera_info_topic != '':
            # 订阅相机信息话题
            self.camera_info_sub = rospy.Subscriber(
                self.camera_info_topic,
                CameraInfo,
                self.camera_info_callback
            )
            rospy.loginfo(f"订阅相机信息话题: {self.camera_info_topic}")
            # 循环等待内参，直到获取成功或超时
            while not rospy.is_shutdown() and self.camera_matrix is None:
                if time.time() - start_time > timeout:
                    raise TimeoutError("等待相机内参超时（10秒）")
                rospy.loginfo(f"等待相机内参话题 {self.camera_info_topic}...")
                rospy.sleep(0.5)  # 缩短等待间隔，提高响应速度

            # 内参获取成功后写入文件
            if self.camera_matrix is not None:
                matrix_path = f"matrix/{self.config['arm']}_camera_matrix.txt"
                np.savetxt(
                    matrix_path,
                    self.camera_matrix,
                    delimiter=',',
                    fmt='%.6f'
                )
                rospy.loginfo(f"相机内参已写入 {matrix_path}")

            self.dist_coeffs=[0.0]*5
            self.dist_coeffs = np.array(self.dist_coeffs).flatten()
            dist_path = f"matrix/{self.config['arm']}_dist_coeffs.txt"
            np.savetxt(
                dist_path,
                self.dist_coeffs,
                delimiter=',',
                fmt='%.6f'
            )
            rospy.loginfo(f"畸变系数已写入 {dist_path}")
            self.camera_info_topic = ''  # 标记为已处理
        else:
            # 从文件加载内参（立即加载，不等待）
            try:
                self.camera_matrix = np.loadtxt(f"matrix/{self.config['arm']}_camera_matrix.txt", delimiter=',')
                self.dist_coeffs = np.loadtxt(f"matrix/{self.config['arm']}_dist_coeffs.txt", delimiter=',')
            except FileNotFoundError:
                raise FileNotFoundError("相机内参文件不存在，请先通过话题获取并保存")

        # 重置超时计时（单独计算外参等待时间）
        start_time_gTc = time.time()

        # 处理TF变换（外参gTc）
        if self.camera_gTc_flag:
            # 循环等待TF变换，直到获取成功或超时
            self.gTc = None
            while self.gTc is None:
                if time.time() - start_time_gTc > timeout:
                    raise TimeoutError("等待相机外参（TF变换）超时（10秒）")
                self.gTc = self.get_tf_transform()
                rospy.loginfo("等待相机外参（TF变换）...")
                rospy.sleep(0.5)

            # 外参获取成功后写入文件
            if self.gTc is not None:
                gt_path = f"matrix/{self.config['arm']}_gTc.txt"
                np.savetxt(
                    gt_path,
                    self.gTc,
                    delimiter=',',
                    fmt='%.10f'
                )
                rospy.loginfo(f"TF变换（gTc）已写入 {gt_path}")
            self.camera_gTc_flag = ''  # 标记为已处理
        else:
            # 从文件加载外参
            try:
                self.gTc = np.loadtxt(f"matrix/{self.config['arm']}_gTc.txt", delimiter=',')
            except FileNotFoundError:
                raise FileNotFoundError("相机外参文件不存在，请先通过TF获取并保存")

        rospy.loginfo("相机参数和TF变换已就绪")


    @staticmethod
    def transform_to_matrix(transform):
        """将TF的Transform消息转换为4x4变换矩阵"""
        # 平移向量
        t = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ])
        # 旋转四元数 -> 旋转矩阵
        q = transform.transform.rotation
        rot = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()  # 3x3旋转矩阵

        # 构造4x4变换矩阵
        matrix = np.eye(4)
        matrix[:3, :3] = rot
        matrix[:3, 3] = t
        return matrix
    
    def start_tf_timer(self):
        """启动独立的TF发布定时器线程"""
        if not self.tf_timer_running:
            self.tf_timer_running = True
            self.tf_timer_thread = threading.Thread(target=self.tf_timer_loop, daemon=True)
            self.tf_timer_thread.start()
            rospy.loginfo("TF发布定时器已启动")
    def tf_timer_loop(self):
        """定时器循环，使用alpha系数滤波器发布平滑的TF变换"""
        rate = 30  # 30Hz发布频率
        while self.tf_timer_running and not rospy.is_shutdown():
            for key in list(self.current_tf.keys()):
                new_transform = self.current_tf[key]
                
                # 应用alpha系数滤波
                filtered_transform = self.alpha_filter(key, new_transform)
                
                # 更新时间戳并发布
                filtered_transform.header.stamp = rospy.Time.now()

                translation = (filtered_transform.transform.translation.x,
                            filtered_transform.transform.translation.y,
                            filtered_transform.transform.translation.z)
                rotation = (filtered_transform.transform.rotation.x,
                            filtered_transform.transform.rotation.y,
                            filtered_transform.transform.rotation.z,
                            filtered_transform.transform.rotation.w)
                timestamp = rospy.Time.now()  # 或使用原时间戳 filtered_transform.header.stamp
                child_frame = filtered_transform.child_frame_id  # "g_camera_color_optical_frame"
                parent_frame = filtered_transform.header.frame_id  # "single_tool0"

                # 正确调用 sendTransform
                self.tf_broadcaster.sendTransform(
                    translation,
                    rotation,
                    timestamp,
                    child_frame,
                    parent_frame
                )
            
            time.sleep(1.0 / rate)

    def alpha_filter(self, key, new_transform):
        """
        指数移动平均滤波：filtered = alpha * new + (1 - alpha) * last
        :param key: TF变换的标识键
        :param new_transform: 新的原始TF变换
        :return: 滤波后的TF变换
        """
        # 若为首次处理该TF，直接使用新值作为初始值
        if key not in self.last_tf:
            self.last_tf[key] = new_transform
            return new_transform
        
        # 获取上一次的滤波结果
        last = self.last_tf[key]
        filtered = geometry_msgs.msg.TransformStamped()
        filtered.header = new_transform.header
        filtered.child_frame_id = new_transform.child_frame_id
        
        # 平移分量滤波：alpha×新值 + (1-alpha)×旧值
        filtered.transform.translation.x = self.alpha * new_transform.transform.translation.x + \
                                        (1 - self.alpha) * last.transform.translation.x
        filtered.transform.translation.y = self.alpha * new_transform.transform.translation.y + \
                                        (1 - self.alpha) * last.transform.translation.y
        filtered.transform.translation.z = self.alpha * new_transform.transform.translation.z + \
                                        (1 - self.alpha) * last.transform.translation.z
        
        # 旋转分量（四元数）滤波：同样使用指数加权平均，最后归一化
        filtered.transform.rotation.x = self.alpha * new_transform.transform.rotation.x + \
                                    (1 - self.alpha) * last.transform.rotation.x
        filtered.transform.rotation.y = self.alpha * new_transform.transform.rotation.y + \
                                    (1 - self.alpha) * last.transform.rotation.y
        filtered.transform.rotation.z = self.alpha * new_transform.transform.rotation.z + \
                                    (1 - self.alpha) * last.transform.rotation.z
        filtered.transform.rotation.w = self.alpha * new_transform.transform.rotation.w + \
                                    (1 - self.alpha) * last.transform.rotation.w
        
        # 归一化四元数（确保旋转分量合法性）
        norm = np.sqrt(
            filtered.transform.rotation.x**2 +
            filtered.transform.rotation.y**2 +
            filtered.transform.rotation.z**2 +
            filtered.transform.rotation.w**2
        )
        if norm > 0:
            filtered.transform.rotation.x /= norm
            filtered.transform.rotation.y /= norm
            filtered.transform.rotation.z /= norm
            filtered.transform.rotation.w /= norm
        
        # 更新上一次的滤波结果
        self.last_tf[key] = filtered
        return filtered
    def add_tf(self, key, value):
        self.current_tf[key]=value
    def remove_tf(self, key):
        """移除不需要发布的TF变换"""
        if key in self.current_tf:
            del self.current_tf[key]
            rospy.loginfo(f"已停止发布TF变换: {key}")
    def shutdown(self):
        """关闭定时器线程"""
        if self.publish_timer is not None and self.publish_timer.is_alive():
            self.publish_timer.shutdown()
            rospy.loginfo("10Hz发布定时器已停止")
        self.tf_timer_running = False
        if self.tf_timer_thread:
            self.tf_timer_thread.join()
        rospy.loginfo("TF发布定时器已关闭")
    def __del__(self):
        self.shutdown()

    def oneloc(self):
        """单次识别逻辑"""
        print('~~~~~~~~开始单次识别~~~~~~~~')
        try:
            self.moveAndCapture.savePhotoAndPose(0, True)
            result_bTt = self.pattern.realtime(
                self.moveAndCapture.camera.saveFrameTo('./img_take/oneloc.png'),
                self.moveAndCapture.robot.getPoseBase(self.moveAndCapture.arm_short)
            )
            print(result_bTt)
            
            # 处理Aruco字典格式
            if isinstance(result_bTt, dict) and len(result_bTt) > 0:
                result_bTt = next(iter(result_bTt.values()))
            print(f"~~~~~~~~结束单次识别~~~~~~~~")
            return result_bTt
        except Exception as e:
            rospy.logerr(f"单次识别失败：{traceback.format_exc()}")
            return None

    def realtime(self):
        """实时识别逻辑"""
        print('~~~~~~~~开始实时识别~~~~~~~~')
        try:
            while not rospy.is_shutdown():
                result_bTt = self.pattern.realtime(
                    self.moveAndCapture.camera.saveFrameTo('./img_take/oneloc.png'),
                    self.moveAndCapture.robot.getPoseBase(self.moveAndCapture.arm_short)
                )
            print(f"~~~~~~~~结束实时识别~~~~~~~~")
        except Exception as e:
            rospy.logerr(f"实时识别失败：{traceback.format_exc()}")

    def publish_holdon_data(self, event):
        """10Hz定时器回调：发布缓存的位姿和站点信息"""
        if self.cached_bTt is None or self.cached_station is None or self.cached_bTt=={}:
            rospy.logwarn_throttle(1, "无有效缓存的bTt或station，跳过发布")
            return
        
        # 转换bTt为PoseStamped消息
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "world"
        pose_msg.header.stamp = rospy.Time.now()
        
        # 位置信息
        pose_msg.pose.position.x = self.cached_bTt[0, 3]
        pose_msg.pose.position.y = self.cached_bTt[1, 3]
        pose_msg.pose.position.z = self.cached_bTt[2, 3]
        
        # 姿态信息（旋转矩阵转四元数）
        rot_matrix = self.cached_bTt[:3, :3]
        quat = R.from_matrix(rot_matrix).as_quat()
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        # 发布消息
        self.obj_bTt_pub.publish(pose_msg)
        self.obj_name_pub.publish(self.cached_station)

    def merge_configs(self, base_config, override_config):
        """递归合并配置字典"""
        merged = base_config.copy()
        for key, value in override_config.items():
            if isinstance(value, dict) and key in merged and isinstance(merged[key], dict):
                merged[key] = self.merge_configs(merged[key], value)
            else:
                merged[key] = value
        return merged
    def _camera_generator(self):
        # 从参数服务器获取话题名
        self.camera_image_topic = rospy.get_param("~camera_image_topic", "")
        if self.camera_info_topic is None:
            self.camera_info_topic = rospy.get_param("~camera_info_topic", "")
        if self.camera_gTc_flag is None:
            self.camera_gTc_flag = rospy.get_param("~camera_gTc_flag", False)


        # 等待相机信息和TF变换就绪后再初始化Camera
        self.wait_for_camera_params()
        self.camera = capture.Camera(
            self.camera_image_topic,
            self.gTc,
            self.camera_matrix,
            self.dist_coeffs
        )
    def _common_location_logic(self, command_dict):
        """通用定位逻辑（Topic和Service共用）"""
        try:
            # 检查station参数
            if 'station' not in command_dict:
                status = "error: the command requires 'station' value"
                rospy.logerr(status)
                return (status, False, None)
            station = command_dict['station']
            self.cached_station = station

            # 检查设备是否在配置中
            if station not in deviceNameToConfigFile:
                available_stations = list(deviceNameToConfigFile.keys())
                status = f"error: unknown station '{station}', available stations: {available_stations}"
                rospy.logerr(status)
                return (status, False, None)
            deviceConfigFile = deviceNameToConfigFile[station]

            # 读取基础配置文件
            try:
                with open(deviceConfigFile, 'r', encoding='utf-8') as f:
                    base_config = json.load(f)
                rospy.loginfo(f"loaded basic config from {deviceConfigFile}")
            except Exception as e:
                status = f"error: failed to load basic config - {str(e)}"
                rospy.logerr(status)
                return (status, False, None)

            # 合并配置
            self.config = self.merge_configs(base_config, command_dict)
            print(json.dumps(self.config, indent=4, ensure_ascii=False))

            # 初始化移动捕获和标定板
            self._camera_generator()
            self.moveAndCapture = capture.MoveAndCapture(self.config,self.camera)
            marker_type = self.config.get('type', '')
            task = self.config.get('task', '')

            # 初始化标定板类型
            if marker_type == 'aruco':
                self.pattern = Aruco(self.config,self.add_tf)
            elif marker_type == 'charuco':
                self.pattern = Charuco(self.config,self.add_tf)
            elif marker_type == 'circleGrid':
                self.pattern = CircleGrid(self.config,self.add_tf)
            else:
                status = f"error: unknown board type '{marker_type}'"
                rospy.logerr(status)
                return (status, False, None)

            self.moveAndCapture.publish_tf = self.pattern.publish_tf
            self.moveAndCapture.gTc = self.pattern.gTc
            print(f'~~~~~config info ({deviceConfigFile})~~~~~')

            # 处理不同任务
            task_result_bTt = None
            if task=="multiloc" and self.config['multiloc']['allNumber']==0:
                task='oneloc'
                self.config['task']='oneloc'
            if task == 'oneloc':
                task_result_bTt = self.oneloc()
                # if task_result_bTt is None or task_result_bTt=={}:
                if task_result_bTt is None:
                    status = "error: single-location failed (no valid bTt)"
                    rospy.logerr(status)
                    return (status, False, None)
                status = "success: single-location completed"

            elif task == 'realtime':
                self.realtime()
                status = "success: realtime location started (no cached bTt)"
                return (status, False, None)

            elif task in ['multiloc', 'calib']:
                # 执行单次识别获取初始bTt
                oneloc_bTt = self.oneloc()
                if oneloc_bTt is None:
                    status = "error: multi-location/calibration failed (initial single-location failed)"
                    rospy.logerr(status)
                    return (status, False, None)
                if isinstance(oneloc_bTt, dict) and len(oneloc_bTt) > 0:
                    oneloc_bTt = next(iter(oneloc_bTt.values()))

                # 移动捕获
                self.moveAndCapture.moveAndCaptureAll(oneloc_bTt)

                # 处理多次定位
                if task == 'multiloc':
                    task_result_bTt = self.pattern.multiloc()
                    if task_result_bTt is None:
                        status = "error: multi-location failed (no valid average bTt)"
                        rospy.logerr(status)
                        return (status, False, None)
                    status = "success: multi-location completed"

                # 处理标定
                elif task == 'calib':
                    gTc = self.pattern.calib()
                    if gTc is None:
                        status = "error: calibration failed"
                        rospy.logerr(status)
                        return (status, False, None)
                    if self.config.get('calib', {}).get('overwrite', False):
                        self.write_gTc_to_xacro(gTc)
                    status = "success: calibration completed (no cached bTt)"
                    return (status, False, None)

            else:
                status = f"error: unknown task '{task}' (supported: oneloc/multiloc/calib/realtime)"
                rospy.logerr(status)
                return (status, False, None)

            # 缓存结果并启动发布定时器
            self.cached_bTt = task_result_bTt
            print("a")
            if self.publish_timer is None or not self.publish_timer.is_alive():
                self.publish_timer = rospy.Timer(rospy.Duration(0.1), self.publish_holdon_data)
                rospy.loginfo("10Hz publishing timer started (publishing obj_to_robot_holdon and class_order_holdon)")
            print('b')

            return (status, True, task_result_bTt)

        except Exception as e:
            status = f"error: failed to execute common logic - {str(e)}\n{traceback.format_exc()}"
            rospy.logerr(status)
            return (status, False, None)

    def locator_topic_callback(self, _data):
        """Topic回调：处理/locator_topic话题请求"""
        rospy.loginfo("\n收到Topic请求：/locator_topic")
        try:
            command_dict = json.loads(_data.data)
            rospy.loginfo(f"Topic命令内容：{command_dict}")
        except json.JSONDecodeError as e:
            rospy.logerr(f"解析Topic命令失败：{str(e)}，原始数据：{_data.data}")
            return

        # 调用通用逻辑
        status, _, _ = self._common_location_logic(command_dict)
        self.camera.__del__()
        rospy.loginfo(f"Topic请求处理结果：{status}")

    def locator_service_callback(self, req):
        """Service回调：处理/locator_service服务请求"""
        rospy.loginfo("\n收到Service请求：/locator_service")
        resp = LocationResponse()  # 初始化响应对象

        try:
            # 解析服务请求参数
            command_dict = json.loads(req.command_json)
            rospy.loginfo(f"Service命令内容：{command_dict}")
        except json.JSONDecodeError as e:
            resp.status = f"illegal json - {str(e)}"
            resp.result_valid = False
            resp.cached_bTt = []
            resp.cached_station = ""
            return resp

        # 调用通用逻辑
        status, result_valid, task_result_bTt = self._common_location_logic(command_dict)
        self.camera.__del__()

        # 构造服务响应
        resp.status = status
        resp.result_valid = result_valid
        resp.cached_station = self.cached_station if self.cached_station else ""

        # 处理bTt矩阵（展平为16元素列表）
        if result_valid and task_result_bTt is not None:
            try:
                # 确保bTt是4x4矩阵
                if isinstance(task_result_bTt, np.ndarray) and task_result_bTt.shape == (4, 4):
                    resp.cached_bTt = task_result_bTt.flatten().tolist()
                else:
                    resp.cached_bTt = []
                    resp.status += "warning,the bTt is not 4x4 matrix"
            except Exception as e:
                resp.cached_bTt = []
                resp.status += f"bTt trans failed：{str(e)}"

        return resp

    def write_gTc_to_xacro(self, gTc):
        """写入标定结果到Xacro文件"""
        try:
            arm_robot_description = os.popen("rospack find arm_robot_description").read().strip()
            if self.config.get('arm', 'left') == 'single':
                xacro_path = os.path.join(arm_robot_description, "urdf", "single_arm_robot.xacro")
            else:
                xacro_path = os.path.join(arm_robot_description, "urdf", "dual_arm_robot.xacro")
            if not os.path.exists(xacro_path):
                rospy.logerr(f"Xacro文件不存在: {xacro_path}")
                return False
            backup_path = f"{xacro_path}.bak"
            shutil.copy2(xacro_path, backup_path)
            rospy.loginfo(f"已备份原文件至: {backup_path}")
        except Exception as e:
            rospy.logerr(f"文件准备失败: {e}")
            return False

        # 解析gTc矩阵
        xyz = [round(v, 6)/1000 for v in gTc[:3, 3].tolist()]
        rot_matrix = gTc[:3, :3]
        r = R.from_matrix(rot_matrix)
        rpy = [round(v, 6) for v in r.as_euler('xyz', degrees=False)]

        # 目标关节名称
        if self.config.get('arm', 'left') == 'left':
            target_joint_name = "${prefix2}tool0_to_camera_color_optical_frame"
        elif self.config.get('arm', 'left') == 'right':
            target_joint_name = "${prefix1}tool0_to_camera_color_optical_frame"
        else:
            target_joint_name = "tool0_to_camera_color_optical_frame"
        target_joint_found = False

        # 处理文件内容
        new_content = []
        try:
            with open(backup_path, 'r') as f:
                lines = f.readlines()

            in_target_joint = False
            in_origin_block = False
            
            for line in lines:
                if f'joint name="{target_joint_name}"' in line and 'type="fixed"' in line:
                    target_joint_found = True
                    in_target_joint = True
                    new_content.append(line)
                    continue

                if in_target_joint:
                    if "<origin" in line and "/>" not in line:
                        in_origin_block = True
                        new_content.append(line)
                        continue
                    
                    if in_origin_block:
                        if "xyz=" in line:
                            new_line = re.sub(
                                r'xyz="[^"]*"',
                                f'xyz="{xyz[0]} {xyz[1]} {xyz[2]}"',
                                line
                            )
                            new_content.append(new_line)
                        elif "rpy=" in line:
                            new_line = re.sub(
                                r'rpy="[^"]*"',
                                f'rpy="{rpy[0]} {rpy[1]} {rpy[2]}"',
                                line
                            )
                            new_content.append(new_line)
                        elif "/>" in line or "</origin>" in line:
                            new_content.append(line)
                            in_origin_block = False
                        else:
                            new_content.append(line)
                        continue

                    new_content.append(line)
                    
                    if "</joint>" in line:
                        in_target_joint = False
                else:
                    new_content.append(line)

            # 未找到关节时添加新关节
            if not target_joint_found:
                rospy.loginfo(f"未找到关节{target_joint_name}，添加新关节")
                if self.config.get('arm', 'left')=='single':
                    new_joint = [
                        f'  <joint name="{target_joint_name}" type="fixed">\n',
                        f'    <origin\n',
                        f'      xyz="{xyz[0]} {xyz[1]} {xyz[2]}"\n',
                        f'      rpy="{rpy[0]} {rpy[1]} {rpy[2]}" />\n',
                        f'    <parent\n',
                        f'      link="tool0" />\n',
                        f'    <child\n',
                        f'      link="camera_color_optical_frame" />\n',
                        f'    <axis\n',
                        f'      xyz="0 0 0" />\n',
                        f'  </joint>\n'
                    ]
                else:
                    new_joint = [
                        f'  <joint name="{target_joint_name}" type="fixed">\n',
                        f'    <origin\n',
                        f'      xyz="{xyz[0]} {xyz[1]} {xyz[2]}"\n',
                        f'      rpy="{rpy[0]} {rpy[1]} {rpy[2]}" />\n',
                        f'    <parent\n',
                        f'      link="${{prefix}}tool0" />\n',
                        f'    <child\n',
                        f'      link="${{prefix}}camera_color_optical_frame" />\n',
                        f'    <axis\n',
                        f'      xyz="0 0 0" />\n',
                        f'  </joint>\n'
                    ]
                # 插入到xacro:macro结束前
                for i in range(len(new_content)):
                    if "</xacro:macro>" in new_content[i]:
                        new_content = new_content[:i] + new_joint + new_content[i:]
                        break

            # 写入文件
            if self.config.get('calib', {}).get('overwrite', False):
                with open(xacro_path, 'w') as f:
                    f.writelines(new_content)
                rospy.loginfo(f"已写入新Xacro文件: {xacro_path}")
                return True
            else:
                rospy.loginfo("overwrite为false，不写入文件")
                return True

        except Exception as e:
            rospy.logerr(f"写入失败: {traceback.format_exc()}")
            shutil.copy2(backup_path, xacro_path)
            return False



if __name__ == '__main__':
    """主函数：启动节点并保持运行"""
    try:
        locator = Locator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        locator.shutdown()
    