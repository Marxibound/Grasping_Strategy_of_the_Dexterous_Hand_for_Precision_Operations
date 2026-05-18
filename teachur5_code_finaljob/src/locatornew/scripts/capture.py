#!/usr/bin/env python3
import numpy as np
import time
import json
import rospy
import time
import pyrealsense2 as rs2  # RealSense相机SDK，用于图像采集
import cv2  # 图像处理库，用于图像保存
from tqdm import tqdm  # 进度条库（当前未使用，预留用于批量任务）
import sys
import csv  # 用于保存机械臂位姿数据到CSV格式
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import math
from scipy.spatial.transform import Rotation as R  # 旋转矩阵处理库
from icecream import ic  # 调试工具，用于打印变量详情
from robot_controller import Robot  # 自定义机械臂控制类（封装机械臂运动API）
from calculate_rotate import *  # 自定义坐标变换/旋转计算模块
from sensor_msgs.msg import Image  # ROS图像消息类型
from cv_bridge import CvBridge  # ROS图像与OpenCV图像转换工具

# ------------------------------ Camera类：RealSense相机控制 ------------------------------
class Camera:
    """
    通用相机控制类：
    - 支持从rosparam读取图像话题名称（参数名：camera_image_topic）
    - 若话题为"realsense"，使用RealSense SDK（rs2）直接获取图像
    - 若为其他话题（如OAK相机的/stereo_inertial_publisher/color/image），从ROS话题订阅获取图像
    实现功能：相机初始化、图像捕获、图像保存
    """

    def __init__(self,camera_image_topic,gTc,camera_matrix,dist_coeffs):
        """初始化相机：读取rosparam配置，根据话题类型选择初始化方式"""
        # 1. 初始化ROS节点（若外部未初始化，此处自动初始化，避免订阅失败）
        if not rospy.core.is_initialized():
            rospy.init_node("camera_controller", anonymous=True)
        
        # 2. 从rosparam读取图像话题名称，默认值设为"realsense"（兼容原RealSense逻辑）
        self.camera_image_topic = camera_image_topic
        self.gTc = gTc
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        rospy.loginfo(f"当前相机图像配置：camera_image_topic = {self.camera_image_topic}")

        # 3. 初始化核心变量
        self.pipe = None  # RealSense流水线对象（仅realsense模式使用）
        self.bridge = CvBridge()  # ROS-OpenCV图像转换器（仅话题订阅模式使用）
        self.latest_frame = None  # 缓存最新图像帧（话题订阅模式：避免阻塞）
        self.frame_lock = False  # 帧同步锁（防止多线程数据冲突）

        # 4. 根据话题类型初始化相机
        if self.camera_image_topic=='':
            # 模式1：RealSense直接连接（使用rs2 SDK）
            self._init_realsense()
        else:
            # 模式2：ROS话题订阅（如OAK相机的/stereo_inertial_publisher/color/image）
            self._init_ros_subscriber()

    def _init_realsense(self):
        """私有方法：初始化RealSense相机（原rs2逻辑）"""
        try:
            self.pipe = rs2.pipeline()
            config = rs2.config()
            # 配置RGB流：1920x1080分辨率，BGR8格式（OpenCV兼容），30fps
            config.enable_stream(rs2.stream.color, 1920, 1080, rs2.format.bgr8, 30)
            self.profile = self.pipe.start(config)
            rospy.loginfo("RealSense相机初始化成功，已启动RGB流")
        except Exception as e:
            rospy.logerr(f"RealSense相机初始化失败: {str(e)}")
            self.pipe = None

    def _init_ros_subscriber(self):
        """私有方法：初始化ROS图像话题订阅（如OAK相机）"""
        try:
            # 订阅目标图像话题，回调函数更新最新帧
            self.subscriber = rospy.Subscriber(
                self.camera_image_topic,  # 从rosparam获取的话题名（如/stereo_inertial_publisher/color/image）
                Image,             # ROS图像消息类型
                self._ros_frame_callback,  # 回调函数
                queue_size=1       # 队列大小：仅缓存最新1帧，避免延迟
            )
            # 等待1秒，确保订阅成功并获取第一帧
            rospy.sleep(1)
            if self.latest_frame is None:
                rospy.logwarn(f"话题 {self.camera_image_topic} 已订阅，但暂未收到图像帧（请检查话题是否有数据）")
            else:
                rospy.loginfo(f"ROS话题 {self.camera_image_topic} 订阅成功，已获取最新图像帧")
        except Exception as e:
            rospy.logerr(f"ROS话题订阅初始化失败: {str(e)}")
            self.subscriber = None

    def _ros_frame_callback(self, msg):
        """私有方法：ROS图像话题回调函数（更新最新帧缓存）"""
        if self.frame_lock:
            return  # 若当前帧正在被读取，跳过本次回调（避免数据冲突）
        try:
            # 将ROS Image消息转换为OpenCV格式（BGR8：与RealSense输出格式一致，保证接口统一）
            cv_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_frame = cv_frame  # 缓存最新帧
        except Exception as e:
            rospy.logerr(f"ROS图像转换失败: {str(e)}")

    def saveFrameTo(self, path):
        """
        捕获一帧图像并保存到指定路径（两种模式通用）
        :param path: 图像保存路径（如"./img_take/0.png"）
        :return: 成功返回OpenCV格式图像（numpy数组），失败返回None
        """
        # 1. 获取当前帧（区分两种模式）
        if self.camera_image_topic.lower() == "":
            # RealSense模式：直接从流水线获取
            if self.pipe is None:
                rospy.logerr("RealSense相机未初始化，无法保存帧")
                return None
            try:
                frames = self.pipe.wait_for_frames()
                cv_frame = np.asarray(frames.get_color_frame().get_data())
            except Exception as e:
                rospy.logerr(f"RealSense帧获取失败: {str(e)}")
                return None
        else:
            # ROS话题模式：从缓存获取最新帧
            if self.latest_frame is None:
                rospy.logerr(f"未从话题 {self.camera_image_topic} 收到图像帧，无法保存")
                return None
            # 加锁：防止读取时回调函数更新帧（避免数据截断）
            self.frame_lock = True
            cv_frame = self.latest_frame.copy()  # 复制帧数据，避免原数据被修改
            self.frame_lock = False

        # 2. 保存图像
        try:
            cv2.imwrite(path, cv_frame)
            rospy.loginfo(f"图像已保存到: {path}")
            return cv_frame
        except Exception as e:
            rospy.logerr(f"图像保存失败: {str(e)}")
            return None

    def getRGBFrame(self):
        """
        获取实时RGB帧（两种模式通用，返回OpenCV格式，便于后续处理）
        :return: 成功返回OpenCV图像（numpy数组，BGR8格式），失败返回None
        """
        if self.camera_image_topic.lower() == "":
            # RealSense模式：直接获取原始帧并转换为OpenCV格式
            if self.pipe is None:
                rospy.logerr("RealSense相机未初始化，无法获取帧")
                return None
            try:
                frames = self.pipe.wait_for_frames()
                cv_frame = np.asarray(frames.get_color_frame().get_data())
                return cv_frame
            except Exception as e:
                rospy.logerr(f"RealSense帧获取失败: {str(e)}")
                return None
        else:
            # ROS话题模式：从缓存获取最新帧
            if self.latest_frame is None:
                rospy.logwarn(f"未从话题 {self.camera_image_topic} 收到图像帧")
                return None
            # 加锁复制，确保数据完整性
            self.frame_lock = True
            cv_frame = self.latest_frame.copy()
            self.frame_lock = False
            return cv_frame

    def __del__(self):
        """析构函数：释放资源（避免内存泄漏）"""
        # 1. 停止RealSense流水线
        print('释放相机资源')
        if self.pipe is not None:
            self.pipe.stop()
            rospy.loginfo("RealSense相机流水线已停止")
        # 2. 取消ROS话题订阅
        if hasattr(self, "subscriber") and self.subscriber is not None:
            self.subscriber.unregister()
            rospy.loginfo(f"ROS话题 {self.camera_image_topic} 订阅已取消")
        # 3. 关闭ROS节点（若当前节点是本类初始化的）
        if rospy.core.is_initialized() and rospy.get_name() == "/camera_controller":
            rospy.signal_shutdown("相机控制对象已销毁，关闭节点")


# ------------------------------ MoveAndCapture类：机械臂运动+图像捕获协同控制 ------------------------------
class MoveAndCapture:
    """机械臂运动与相机捕获协同类，根据配置文件控制机械臂移动到目标位姿并拍照，记录位姿数据"""
    
    def __init__(self, config, camera):
        """
        初始化协同控制对象
        :param bTt: 基础坐标系到目标坐标系的变换矩阵（4x4，用于坐标转换）
        """
        # 1. 创建图像保存目录（./img_take），用于存储拍摄的图像和位姿数据
        file_path = "./img_take"
        self.file_path = file_path
        if not os.path.exists(file_path):
            os.mkdir(file_path)  # 目录不存在则创建

        # 2. 清空目录中之前的文件（避免旧数据干扰当前任务）
        for file in os.listdir(file_path):
            os.remove(f'{file_path}/{file}')  # 删除目录下所有文件

        # 3. 初始化位姿数据文件（pose.txt），用于记录每个拍摄点的机械臂位姿
        fpose = open(f'{file_path}/pose.txt', 'w')
        self.fpose = fpose  # 保存文件句柄，供后续写入
        fpose.write('id,px,py,pz,rx,ry,rz\r\n')  # 写入CSV表头：id（拍摄序号）+ 位姿（x/y/z/rx/ry/rz）
        self.csvwriter = csv.writer(fpose)  # 创建CSV写入器，方便按行写入数据

        # 4. 初始化机械臂控制对象（通过Robot类封装机械臂运动指令）
        self.robot = Robot()
        # 5. 关联全局相机对象（复用之前初始化的RealSense相机）
        self.camera = camera

        # 6. 存储机械臂位姿变化量（用于后续计算运动精度，如均值、标准差）
        self.allDeltaPose = []
        # 7. 读取任务配置文件（拍摄范围、任务类型等）
        self.config=config
        if self.config.get('matrix') is None:
            self.config['matrix'] = 'A1'
        if self.config.get('moveP_vel') is None:
            self.config['moveP_vel']=0.1
        if self.config.get('moveP_acc') is None:
            self.config['moveP_acc']=0.1
        self.publish_tf=None
        self.arm_short=self.config['arm'][0].upper()
        self.offset=[self.config['offset']['x'],self.config['offset']['y'],self.config['offset']['z']]
    def readConfig(self, file):
        """
        读取任务配置文件（JSON格式），解析拍摄参数
        :param file: 配置文件路径
        """
        with open(file, 'r') as f:
            config = json.load(f)  # 加载JSON配置
        print('~~~~~~~~拍摄参数~~~~~~')
        ic(file)  # 调试打印：配置文件路径
        ic(config)  # 调试打印：完整配置内容（如任务类型、拍摄范围、点数）
        self.config = config  # 保存配置到类属性

        # 兼容配置：若配置中无"matrix"字段（坐标系标识），默认设为"A1"
        if self.config.get('matrix') is None:
            self.config['matrix'] = 'A1'
        if self.config.get('moveP_vel') is None:
            self.config['moveP_vel']=0.1
        if self.config.get('moveP_acc') is None:
            self.config['moveP_acc']=0.1

    def generate_all_xyz(self, config):
        """
        根据任务类型（标定/calib 或 定位/loc）生成所有目标点的XYZ坐标（目标坐标系下）
        :param config: 任务配置字典
        :return: allXYZ - 目标点列表（每个元素为[x, y, z]，单位：毫米）
        """
        config = self.config
        task = config['task']
        # 从配置中提取拍摄范围（目标坐标系下，单位：毫米）
        minX = config[task]['minX']  # X轴最小范围
        maxX = config[task]['maxX']  # X轴最大范围
        minY = config[task]['minY']  # Y轴最小范围
        maxY = config[task]['maxY']  # Y轴最大范围
        minZ = config[task]['minZ']  # Z轴最小范围
        maxZ = config[task]['maxZ']  # Z轴最大范围

        # 提取任务参数（标定任务需固定点数，定位任务需随机点数）
        pointNumX = config['calib']['pointNumX']  # 标定任务X轴点数
        pointNumY = config['calib']['pointNumY']  # 标定任务Y轴点数
        pointNumZ = config['calib']['pointNumZ']  # 标定任务Z轴点数
        allNumber = config['multiloc']['allNumber']    # 定位任务总点数

        # 校验任务类型（仅支持calib/loc）
        assert task in ['calib', 'multiloc'], "任务类型错误，仅支持'calib'或'multiloc'"
        allXYZ = []

        if task == 'calib':
            # 标定任务：生成均匀分布的网格点（X→Y→Z顺序遍历）
            allX = np.linspace(minX, maxX, pointNumX)  # X轴均匀取点
            allY = np.linspace(minY, maxY, pointNumY)  # Y轴均匀取点
            allZ = np.linspace(minZ, maxZ, pointNumZ)  # Z轴均匀取点
            # 三维网格遍历：先Z轴，再X轴，最后Y轴（确保覆盖整个拍摄空间）
            for z in allZ:
                for x in allX:
                    for y in allY:
                        allXYZ.append([x, y, z])
        else:  # task == 'multiloc'
            # 定位任务：生成随机分布的点（覆盖指定范围）
            for i in range(allNumber):
                allXYZ.append([
                    np.random.uniform(minX, maxX),  # X轴随机值
                    np.random.uniform(minY, maxY),  # Y轴随机值
                    np.random.uniform(minZ, maxZ),  # Z轴随机值
                ])
        return allXYZ

    def savePhotoAndPose(self, nums=0, oneloc=False):
        """
        拍摄图像并记录机械臂位姿（含运动精度计算）
        :param nums: 拍摄序号（用于图像命名和位姿记录ID）
        """
        # 1. 获取机械臂运动前的位姿（基础坐标系下，单位：米/弧度）
        initial_state = self.robot.getPoseBase(self.arm_short)
        # 2. 拍摄图像并保存（命名格式：序号.png，如0.png）
        image_name='oneloc.png' if oneloc else f'multiloc_{nums}.png'
        self.camera.saveFrameTo(f'{self.file_path}/{image_name}')
        # 3. 获取机械臂运动后的位姿（用于计算运动前后的偏差）
        final_state = self.robot.getPoseBase(self.arm_short)

        # 4. 计算位姿变化量（转换为毫米单位，方便观察）
        delta_state = (np.array(final_state) - np.array(initial_state)) * 1000
        self.allDeltaPose.append(delta_state)  # 保存变化量，用于后续统计

        # 5. 计算运动前后的平均位姿（作为当前拍摄点的最终位姿）
        current_state = list(np.mean([initial_state, final_state], axis=0))
        # ic(current_state)  # 调试打印：当前拍摄点的平均位姿
        # 6. 将拍摄序号和最终位姿写入CSV文件（id, px, py, pz, rx, ry, rz）
        self.csvwriter.writerow([nums] + list(final_state))
        print(f'成功保存图片"./img_take/{image_name}与位姿')

    def moveAndTakePhoto(self, nums, targetXYZ_tTc):
        """
        核心函数：计算机械臂目标位姿→控制机械臂移动→拍照→记录位姿
        :param nums: 拍摄序号
        :param targetXYZ_tTc: 目标坐标系下的XYZ坐标（单位：毫米）
        """
        # ic(nums, targetXYZ_tTc)  # 调试打印：拍摄序号和目标点（目标坐标系下）

        # 1. 坐标变换：将目标坐标系下的点转换为机械臂基础坐标系下的点+姿态
        # （调用自定义函数transform_xyz_tTc_to_bTc，输入：目标点、bTt变换矩阵、坐标系标识）
        target_bTg = transform_xyz_tTc_to_bTg(targetXYZ_tTc, self.bTt, self.camera.gTc, self.config['matrix'], self.camera.camera_matrix, self.camera.dist_coeffs)
        # 7. 从齐次变换矩阵中提取平移向量和旋转向量（格式适配机械臂控制）
        # 提取平移向量：齐次矩阵上右3x1部分，展平为1D数组
        point = target_bTg[:3, 3].flatten()/1000
        # 提取旋转矩阵并转换为旋转向量（Rodrigues向量）：
        # 'xyz' 表示旋转顺序：绕X轴（Roll）→ 绕Y轴（Pitch）→ 绕Z轴（Yaw），与 xyzrpyToPose 一致
        rpy_rad = R.from_matrix(target_bTg[:3, :3]).as_euler('xyz')  
        # 转为角度制（因为 xyzrpyToPose 输入的 R/P/Y 是角度制）
        rpy_deg = np.rad2deg(rpy_rad)  
        # 构造机械臂目标位姿：[x, y, z（米）, Roll, Pitch, Yaw（角度）]
        pose = [point[0], point[1], point[2], rpy_deg[0], rpy_deg[1], rpy_deg[2]]
        print(f"第{nums}个目标点：{point}, rpy_deg:{rpy_deg}")
        # return
        # 2. 安全校验：Z轴坐标范围（避免机械臂碰撞，假设安全范围：-0.2m ~ 1m）
        z = point[2]
        assert -.2 <= z <= 1, f"Z轴坐标超出安全范围：{z}m（安全范围：-0.2~1m）"

        # 3. 构造机械臂目标位姿（[x, y, z, rx, ry, rz]，单位：米/弧度）
        pose = [point[0], point[1], point[2], rpy_deg[0], rpy_deg[1], rpy_deg[2]]
        if self.publish_tf is not None:
            self.publish_tf(target_bTg, 'b', 'g')
            # self.publish_tf(self.bTt, 'b', 't')
            # self.publish_tf(self.camera.gTc, 'g', 'c')
            print('-----')
            # if input("Y/N:")!='Y':
            #     return False
        # 4. 控制机械臂移动到目标位姿（速度0.2m/s，加速度0.1m/s²，调用Robot类的moveP函数）
        if not self.robot.moveP(self.config['moveP_vel'], self.config['moveP_acc'], pose, self.arm_short, self.offset):
            return False

        # 5. 等待机械臂稳定（0.1秒延迟，避免运动未结束就拍照）
        time.sleep(.1)
        # 6. 拍照并记录位姿
        self.savePhotoAndPose(nums)

    def moveAndCaptureAll(self, bTt):
        """
        批量执行拍摄任务：根据任务类型（oneloc/calib/loc）控制机械臂移动并拍照
        是整个类的入口函数，串联所有流程
        """
        print('~~~~~~~~开始拍摄~~~~~~~~')
        bTt[:3,3]*=1000
        self.bTt=bTt
        # print(f'bTt={self.bTt[:3,3]}')
        self.publish_tf(self.bTt, 'b', 't')
        config = self.config
        task = config['task']  # 获取任务类型（oneloc：单次拍摄；calib：标定；loc：定位）

        if task == 'oneloc':  # 单次拍摄任务（如初始定位、单帧校准）
            print(f'单次拍摄 task={task}')
            self.savePhotoAndPose(0)  # 直接拍照（序号0），无需机械臂移动
        else:  # 多次拍摄任务（calib或loc）
            print(f'多次拍摄 task={task}')
            allXYZ = self.generate_all_xyz(config)  # 生成所有目标点
            # 遍历所有目标点，依次移动并拍照
            for nums in range(len(allXYZ)):
                targetXYZ_tTc = allXYZ[nums]  # 当前目标点（目标坐标系下）
                self.moveAndTakePhoto(nums, targetXYZ_tTc)

        # 任务结束后：统计机械臂运动精度（位姿变化量的均值和标准差）
        mean = np.mean(self.allDeltaPose, axis=0)  # 变化量均值（越小越稳定）
        std = np.std(self.allDeltaPose, axis=0)    # 变化量标准差（越小越精确）
        # ic(mean)  # 调试打印：运动精度均值
        # ic(std)   # 调试打印：运动精度标准差

        # 将运动精度统计结果写入日志文件（capture-log.txt）
        with open(f'{self.file_path}/capture-log.txt', 'w') as f:
            f.write(f'mean delta: {mean}\n')  # 均值：各轴平均偏差（毫米）
            f.write(f'std delta: {std}\n')    # 标准差：各轴偏差波动（毫米）

        # 释放资源：关闭位姿文件、断开机械臂连接
        self.fpose.close()
        self.robot.close()
        print('~~~~~~~~结束拍摄~~~~~~~~')


# ------------------------------ 主函数：任务入口 ------------------------------
if __name__ == '__main__':
    # 初始化ROS节点（用于与机械臂/其他模块通信，匿名=True避免节点名冲突）
    rospy.init_node('capture_node', anonymous=True)
    
    # 加载基础坐标系到目标坐标系的变换矩阵（从txt文件读取，逗号分隔）
    # 注：该矩阵通常来自前期标定（如oneloc任务的输出），是坐标变换的核心依据
    bTt = np.loadtxt('matrix/oneloc_bTt.txt', delimiter=',')
    
    # 校验命令行参数：需传入1个配置文件路径（如python capture.py capture-config/oneloc.json）
    assert len(sys.argv) == 2, "参数错误！用法：python capture.py <配置文件路径>"
    configFile = sys.argv[1]  # 获取命令行传入的配置文件路径

    # 初始化MoveAndCapture对象并执行批量拍摄任务
    moveAndCapture = MoveAndCapture(configFile, bTt)
    moveAndCapture.moveAndCaptureAll()