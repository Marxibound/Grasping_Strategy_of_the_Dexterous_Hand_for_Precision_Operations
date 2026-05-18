import pybullet as p
# import torch
import pybullet_data
import math
import numpy as np
import os
from glob import glob
import time
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import numpy as np
from pynput.mouse import Listener, Button
import time
def quat2euler(qx, qy, qz, qw):
            roll = np.arctan2(2*(qw*qx + qy*qz), 1 - 2*(qx**2 + qy**2))
            pitch = np.arcsin(2*(qw*qy - qz*qx))
            yaw = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))
            return roll, pitch, yaw

# 手动实现：欧拉角转四元数
def euler2quat(roll, pitch, yaw):
    cr = np.cos(roll/2)
    sr = np.sin(roll/2)
    cp = np.cos(pitch/2)
    sp = np.sin(pitch/2)
    cy = np.cos(yaw/2)
    sy = np.sin(yaw/2)
    qw = cr*cp*cy + sr*sp*sy
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    return [qx, qy, qz, qw]
class positionSubscriber(Node):
    def __init__(self):
        super().__init__("position_subscriber")
        self.subscriber_=self.create_subscription(
            PoseStamped,  ##根据相机发布的数据类型来
            "/vrpn/RigidBody_1/pose",
            self.my_call_back,
            10
        )
    def my_call_back(self,msg):

        # 坐标赋值（保留不变）
        self.x = msg.pose.position.x
        self.y = msg.pose.position.z
        self.z = -msg.pose.position.y

        self.rx, self.ry, self.rz, self.rw=msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
        # # 提取原始四元数
        # qx, qy, qz, qw = msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w

        # # 手动实现：四元数转欧拉角（roll, pitch, yaw）→ 弧度，轴序rzyx
        

        # # 执行转换和轴互换
        # rx, ry, rz = quat2euler(qx, qy, qz, qw)
        # #rx_new, ry_new, rz_new = ry, rx, -rz  # 互换欧拉角y/z
        # rx_new, ry_new, rz_new = rx, rz, ry
        # q_new = euler2quat(rx_new, ry_new, rz_new)

        # # 赋值新四元数
        # self.rx, self.ry, self.rz, self.rw = q_new

    def get_delta(self):
        # 位置变化量：当前位置 - 初始位置
        self.delta_x = self.x - self.init_x
        self.delta_y = self.y - self.init_y
        self.delta_z = self.z - self.init_z

        # 四元数共轭（初始姿态init_rx/ry/rz/rw的共轭）
        conj_rx = -self.init_rx
        conj_ry = -self.init_ry
        conj_rz = -self.init_rz
        conj_rw = self.init_rw

        # 四元数乘法：当前姿态(rx,ry,rz,rw) ⊗ 初始共轭(conj_rx,conj_ry,conj_rz,conj_rw) → 姿态变化量delta_r
        x1, y1, z1, w1 = self.rx, self.ry, self.rz, self.rw
        x2, y2, z2, w2 = conj_rx, conj_ry, conj_rz, conj_rw
        delta_rx = x1*w2 + y1*z2 - z1*y2 + w1*x2
        delta_ry = -x1*z2 + y1*w2 + z1*x2 + w1*y2
        delta_rz = x1*y2 - y1*x2 + z1*w2 + w1*z2
        delta_rw = -x1*x2 - y1*y2 - z1*z2 + w1*w2

        # 姿态变化量归一化（必须做，消除误差）
        norm = math.sqrt(delta_rx**2 + delta_ry**2 + delta_rz**2 + delta_rw**2)
        if norm < 1e-6:
            delta_rx, delta_ry, delta_rz, delta_rw = 0.0, 0.0, 0.0, 1.0
        else:
            delta_rx /= norm
            delta_ry /= norm
            delta_rz /= norm
            delta_rw /= norm
        self.delta_rx=delta_rx
        self.delta_ry=delta_ry
        self.delta_rz=delta_rz
        self.delta_rw=delta_rw
    def get_new_sim(self):
        # 计算新仿真位置：初始仿真位置 + 真实位置变化量
        self.new_sim_pos = [
            self.sim_init_pos[0] + self.delta_x,
            self.sim_init_pos[1] + self.delta_y,
            self.sim_init_pos[2] + self.delta_z
        ]

        # 四元数乘法（PyBullet/ROS格式x,y,z,w：仿真初始姿态 ⊗ 姿态变化量）
        x1,y1,z1,w1 = self.sim_init_ori
        x2,y2,z2,w2 = self.delta_rx, self.delta_ry, self.delta_rz, self.delta_rw
        new_sim_ori = (
            x1*w2 + y1*z2 - z1*y2 + w1*x2,
            -x1*z2 + y1*w2 + z1*x2 + w1*y2,
            x1*y2 - y1*x2 + z1*w2 + w1*z2,
            -x1*x2 - y1*y2 - z1*z2 + w1*w2
        )

        # 归一化新仿真姿态四元数（消除计算误差，必做）
        n = math.sqrt(new_sim_ori[0]**2 + new_sim_ori[1]**2 + new_sim_ori[2]**2 + new_sim_ori[3]**2)
        if n < 1e-6:new_sim_ori=(0.0,0.0,0.0,1.0)
        else:new_sim_ori=(new_sim_ori[0]/n,new_sim_ori[1]/n,new_sim_ori[2]/n,new_sim_ori[3]/n)

        self.new_sim_ori=new_sim_ori
    # 接受到相机数据即此时位姿后要进行处理，然后用self.position和self.rotation进行存储
    # 可能这里的self.position和self.rotation代表的是delta,用此时的数据减去
    # 现实手套的初始值,所以最后的结果还需要加上仿真环境的初始值，才是目标值

    
class JointStateSubscriber(Node):
    """
    一个简单的 ROS 2 节点，用于订阅并打印 JointState 话题中的数据。
    """
    def __init__(self):
        # 初始化节点，节点名称为 "joint_state_subscriber_py"
        super().__init__('joint_state_subscriber_py')
        self.angles=[0]*12
        # 创建一个订阅者
        # 1. 订阅的话题名称: "my_msg" (请根据你的实际话题名修改)
        # 2. 消息类型: JointState
        # 3. 回调函数: self.listener_callback
        # 4. 队列大小: 10
        self.subscription = self.create_subscription(
            JointState,
            'my_msg',
            self.listener_callback,
            10)
        
        # 防止订阅者被Python垃圾回收器意外销毁
        self.subscription  

        self.get_logger().info('JointState Subscriber (Python) has been started.')
        self.get_logger().info('Waiting for data on topic: \'joint_states\'...')

    def listener_callback(self, msg):
        """
        当收到 JointState 消息时的回调函数。
        """
        # self.get_logger().info('Received JointState message.')

        # 检查 name 和 position 列表的长度是否一致
        if len(msg.name) != len(msg.position):
            self.get_logger().warn('Warning: \'name\' and \'position\' lists have different sizes!')
            return
        # 使用 zip 函数同时遍历名称和位置列表
        for name, position in zip(msg.name, msg.position):
            if name=='left_thumb_root':
                #spread=(position-17)*3
                #print(f"spread为{spread}")
                self.angles[9]=1.5*to_rad(-position+20)
                mid_angle=90+to_deg(0.5-self.angles[9])
                mid_angle2=from_active_to_passive_thumb1(mid_angle)
                self.angles[10]=-0.3-to_rad(180-mid_angle2)
                self.angles[11]=-0.8-to_rad(from_active_to_passive_thumb2(180-mid_angle2))
                
            if name=='left_thumb_ip':
                #stretch=position*2
                self.angles[8]=to_rad(1.5*(-position-10))
                #self.angles[8]=to_rad(0)
            
            if name=='left_index':
                
                self.angles[0]=to_rad(position)-math.pi
                self.angles[1]=to_rad(from_active_to_passive(position))-math.pi
            if name=='left_middle':
                self.angles[2]=to_rad(position)-math.pi
                self.angles[3]=to_rad(from_active_to_passive(position))-math.pi
            if name=='left_ring':
                self.angles[4]=to_rad(position)-math.pi
                self.angles[5]=to_rad(from_active_to_passive(position))-math.pi
            if name=='left_pinky':
                self.angles[6]=to_rad(position)-math.pi
                self.angles[7]=to_rad(from_active_to_passive(position))-math.pi
        # link5_angle,link51_angle=map_to_thumb(spread,stretch)
        # self.angles[8]=to_rad(link5_angle)-0.78
        # self.angles[9]=0.45-to_rad(link51_angle)
        # link52_angle=from_active_to_passive_thumb1(90+link51_angle)
        # self.angles[10]=to_rad(180-link52_angle)
        # #self.angles[10]=to_rad(45)
        # link53_angle=from_active_to_passive_thumb2(link52_angle)
        # self.angles[11]=to_rad(180-link53_angle)
        # #self.angles[11]=to_rad(45)


        # self.angles[8]=0
        # self.angles[9]=to_rad(45)
        # link52_angle=from_active_to_passive_thumb1(90+link51_angle)
        # self.angles[10]=0
        # link53_angle=from_active_to_passive_thumb2(link52_angle)
        # self.angles[11]=0
        
def from_active_to_passive_thumb1(active):
    return 1.1424*active+28.0925
def from_active_to_passive_thumb2(active):
    return 0.7508*active-31.6208
def from_active_to_passive(active):
    return 1.1169*active-10.6867


def map_to_thumb(spread,stretch):
    spread=to_rad(spread)
    stretch=to_rad(stretch)
    link51_angle_=math.acos(math.cos(spread)*math.cos(stretch))
    link51_angle_=to_deg(link51_angle_)
    #print(f"弯曲为{link51_angle_}")

    link5_angle_=math.acos(abs(
        math.sin(stretch)/math.sqrt(
            math.sin(stretch)**2+(math.cos(stretch)*math.sin(spread))**2
        )
    ))
    link5_angle_=to_deg(link5_angle_)
    #print(f"旋转为{link5_angle_}")
    return link5_angle_,link51_angle_

def to_rad(angle):
    return angle*math.pi/180
def to_deg(angle):
    return angle/math.pi*180

# ===================== 全局配置参数（务必按你的实际情况修改） =====================
# 灵巧手配置
HAND_URDF_PATH = "inspire_hand_left.urdf"  # 你的灵巧手URDF路径
HAND_INIT_POS = [0, -0.15, 0]  # 灵巧手初始位置
HAND_INIT_ORN = p.getQuaternionFromEuler([-math.pi/2, 0, 0])  # 灵巧手初始姿态
HAND_JOINT_INDICES = []  # 你的灵巧手可动关节索引（需与数据手套映射）

# 物体配置
OBJECT_MESH_DIR = "mesh_files"  # 物体Mesh文件夹（存放所有待抓取物体的STL/OBJ/PLY）
OBJECT_SCALE = 1.0  # 物体统一缩放系数（可按需单独调整）
OBJECT_INIT_POS = [0, 0, 0]  # 物体初始位置（在灵巧手下方）
OBJECT_INIT_ORN = p.getQuaternionFromEuler([0, 0, 0])  # 物体初始姿态

# 数据集配置
OUTPUT_PT_PATH = "output/Manual_Grasp_Dataset_My_Hand.pt"  # 最终数据集保存路径
GRASP_METADATA_KEYS = ["rotations", "joint_positions", "translations", "object_name", "scale"]  # 数据集必需字段

# ===================== 核心功能函数 =====================
#### 可修改：204行重力，224行灵巧手固定，297行物体固定
def init_simulation():
    """初始化PyBullet仿真环境（可视化模式，支持手动交互）"""
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1/240)
    p.resetDebugVisualizerCamera(
        cameraDistance=1.5,
        cameraYaw=45,
        cameraPitch=-30,
        cameraTargetPosition=[0, 0, 0]
    )
    # 关闭自动仿真步长，改为手动控制（便于精准摆姿）
    
    p.setRealTimeSimulation(1)  # 实时仿真，同步数据手套动作
    print("仿真环境初始化完成，已开启实时仿真模式")
    return physics_client

def load_dexterous_hand(physics_client):
    """加载灵巧手并打印关节信息，返回手ID"""
    # 加载灵巧手（固定底座，仅关节运动）
    hand_id = p.loadURDF(
        fileName=HAND_URDF_PATH,
        basePosition=HAND_INIT_POS,
        baseOrientation=HAND_INIT_ORN,
        useFixedBase=True
    )

    # 打印关节信息，核对可动关节索引
    joint_num = p.getNumJoints(hand_id)
    print("="*60)
    print(f"灵巧手加载成功，共{joint_num}个关节：")
    global HAND_JOINT_INDICES
    HAND_JOINT_INDICES = [i for i in range(p.getNumJoints(hand_id)) if p.getJointInfo(hand_id, i)[2] != p.JOINT_FIXED]
    #print(HAND_JOINT_INDICES)
    for i in range(joint_num):
        info = p.getJointInfo(hand_id, i)
        joint_name = info[1].decode("utf-8")
        joint_type = info[2]
        lower_limit = info[8]
        upper_limit = info[9]
        print(f"关节{i}：名称={joint_name}，类型={joint_type}，限位=[{lower_limit:.2f}, {upper_limit:.2f}]")
    print("="*60)
    return hand_id

# def load_object_list():
#     """获取物体Mesh文件列表，返回物体路径与名称映射"""
#     # 支持的Mesh格式
#     supported_formats = [".stl", ".obj", ".ply"]
#     object_files = []
#     for fmt in supported_formats:
#         object_files.extend(glob(os.path.join(OBJECT_MESH_DIR, f"*{fmt}")))
    
#     if not object_files:
#         raise FileNotFoundError(f"在{OBJECT_MESH_DIR}目录下未找到STL/OBJ/PLY格式的物体Mesh文件")
    
#     # 构建物体路径-名称映射
#     object_dict = {}
#     for file_path in object_files:
#         object_name = os.path.basename(file_path).split(".")[0]
#         object_dict[object_name] = file_path
    
#     print(f"共找到{len(object_dict)}个待抓取物体：{list(object_dict.keys())}")
#     return object_dict
def load_object_list():
    """获取物体Mesh文件列表，严格按文件名开头0xx数字编号升序排列，返回路径与名称映射"""
    # 支持的Mesh格式
    supported_formats = [".stl", ".obj", ".ply"]
    supported_suffix = {fmt.lower() for fmt in supported_formats}
    object_files = []

    # 1. 获取目录下所有文件，过滤出支持的格式（先筛选，再排序）
    all_files = os.listdir(OBJECT_MESH_DIR)
    for file_name in all_files:
        file_path = os.path.join(OBJECT_MESH_DIR, file_name)
        # 只处理文件，过滤子文件夹
        if not os.path.isfile(file_path):
            continue
        # 过滤支持的文件格式（忽略后缀大小写）
        file_suffix = os.path.splitext(file_name)[1].lower()
        if file_suffix in supported_suffix:
            object_files.append(file_path)

    # 无文件则抛异常，保留原逻辑
    if not object_files:
        raise FileNotFoundError(f"在{OBJECT_MESH_DIR}目录下未找到STL/OBJ/PLY格式的物体Mesh文件")

    # 2. 核心：按文件名开头的0xx数字编号纯数值升序排序
    def get_sort_key(file_path):
        """提取文件名开头的数字作为排序键，转为整数"""
        file_name = os.path.basename(file_path)
        # 分割文件名，提取开头的数字部分（005/012/021...），非数字部分直接忽略
        num_part = ''.join([c for c in file_name if c.isdigit()])
        # 转为整数，实现纯数值排序（005→5，012→12，021→21）
        return int(num_part) if num_part else 9999  # 无数字的放最后

    # 按提取的数字键升序排序，这一步保证002→003→012→021的顺序
    object_files_sorted = sorted(object_files, key=get_sort_key)

    # 3. 基于排序后的列表构建字典，顺序严格按0xx数字排列
    object_dict = {}
    for file_path in object_files_sorted:
        object_name = os.path.basename(file_path).split(".")[0]
        object_dict[object_name] = file_path
    # 打印排序后的结果，方便验证
    print(f"共找到{len(object_dict)}个待抓取物体（按0xx数字排序）：{list(object_dict.keys())}")
    return object_dict
def load_single_object(physics_client, object_path):
    """加载单个物体，返回物体ID和名称"""
    object_name = os.path.basename(object_path).split(".")[0]
    # 临时URDF封装（兼容所有Mesh格式，带碰撞检测）
    temp_urdf_path = f"temp_object_{object_name}.urdf"
    mesh_ext = os.path.splitext(object_path)[-1].lower()
    with open(temp_urdf_path, "w") as f:
        f.write(f"""
<robot name="{object_name}">
  <link name="base">
    <visual>
      <geometry>
        <mesh filename="{os.path.abspath(object_path)}" scale="{OBJECT_SCALE} {OBJECT_SCALE} {OBJECT_SCALE}"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <mesh filename="{os.path.abspath(object_path)}" scale="{OBJECT_SCALE} {OBJECT_SCALE} {OBJECT_SCALE}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>
</robot>
        """)
    
    # 加载物体
    object_id = p.loadURDF(
        fileName=temp_urdf_path,
        basePosition=OBJECT_INIT_POS,
        baseOrientation=OBJECT_INIT_ORN,
        useFixedBase=True
    )
    # 删除临时URDF文件
    os.remove(temp_urdf_path)
    print(f"\n物体「{object_name}」加载成功，ID={object_id-1}")
    return object_id, object_name

# def map_data_glove_to_hand(hand_id):
#     """数据手套动作映射到仿真灵巧手（需你补充硬件通信逻辑）
#     说明：此处为框架接口，你需要根据数据手套的SDK替换为实际读取和映射代码
#     """
#     # 示例：读取数据手套关节角度（需替换为你的硬件读取逻辑）
#     # glove_joint_angles = your_glove_sdk.read_joint_angles()  # 真实数据手套读取
#     glove_joint_angles = np.random.uniform(-0.5, 0.5, len(HAND_JOINT_INDICES))  # 临时模拟，需删除
    
#     # 将数据手套角度映射到灵巧手关节（按关节索引对应）
#     for idx, angle in zip(HAND_JOINT_INDICES, glove_joint_angles):
#         # 限制关节角度在限位范围内
#         joint_info = p.getJointInfo(hand_id, idx)
#         lower_limit = joint_info[8]
#         upper_limit = joint_info[9]
#         angle_clamped = np.clip(angle, lower_limit, upper_limit)
        
#         # 控制灵巧手关节运动
#         p.setJointMotorControl2(
#             bodyUniqueId=hand_id,
#             jointIndex=idx,
#             controlMode=p.POSITION_CONTROL,
#             targetPosition=angle_clamped,
#             force=500,  # 关节驱动力，确保运动顺滑
#             velocity=1.0
#         )

# def record_grasp_pose(hand_id, object_name):
#     """记录当前灵巧手的抓取姿态，返回符合数据集格式的字典"""
#     # 1. 获取灵巧手全局平移向量
#     hand_trans, hand_orn = p.getBasePositionAndOrientation(hand_id)
#     translations = torch.tensor(hand_trans, dtype=torch.float32)
    
#     # 2. 获取灵巧手旋转矩阵（四元数转3x3旋转矩阵）
#     hand_rot_mat = np.array(p.getMatrixFromQuaternion(hand_orn)).reshape(3, 3)
#     rotations = torch.tensor(hand_rot_mat, dtype=torch.float32)
    
#     # 3. 获取灵巧手可动关节角度
#     joint_positions = []
#     for idx in HAND_JOINT_INDICES:
#         joint_state = p.getJointState(hand_id, idx)
#         joint_angle = joint_state[0]  # 关节当前角度
#         joint_positions.append(joint_angle)
#     joint_positions = torch.tensor(joint_positions, dtype=torch.float32)
    
#     # 4. 整理为数据集字典格式
#     grasp_data = {
#         "rotations": rotations,          # (3,3) 旋转矩阵
#         "joint_positions": joint_positions,  # (N,) 关节角度（N为可动关节数）
#         "translations": translations,    # (3,) 全局平移
#         "object_name": object_name,      # 物体名称
#         "scale": OBJECT_SCALE,           # 物体缩放系数
#         "record_time": time.strftime("%Y-%m-%d %H:%M:%S")  # 记录时间（可选）
#     }
#     print(f"已记录物体「{object_name}」的抓取姿态，关节维度：{joint_positions.shape[0]}")
#     return grasp_data
def record_grasp_pose2(hand_id, object_name):
    """记录当前灵巧手的抓取姿态，返回符合数据集格式的字典"""
    # 1. 获取灵巧手全局平移向量
    hand_trans, hand_orn = p.getBasePositionAndOrientation(hand_id)
    #translations = torch.tensor(hand_trans, dtype=torch.float32)
    
    # 2. 获取灵巧手旋转矩阵（四元数转3x3旋转矩阵）
    hand_rot_mat = np.array(p.getMatrixFromQuaternion(hand_orn)).reshape(3, 3)
    hand_rot_mat = hand_rot_mat.tolist()
    #rotations = torch.tensor(hand_rot_mat, dtype=torch.float32)
    
    # 3. 获取灵巧手可动关节角度
    joint_positions = []
    for idx in HAND_JOINT_INDICES:
        joint_state = p.getJointState(hand_id, idx)
        joint_angle = joint_state[0]  # 关节当前角度
        joint_positions.append(joint_angle)
    #joint_positions = torch.tensor(joint_positions, dtype=torch.float32)
    
    # 4. 整理为数据集字典格式
    grasp_data = {
        "rotations": hand_rot_mat,          # (3,3) 旋转矩阵
        "joint_positions": joint_positions,  # (N,) 关节角度（N为可动关节数）
        "translations": hand_trans,    # (3,) 全局平移
        "object_name": object_name,      # 物体名称
        "scale": OBJECT_SCALE,           # 物体缩放系数
        "record_time": time.strftime("%Y-%m-%d %H:%M:%S")  # 记录时间（可选）
    }
    print(f"已记录物体「{object_name}」的抓取姿态，关节维度：{len(joint_positions)}")
    return grasp_data

def save_grasp_dataset2(grasp_metadata_list):
    """将所有记录的抓取数据保存为.pt文件（符合数据集格式要求）"""
    # 统计每个物体的记录数量
    num_per_object = {}
    for grasp_data in grasp_metadata_list:
        obj_name = grasp_data["object_name"]
        num_per_object[obj_name] = num_per_object.get(obj_name, 0) + 1
    
    # 构建完整数据集字典
    grasp_dataset_dict = {
        "info": {
            "dataset_source": "Manual Control (Data Glove) + PyBullet Simulation",
            "hand_type": os.path.basename(HAND_URDF_PATH).split(".")[0],
            "record_time_range": [grasp_metadata_list[0]["record_time"], grasp_metadata_list[-1]["record_time"]],
            "num_per_object": num_per_object,
            "total_valid_grasps": len(grasp_metadata_list),
            "object_dir": OBJECT_MESH_DIR
        },
        "metadata": grasp_metadata_list
    }
    
    # 保存为.pt文件
    #torch.save(grasp_dataset_dict, OUTPUT_PT_PATH)
    i2=0
    for i in range(1000):
        if os.path.exists(f"output/result{i}.json"):
            i+=1
        else:
            i2=i
            break

    with open(f"output/result{i2}.json", "w", encoding="utf-8") as f:
        json.dump(grasp_dataset_dict, f, indent=4, ensure_ascii=False)
    print("="*60)
    print(f"数据集保存成功！路径：{OUTPUT_PT_PATH}")
    print(f"数据集统计：")
    print(f"  总记录抓取姿态数：{len(grasp_metadata_list)}")
    print(f"  涉及物体数：{len(num_per_object)}")
    for obj, count in num_per_object.items():
        print(f"    - {obj}：{count}个姿态")
    print("="*60)
    # def save_grasp_dataset(grasp_metadata_list):
#     """将所有记录的抓取数据保存为.pt文件（符合数据集格式要求）"""
#     # 统计每个物体的记录数量
#     num_per_object = {}
#     for grasp_data in grasp_metadata_list:
#         obj_name = grasp_data["object_name"]
#         num_per_object[obj_name] = num_per_object.get(obj_name, 0) + 1
    
#     # 构建完整数据集字典
#     grasp_dataset_dict = {
#         "info": {
#             "dataset_source": "Manual Control (Data Glove) + PyBullet Simulation",
#             "hand_type": os.path.basename(HAND_URDF_PATH).split(".")[0],
#             "record_time_range": [grasp_metadata_list[0]["record_time"], grasp_metadata_list[-1]["record_time"]],
#             "num_per_object": num_per_object,
#             "total_valid_grasps": len(grasp_metadata_list),
#             "object_dir": OBJECT_MESH_DIR
#         },
#         "metadata": grasp_metadata_list
#     }
    
#     # 保存为.pt文件
#     torch.save(grasp_dataset_dict, OUTPUT_PT_PATH)
#     print("="*60)
#     print(f"数据集保存成功！路径：{OUTPUT_PT_PATH}")
#     print(f"数据集统计：")
#     print(f"  总记录抓取姿态数：{len(grasp_metadata_list)}")
#     print(f"  涉及物体数：{len(num_per_object)}")
#     for obj, count in num_per_object.items():
#         print(f"    - {obj}：{count}个姿态")
#     print("="*60)








# ===================== 主流程（手动交互采集） =====================
# if __name__ == "__main__":
#     # 初始化流程
#     grasp_metadata_list = []  # 存储所有记录的抓取姿态
#     try:
#         # 1. 初始化仿真环境
#         physics_client = init_simulation()
        
#         # 2. 加载灵巧手
#         hand_id = load_dexterous_hand(physics_client)
        
#         # 3. 获取物体列表
#         object_dict = load_object_list()
#         object_names = list(object_dict.keys())
        
#         # 4. 逐个物体进行手动抓取采集
#         for idx, obj_name in enumerate(object_names):
#             print("\n" + "="*60)
#             print(f"正在处理第{idx+1}/{len(object_names)}个物体：{obj_name}")
#             print("操作说明：")
#             print("  1. 通过数据手套操控灵巧手摆好抓取姿态")
#             print("  2. 摆好后按「Enter」键记录当前姿态")
#             print("  3. 按「ESC」键跳过当前物体/结束采集流程")
#             print("="*60)
            
#             # 加载当前物体
#             object_path = object_dict[obj_name]
#             object_id, current_obj_name = load_single_object(physics_client, object_path)
            
#             # 等待用户交互（摆姿+按键记录）
#             record_flag = False
#             while True:
#                 # 实时映射数据手套动作到灵巧手

#                 map_data_glove_to_hand(hand_id)
                
#                 # 获取键盘事件
#                 keys = p.getKeyboardEvents()
                
#                 # 按Enter键记录姿态
#                 if p.B3G_RETURN in keys and keys[p.B3G_RETURN] & p.KEY_WAS_TRIGGERED:
#                     grasp_data = record_grasp_pose(hand_id, current_obj_name)
#                     grasp_metadata_list.append(grasp_data)
#                     record_flag = True
#                     break  # 记录完成，进入下一个物体
                
#                 # 按ESC键跳过当前物体
#                 if p.B3G_ESCAPE in keys and keys[p.B3G_ESCAPE] & p.KEY_WAS_TRIGGERED:
#                     print(f"已跳过物体「{current_obj_name}」")
#                     break
                
#                 time.sleep(0.01)  # 轻微延时，降低CPU占用
            
#             # 移除当前物体，准备下一个
#             p.removeBody(object_id)
#             if record_flag:
#                 print(f"物体「{current_obj_name}」采集完成，已记录1个抓取姿态")
#             time.sleep(0.5)  # 切换物体间隔，便于观察
        
#         # 5. 按ESC键结束采集，保存数据集
#         print("\n所有物体处理完毕，等待确认结束...")
#         print("按「ESC」键保存数据集并退出仿真")
#         while True:
#             keys = p.getKeyboardEvents()
#             if p.B3G_ESCAPE in keys and keys[p.B3G_ESCAPE] & p.KEY_WAS_TRIGGERED:
#                 if grasp_metadata_list:
#                     save_grasp_dataset(grasp_metadata_list)
#                 else:
#                     print("警告：未记录任何抓取姿态，无需保存数据集")
#                 break
#             time.sleep(0.01)
    
#     except Exception as e:
#         print(f"程序异常：{e}")
#     finally:
#         # 关闭仿真环境
#         p.disconnect()
#         print("仿真环境已关闭，程序结束")
def map_data_glove_to_hand(hand_id,joint_state_subscriber: JointStateSubscriber,position_subscriber:positionSubscriber):
    """数据手套动作映射到仿真灵巧手（需你补充硬件通信逻辑）
    说明：此处为框架接口，你需要根据数据手套的SDK替换为实际读取和映射代码
    """
    # 示例：读取数据手套关节角度（需替换为你的硬件读取逻辑）
    rclpy.spin_once(joint_state_subscriber)
    glove_joint_angles = joint_state_subscriber.angles;  # 真实数据手套读取
    #glove_joint_angles = np.random.uniform(-0.5, 0.5, len(HAND_JOINT_INDICES))  # 临时模拟，需删除

    rclpy.spin_once(position_subscriber,timeout_sec=0)
    position_subscriber.get_delta()
    position_subscriber.get_new_sim()

    p.resetBasePositionAndOrientation(hand_id, position_subscriber.new_sim_pos, position_subscriber.new_sim_ori)
    # 或者此处可能表示差值，还需要加上手在仿真环境中的初值
    
    # 此处需要加上根据glove_position和glove_rotation控制灵巧手位姿的代码
    

    # 将数据手套角度映射到灵巧手关节（按关节索引对应）
    for idx, angle in zip(HAND_JOINT_INDICES, glove_joint_angles):
        # 限制关节角度在限位范围内
        joint_info = p.getJointInfo(hand_id, idx)
        lower_limit = joint_info[8]
        upper_limit = joint_info[9]
        angle_clamped = np.clip(angle, lower_limit, upper_limit)
        
        # 控制灵巧手关节运动
        p.setJointMotorControl2(
            bodyUniqueId=hand_id,
            jointIndex=idx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=angle_clamped,
            force=500,  # 关节驱动力，确保运动顺滑
        )
if __name__=="__main__":
    grasp_metadata_list = []  # 存储所有记录的抓取姿态
    middle_click = False  # 中键按下标记，初始为False
    listener = Listener(on_click=lambda x,y,b,p: globals().update(middle_click=True) if b==Button.middle and p else None)
    listener.start()
    try:
        rclpy.init()
        joint_state_subscriber = JointStateSubscriber()
        position_subscriber = positionSubscriber()

        physics_client = init_simulation()

        hand_id = load_dexterous_hand(physics_client)

        object_dict = load_object_list()
        object_names = list(object_dict.keys())

        finish=0
        for idx, obj_name in enumerate(object_names):
            now_index=0
            if(idx<now_index):
                continue
            object_path = object_dict[obj_name]
            object_id, current_obj_name = load_single_object(physics_client, object_path)
            
            record_flag = False


            # 将仿真环境中的手复原向上的代码
            p.resetBasePositionAndOrientation(hand_id, HAND_INIT_POS, HAND_INIT_ORN)

            position_subscriber.sim_init_pos, position_subscriber.sim_init_ori = p.getBasePositionAndOrientation(hand_id)

            
            rclpy.spin_once(position_subscriber,timeout_sec=0)
            # 位置xyz 初始值赋值（记录初始位置基准，用于后续相对位置计算）
            position_subscriber.init_x = position_subscriber.x
            position_subscriber.init_y = position_subscriber.y
            position_subscriber.init_z = position_subscriber.z

            # 四元数rx/ry/rz/rw 初始值赋值（rx=四元数x, ry=四元数y, rz=四元数z, rw=四元数w，记录初始姿态基准）
            position_subscriber.init_rx = position_subscriber.rx
            position_subscriber.init_ry = position_subscriber.ry
            position_subscriber.init_rz = position_subscriber.rz
            position_subscriber.init_rw = position_subscriber.rw
            

            print("\n" + "="*60)
            print(f"正在处理第{idx+1}/{len(object_names)}个物体：{obj_name}")
            print("操作说明：")
            print("  1. 通过数据手套操控灵巧手摆好抓取姿态")
            print("  2. 摆好后按「Enter」键记录当前姿态")
            print("  3. 按「ESC」键跳过当前物体/结束采集流程")
            print("="*60)

            strategy_num=0
            #print(f"是这个:{HAND_JOINT_INDICES}")
            while True:
                map_data_glove_to_hand(hand_id,joint_state_subscriber,position_subscriber)
                keys = p.getKeyboardEvents()
                mouse_events=p.getMouseEvents()

                # current_pos, current_ori = p.getBasePositionAndOrientation(hand_id)
                # current_pos = list(current_pos) # 转换为列表以便修改
                # current_euler = p.getEulerFromQuaternion(current_ori) # 将四元数转为欧拉角 (roll, pitch, yaw)
                # current_euler = list(current_euler)
                
                
                # if ord('j') in keys and keys[ord('j')] & p.KEY_IS_DOWN:
                #     current_pos[0] -= TRANSLATION_STEP
                
               
                # if ord('a') in keys and keys[ord('a')] & p.KEY_IS_DOWN:
                #     current_euler[0] += ROTATION_STEP

                # #将更新后的欧拉角转回四元数
                # new_ori = p.getQuaternionFromEuler(current_euler)

                # #应用新的位姿到灵巧手模型
                # p.resetBasePositionAndOrientation(hand_id, current_pos, new_ori)
                if middle_click:
                    # 这里写中键按下要执行的代码
                    grasp_data = record_grasp_pose2(hand_id, current_obj_name)
                    grasp_metadata_list.append(grasp_data)
                    strategy_num+=1
                    record_flag=1
                    print(f"记录成功，第{idx+1}/{len(object_names)}个物体,第{strategy_num}个姿态，继续抓取")  # 按一次中键，只执行一次
                    middle_click = False  # 重置标记，保证单次触发

                if p.B3G_RETURN in keys and keys[p.B3G_RETURN] & p.KEY_WAS_TRIGGERED:
                    print("下一个物体")
                    print("已重新加载灵巧手，请在2秒内将手部姿态复原！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！")
                    time.sleep(5)
                    map_data_glove_to_hand(hand_id,joint_state_subscriber,position_subscriber)
                    break
                    
                if ord("f") in keys and keys[ord("f")] & p.KEY_WAS_TRIGGERED:
                    print(f"抓取结束")
                    finish=1
                    break
                p.stepSimulation()
                time.sleep(0.01)  # 轻微延时，降低CPU占用
            if finish==1:
                break
            p.removeBody(object_id)
            if record_flag:
                print(f"物体「{current_obj_name}」采集完成")
            time.sleep(0.5)  # 切换物体间隔，便于观察
        print("\n所有物体处理完毕")
        print("开始记录数据")
        save_grasp_dataset2(grasp_metadata_list)
        print("记录数据完成")
                
    except Exception as e:
        print(f"程序异常：{e}")
    finally:
        # 关闭仿真环境
        #p.disconnect()
        print("仿真环境已关闭，程序结束")