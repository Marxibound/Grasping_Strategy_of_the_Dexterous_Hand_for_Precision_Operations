import pybullet as p
# import torch
import pybullet_data
import math
import numpy as np
import os
import numpy as np

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
    #     print(f"关节{i}：名称={joint_name}，类型={joint_type}，限位=[{lower_limit:.2f}, {upper_limit:.2f}]")
    # print("="*60)
        print(upper_limit,end=',')
    return hand_id
if __name__=="__main__":
    physics_client = init_simulation()
    load_dexterous_hand(physics_client)