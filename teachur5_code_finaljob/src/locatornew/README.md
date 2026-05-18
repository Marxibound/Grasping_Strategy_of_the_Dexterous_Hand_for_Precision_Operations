# 重定位功能包（locatornew）使用说明

## 功能概述

`locatornew` 是一个基于ROS1的重定位功能包，用于实现双机械臂系统提供标定、定位等功能。该包支持多种标定板类型（圆形网格、Aruco标记、Charuco棋盘格），提供话题（Topic）和服务（Service）两种交互接口，可实现实时识别、单次定位、标定和多次定位等操作。

## 目录结构

```
locatornew/
├── config/                  # 配置文件目录
│   └── [站点名称].json      # 目标站点的定位参数配置文件
├── img_take/                # 图像与位姿数据存储目录
│   ├── [图像文件]           # 拍摄的标定板/标记图像
│   └── pose.txt             # 对应的机械臂位姿数据
├── matrix/                  # 矩阵变换矩阵存储目录
│   ├── *_camera_matrix.txt  # 相机内参矩阵（标定结果）
│   ├── *_dist_coeffs.txt    # 相机畸变系数（标定结果）
│   ├── *_gTc.txt            # 手眼标定变换矩阵（工具到相机）
│   └── oneloc_bTt.txt       # 单次定位变换矩阵（世界到目标）
├── scripts/                 # 核心代码目录
│   ├── location_service.py  # 定位服务主节点（提供Topic和Service接口）
│   ├── analyze.py           # 标定板识别模块
│   ├── capture.py           # 图像捕获与处理模块
│   └── robot_controller.py  # 机械臂控制接口（支持moveP和getPoseBase方法）
├── CMakeLists.txt           # 编译配置文件
├── package.xml              # ROS包信息文件
└── setup.py                 # Python模块配置文件
```

## 配置文件说明

详见`config/example.json`中的解释

## 核心功能模块

### 1. 定位服务节点（location_service.py）

该节点是功能包的核心，提供两种交互接口：

#### 话题接口（Topic）

- **订阅话题**：`/locator_topic`（消息类型：`std_msgs/String`）
  - 接收JSON格式的命令字符串，格式与配置文件中的顶级参数一致
  - 示例：`{"station": "uv", "arm": "left", "task": "oneloc"}`

- **发布话题**：
  - `/obj_to_robot_holdon`（`PoseStamped`）：目标到机器人的位姿变换
  - `/class_order_holdon`（`String`）：目标名称

#### 服务接口（Service）

- **服务名称**：`/locator_service`
- **服务类型**：自定义`Location`服务，定义如下：
  ```
  string command_json  # 输入：JSON格式的命令字符串
  ---
  string status        # 输出：处理状态信息
  bool result_valid    # 输出：结果是否有效
  float64[] cached_bTt # 输出：4x4变换矩阵（展平为16元素列表）
  string cached_station # 输出：缓存的站点名称
  ```

- **调用示例**：
  - 单次定位:
    ```bash
    rosservice call /locator_service '{"command_json": "{\"station\": \"102table\", \"task\": \"oneloc\"}"}'
    ```
  - 实时定位ie:
    ```bash
    rosservice call /locator_service '{"command_json": "{\"station\": \"102table\", \"task\": \"realtime\"}"}'
    ```
  - 重定位:
    ```bash
    rosservice call /locator_service '{"command_json": "{\"station\": \"102table\", \"task\": \"multiloc\"}"}'
    ```
  - 手眼标定:
    ```bash
    rosservice call /locator_service '{"command_json": "{\"station\": \"102table\", \"task\": \"calib\"}"}'
    ```
## 变换描述
- `bTg`:base to gripper，即`/world`到`/tool0`(机械臂TCP)的变换
- `gTc`:gripper to camera，即`/tool0`到`/camera_color_optical_frame`的变换，手眼矩阵
- `cTt`:camera to target，即`/camera_color_optical_frame`到`target`的变换
- `bTt`:base to target
## 使用步骤

### 手眼标定gTc:gripper to camera

- 预先在`arm_robot_description/urdf/dual_arm_robot.xacro或single_arm_robot.xacro`中设置好相机的近似位置，确定在rviz中能看到相应的`camera_color_optical_frame`的tf，视野向下为Y轴，向右为X轴，向前为Z轴
- 起驱动的基本方式是:

  `roslaunch locatornew location_service.launch`
  - 若要用urdf模型中的`gTc(gripper to camera)`相对位置关系作为手眼标定前的初始位置，则需要在起驱动时增加`camera_gTc_flag:=true`，否则将从`locatornew/scripts/matrix/*_gTc.txt`中加载gTc矩阵，单位为mm
  - 本包内含realsense驱动，若需要使用其他相机，需要在起驱动时增加`camera_image_topic:=***/color/image`相应的图像话题，否则将使用realsense
  - 初次使用一台新的相机，若不知道其内参和畸变矩阵，且相机驱动开启后有类似于`*/color/camera_info`的话题，其中包含相机内参信息的，可以在起驱动时增加`camera_info_topic:=***/color/camera_info`，自动获取相机的内参，畸变矩阵默认为0
    ```shell
    ubuntu@A1-2X4:~$ rostopic echo /stereo_inertial_publisher/color/camera_info 
    header: 
      seq: 2591
      stamp: 
        secs: 1762456759
        nsecs: 112704556
      frame_id: "oak_rgb_camera_optical_frame"
    height: 720
    width: 1280
    distortion_model: "rational_polynomial"
    D: [43.592350006103516, -315.689208984375, -0.00023111335758585483, -0.0008365662069991231, 769.9691162109375, 42.92412567138672, -311.9549560546875, 760.6991577148438]
    K: [1025.91259765625, 0.0, 624.7284545898438, 0.0, 1025.91259765625, 369.03857421875, 0.0, 0.0, 1.0]
    R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    P: [1025.91259765625, 0.0, 624.7284545898438, 0.0, 0.0, 1025.91259765625, 369.03857421875, 0.0, 0.0, 0.0, 1.0, 0.0]
    ```
- 若相机未经手眼标定，仅有一个粗略的gTc，则需要在`locatornew/scripts/config`中创建一个配置文件（以`102table`为例），设置好站点名、手眼标定的机械臂、标定板等参数。
- 在`locatornew/scripts/location_service.py`中增加站点代号与站点配置文件路径的映射关系
  ```python
  deviceNameToConfigFile = {
      '121table': 'config/121光学平台.json',
      '102table': 'config/102桌面.json',
  }
  ```
- 执行实时识别，确定目前的gTc基本正确，标定板tf在rviz中的运动位置符合预期:

  `rosservice call /locator_service '{"command_json": "{\"station\": \"102table\", \"task\": \"realtime\"}"}'`
- 保证机械臂驱动均已开启，若使用其他机械臂，可以修改`locatornew/scripts/robot_controller.py`的接口来做适配
- 执行手眼标定，建议首次手眼标定将机械臂运动速度、加速度调低，xyz的运动范围设小一点，z设高一点，避免机械臂碰撞:

  `rosservice call /locator_service '{"command_json": "{\"station\": \"102table\", \"task\": \"calib\"}"}'`
- 手眼标定的结果gTc和相机内参、畸变将自动存在`locatornew/scripts/matrix`中，并且会将`arm_robot_description/urdf/dual_arm_robot.xacro或single_arm_robot.xacro`在同目录下备份，gTc写入新的文件中，重启机械臂驱动即可更新`camera_color_optical_frame`
- 终端会显示手眼标定的均方根误差：正常x和y轴误差应小于1mm，z轴误差应小于2mm
  ```shell
  ============================================================
  A^-1 B 相对于不变换的误差分析（按类型分类）：
  ------------------------------------------------------------
  【平移误差 RMSE】（单位：mm，越小越好）
    X轴平移误差：0.1715861 mm
    Y轴平移误差：0.236929 mm
    Z轴平移误差：0.901125 mm
    平移误差最大值：0.901125 mm
  ------------------------------------------------------------
  【旋转误差 RMSE】（单位：度，越小越好）
    X轴旋转误差（滚转）：0.010316 ° （原始：0.001800 rad）
    Y轴旋转误差（俯仰）：0.012274 ° （原始：0.002142 rad）
    Z轴旋转误差（偏航）：0.003387 ° （原始：0.000591 rad）
    旋转误差最大值：0.012274 °
  ------------------------------------------------------------
  ```

### 重定位bTt:base to target
-  重定位类似于手眼标定，采用随机拍照的方式求平均bTt:

  `rosservice call /locator_service '{"command_json": "{\"station\": \"102table\", \"task\": \"multiloc\"}"}'`
- 流程大致为先进行一次定位`oneloc`，然后根据定位结果随机产生若干机械臂g位置，并在该位置指向目标拍摄
- 最终结果将发布到`/obj_to_robot_holdon`和`/class_order_holdon`中，分别为bTt的变换和目标名称（配置内的站点名）