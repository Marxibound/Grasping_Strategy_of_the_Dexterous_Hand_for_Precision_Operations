#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import tf.transformations as tf_trans  # 新增：用于四元数和旋转矩阵转换
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from geometry_msgs.msg import TransformStamped, Point
from tf2_ros import TransformBroadcaster
import tf2_ros
import image_geometry
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2
from scipy.optimize import least_squares
import threading

from ultralytics import YOLO

import math
import torch

class BlockDetector:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('block_detection_node', anonymous=True)
        
        # 相机内参相关
        self.camera_model = image_geometry.PinholeCameraModel()
        self.cv_bridge = CvBridge()
        self.camera_info_received = False
        
        # TF相关
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = TransformBroadcaster()
        self.target_frame = "camera_color_optical_frame"  # 父帧：相机光学帧
        
        # 手动选点相关
        self.clicked_corners = []  # 存储点击的四个角点（像素坐标）
        self.image_lock = threading.Lock()  # 图像线程锁
        self.cv_color_img = None  # 缓存当前图像
        self.select_corners_done = False  # 选点完成标志
        
        # 缓存数据
        self.pointcloud = None
        self.block_top_center_pixel = None  # 顶面中心（像素坐标）
        self.block_top_center_cam = None    # 顶面中心（相机坐标）
        self.flatness_error = None          # 平整度误差
        self.plane_normal = None            # 顶面平面法向量（单位向量）
        self.plane_params = None  # 存储平面方程 Ax+By+Cz+D=0 的系数

        # 新增：Reset按钮参数
        self.btn_rect = (10, 10, 100, 40)  # 按钮区域：(x, y, width, height) 左上角坐标+宽高
        self.btn_text = "Reset"
        self.is_drawing = False  # 标记是否正在刷新图像
        
        # 发布识别结果图像
        self.result_img_pub = rospy.Publisher("/block_detection/result_image", Image, queue_size=1)
        # 新增：初始化平面Marker发布器
        self.plane_marker_pub = rospy.Publisher("/block_detection/plane_marker", Marker, queue_size=1)
        
        # 窗口名称
        self.window_name = "Select Block Top 4 Corners (Click in order: TL->TR->BR->BL)"

        # 加载 yolo 模型
        self.model = YOLO("/data/teachur5_code/src/locatornew/model/best(3).pt")

        # 发布可视化图像
        self.image_pub = rospy.Publisher(
            "/yolo/result_image",
            Image,
            queue_size=1
        )

        self.yaw_angle = 0.0
        self.last_exec_time = rospy.Time.now()
        
        rospy.loginfo("方块检测节点已启动，等待相机数据...")
        rospy.loginfo("提示：相机图像显示后，按 左上→ 右上→ 右下→ 左下 顺序点击方块顶面四个角点")

        # 订阅话题并绑定回调函数（关键新增部分）
        rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_cb)
        rospy.Subscriber("/camera/color/image_raw", Image, self.color_image_cb, queue_size=1)
        rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.pointcloud_cb, queue_size=1)
        # # 新增：启动图像刷新线程
        # self.image_thread = threading.Thread(target=self.image_refresh_loop, daemon=True)
        # self.image_thread.start()

    def camera_info_cb(self, msg):
        """获取相机内参，初始化相机模型"""
        if not self.camera_info_received:
            self.camera_model.fromCameraInfo(msg)
            self.camera_info_received = True
            rospy.loginfo("相机内参已获取")
    def image_refresh_loop(self):
        """循环刷新图像窗口，显示按钮和采点状态"""
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1280, 720)
        cv2.setMouseCallback(self.window_name, self.mouse_click_callback)
        
        while not rospy.is_shutdown():
            if self.cv_color_img is None:
                rospy.sleep(0.1)
                continue
            
            with self.image_lock:
                temp_img = self.cv_color_img.copy()
            
            # 绘制Reset按钮
            x, y, w, h = self.btn_rect
            cv2.rectangle(temp_img, (x, y), (x + w, y + h), (255, 0, 0), -1)  # 蓝色按钮
            cv2.putText(temp_img, self.btn_text, (x + 10, y + 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)  # 白色文字
            
            # 绘制采点提示和状态
            if not self.select_corners_done:
                cv2.putText(temp_img, "Click 4 corners in order: TL->TR->BR->BL", (10, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(temp_img, "Selection Done! Click Reset to reselect", (10, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # 绘制已点击的角点和轮廓
            if self.clicked_corners:
                for i, (cx, cy) in enumerate(self.clicked_corners):
                    cv2.circle(temp_img, (cx, cy), 5, (0, 0, 255), -1)
                    cv2.putText(temp_img, str(i+1), (cx+10, cy),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                if len(self.clicked_corners) == 4:
                    corners = np.array(self.clicked_corners, dtype=np.int32)
                    cv2.drawContours(temp_img, [corners], -1, (0, 255, 0), 3)
                    cx_center = int(np.mean(corners[:, 0]))
                    cy_center = int(np.mean(corners[:, 1]))
                    cv2.circle(temp_img, (cx_center, cy_center), 7, (255, 0, 0), -1)
            
            # 显示图像
            self.is_drawing = True
            cv2.imshow(self.window_name, temp_img)
            cv2.waitKey(1)
            self.is_drawing = False
            
            rospy.sleep(0.03)  # 20Hz刷新频率

    def color_image_cb(self, msg):
        """彩色图像回调：仅缓存图像，不关闭窗口"""
        # with self.image_lock:
        #     self.cv_color_img = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        # # 移除原有调用show_image_and_wait_click的逻辑，改为由刷新线程处理
        cv_color_img = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")

        # try:
        #     results = self.model(
        #         cv_color_img,
        #         imgsz = 640,
        #         conf = 0.25,
        #         verbose = False
        #     )
        # except Exception as e:
        #     rospy.logerr("Yolo detect error %s", e)
        #     return
        

        # 1. ROS Image -> OpenCV
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("cv_bridge error: %s", e)
            return

        # 2. YOLO 推理
        try:
            results = self.model(
                cv_image,
                imgsz=640,
                conf=0.6,
                verbose=False,
            )
        except Exception as e:
            rospy.logerr("YOLO inference error: %s", e)
            return

        # 3. 画框可视化
        r = results[0]
        annotated = r.plot()  # numpy BGR

        # 4. 发布图像
        try:
            out_msg = self.cv_bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            out_msg.header = msg.header  # 保留时间戳和 frame_id
            self.image_pub.publish(out_msg)
        except CvBridgeError as e:
            rospy.logerr("cv_bridge publish error: %s", e)

        # 
        # pts0 = r.keypoints.xy[0].cpu().numpy()   # [K, 2]
        min_idx = torch.argmin(r.boxes.cls)
        print(min_idx)
        pts0 = r.keypoints.xy[min_idx].cpu().numpy().astype(np.int32)   # [K, 2]
        self.clicked_corners = pts0.tolist()
        # 当四个角点选完后，计算顶面中心像素坐标
        if len(self.clicked_corners) == 4:
            self.select_corners_done = True
            # 计算四个角点的平均坐标作为中心
            corners = np.array(self.clicked_corners, dtype=np.int32)
            cx = int(np.mean(corners[:, 0]))
            cy = int(np.mean(corners[:, 1]))
            self.block_top_center_pixel = (cx, cy)  # 关键赋值
            rospy.loginfo(f"四个角点已选完，顶面中心像素坐标：({cx}, {cy})")

        dx = corners[1, 0] - corners[0, 0]
        dy = corners[1, 1] - corners[0, 1]
        self.yaw_angle = math.atan2(dy, dx)

        # print(self.yaw_angle / math.pi * 180.0)


        # # 5. 简单发布检测信息（类别+置信度）
        # #   后面你可以改成发布自定义消息 / 目标坐标等
        # if r.boxes is not None and len(r.boxes) > 0:
        #     boxes = r.boxes
        #     cls = boxes.cls.cpu().numpy()
        #     conf = boxes.conf.cpu().numpy()
        #     xyxy = boxes.xyxy.cpu().numpy()

        #     texts = []
        #     for i in range(len(cls)):
        #         cls_id = int(cls[i])
        #         score = float(conf[i])
        #         x1, y1, x2, y2 = xyxy[i]
        #         texts.append(
        #             "id={} conf={:.2f} box=({:.1f},{:.1f},{:.1f},{:.1f})".format(
        #                 cls_id, score, x1, y1, x2, y2
        #             )
        #         )

        #     result_str = "\n".join(texts)
        # else:
        #     result_str = "no detections"

        # self.result_pub.publish(result_str)

        # # 如果你用 pose 模型，可以取关键点：
        # if hasattr(r, "keypoints") and r.keypoints is not None:
        #     kpts = r.keypoints.xy.cpu().numpy()   # [N, K, 2]
        #     # 在这里处理关键点逻辑
        



    def mouse_click_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # 检测是否点击Reset按钮
            btn_x, btn_y, btn_w, btn_h = self.btn_rect
            if (btn_x <= x <= btn_x + btn_w) and (btn_y <= y <= btn_y + btn_h):
                # 重置采点状态（包含重置block_top_center_pixel）
                self.clicked_corners = []
                self.select_corners_done = False
                self.block_top_center_pixel = None  # 重置
                self.block_top_center_cam = None
                self.plane_normal = None
                self.plane_params = None
                rospy.loginfo("已重置采点，可重新点击四个角点")
                return
            
            # 正常采点逻辑（仅在未完成时生效）
            if not self.select_corners_done and len(self.clicked_corners) < 4:
                self.clicked_corners.append((x, y))
                rospy.loginfo(f"已点击第 {len(self.clicked_corners)} 个角点：({x}, {y})")
                
                # 新增：当四个角点选完后，计算顶面中心像素坐标
                if len(self.clicked_corners) == 4:
                    self.select_corners_done = True
                    # 计算四个角点的平均坐标作为中心
                    corners = np.array(self.clicked_corners, dtype=np.int32)
                    cx = int(np.mean(corners[:, 0]))
                    cy = int(np.mean(corners[:, 1]))
                    self.block_top_center_pixel = (cx, cy)  # 关键赋值
                    rospy.loginfo(f"四个角点已选完，顶面中心像素坐标：({cx}, {cy})")



    def show_image_and_wait_click(self):
        """显示图像并等待用户点击四个角点"""
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1280, 720)
        cv2.setMouseCallback(self.window_name, self.mouse_click_callback)

        
        # 显示初始图像
        with self.image_lock:
            temp_img = self.cv_color_img.copy()
            cv2.putText(temp_img, "Click 4 corners in order: TL->TR->BR->BL", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow(self.window_name, temp_img)
        
        # 等待选点完成（最多等待30秒）
        start_time = rospy.Time.now()
        while not self.select_corners_done and not rospy.is_shutdown():
            # if (rospy.Time.now() - start_time).to_sec() > 30:
            #     rospy.logerr("选点超时（30秒），程序退出")
            #     cv2.destroyAllWindows()
            #     rospy.signal_shutdown("选点超时")
            #     return
            cv2.waitKey(100)
        
        # 选点完成，关闭窗口
        cv2.destroyAllWindows()
        rospy.loginfo("选点完成，开始计算顶面中心、平面法向量和平整度...")
    def is_point_in_polygon(self, point, polygon, min_edge_distance=5):
        """
        判断点是否在多边形内部，且距离所有边缘不小于min_edge_distance（像素单位）
        :param point: 待判断点 (u, v)
        :param polygon: 多边形顶点列表（像素坐标）
        :param min_edge_distance: 最小边缘距离（像素，默认5）
        :return: 满足条件返回True，否则False
        """
        x, y = point
        n = len(polygon)
        if n < 3:
            return False  # 多边形至少需要3个顶点
        
        # 第一步：射线法判断是否在多边形内部
        inside = False
        for i in range(n):
            j = (i + 1) % n
            xi, yi = polygon[i]
            xj, yj = polygon[j]
            
            # 射线与边相交判断
            if ((yi > y) != (yj > y)):
                x_intersect = ((y - yi) * (xj - xi)) / (yj - yi + 1e-8) + xi  # 避免除零
                if x < x_intersect:
                    inside = not inside
        if not inside:
            return False  # 不在内部直接返回
        
        # 第二步：计算点到多边形所有边缘的最小距离，必须 >= min_edge_distance
        min_dist = float('inf')
        for i in range(n):
            j = (i + 1) % n
            # 边缘的两个端点
            p1 = np.array(polygon[i], dtype=np.float32)
            p2 = np.array(polygon[j], dtype=np.float32)
            # 待判断点
            p = np.array([x, y], dtype=np.float32)
            
            # 计算点到线段的距离（向量法）
            vec_p1p2 = p2 - p1
            vec_p1p = p - p1
            # 投影长度（相对于线段p1p2）
            proj_len = np.dot(vec_p1p, vec_p1p2) / (np.dot(vec_p1p2, vec_p1p2) + 1e-8)  # 避免除零
            # 限制投影在[0,1]范围内（线段上的最近点）
            proj_len = max(0, min(1, proj_len))
            # 线段上的最近点
            closest = p1 + proj_len * vec_p1p2
            # 计算距离
            dist = np.linalg.norm(p - closest)
            
            # 更新最小距离
            if dist < min_dist:
                min_dist = dist
        
        # 只有最小距离 >= 设定阈值时，才认为有效
        return min_dist >= min_edge_distance

    def pointcloud_cb(self, msg):
        # print(f'get {self.camera_info_received} {self.select_corners_done} {self.block_top_center_pixel}')
        if not self.camera_info_received or not self.select_corners_done or self.block_top_center_pixel is None:
            return
        
        current_time = rospy.Time.now()
        time_last_exec = (current_time - self.last_exec_time).to_sec()
        print(time_last_exec)
        if time_last_exec < 5.0:
            # print(time_last_exec)
            return
        self.last_exec_time = current_time
        
        self.pointcloud = msg
        cx, cy = self.block_top_center_pixel
        corners_pixel = np.array(self.clicked_corners, dtype=np.float32)
        
        # # 新增：缓存四个角点的3D坐标（用于计算棱边方向）
        # self.corner_3d = []  # 存储四个角点的相机坐标 (x,y,z)
        # # 先收集所有点云的像素-3D映射（避免重复遍历）
        # print("\n------------------------------------ new msg -------------------------------\n")
        # pixel_to_3d = {}
        # for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        #     x, y, z = point
        #     uv = self.camera_model.project3dToPixel((x, y, z))
        #     u, v = int(uv[0]), int(uv[1])
        #     # print("point[%.2f, %.2f, %.2f]: [%.2f, %.2f]\n" % (x, y, z, u, v))
        #     pixel_to_3d[(u, v)] = (x, y, z)
        
        # # 为每个角点找最近的3D点
        # for (u, v) in self.clicked_corners:
        #     print("point[%.2f, %.2f]" % (u, v))
        #     # 搜索附近5x5像素范围内的点（避免精确匹配失败）
        #     found = False
        #     range_px = 5
        #     for du in range(-range_px, range_px+1):
        #         for dv in range(-range_px, range_px+1):
        #             if (u+du, v+dv) in pixel_to_3d:
        #                 self.corner_3d.append(pixel_to_3d[(u+du, v+dv)])
        #                 print("point[%.2f, %.2f]: [%.2f, %.2f, %.2f]\n" % (u+du, v+dv, pixel_to_3d[(u+du, v+dv)][0], pixel_to_3d[(u+du, v+dv)][1], pixel_to_3d[(u+du, v+dv)][2]))
        #                 found = True
        #                 break
        #         if found:
        #             break
        #     if not found:
        #         rospy.logwarn(f"角点({u},{v})未找到对应3D点，可能影响方向计算")
        #         self.corner_3d.append(None)
        
        # 提取顶面多边形内的点云（原有逻辑）
        top_points = []
        count_in=0
        count_out=0
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point
            uv = self.camera_model.project3dToPixel((x, y, z))
            u, v = uv[0], uv[1]
            if self.is_point_in_polygon((u, v), corners_pixel):
                top_points.append([x, y, z])
                count_in+=1
            else:
                count_out+=1
        print(f'点比例 {count_in}/{count_out}')
        
        if len(top_points) < 10:
            rospy.logwarn_throttle(1, "顶面点云数量不足（<10个），无法计算")
            return
        top_points = np.array(top_points)
        
        # 计算顶面中心和平面（原有逻辑）
        self.block_top_center_cam = np.mean(top_points, axis=0)
        rospy.loginfo(f"顶面中心（相机坐标）：x={self.block_top_center_cam[0]:.6f}, y={self.block_top_center_cam[1]:.6f}, z={self.block_top_center_cam[2]:.6f}")
        # self.fit_plane_and_normal(top_points)

        self.plane_normal = [0.0, 0.0, 1.0]

        rospy.loginfo(f"顶面法向量（单位化）：x={self.plane_normal[0]:.6f}, y={self.plane_normal[1]:.6f}, z={self.plane_normal[2]:.6f}")
        # self.flatness_error = np.mean(np.abs(self.plane_params[0]*top_points[:,0] + self.plane_params[1]*top_points[:,1] + self.plane_params[2]*top_points[:,2] + self.plane_params[3]) / np.linalg.norm(self.plane_params[:3]))
        # rospy.loginfo(f"顶面平整度误差：{self.flatness_error:.6f}米（<0.002米为平整）")
        


        # 发布结果
        # self.publish_result_image()
        self.publish_block_center_tf()  # 此处会使用新的XY轴计算逻辑
        # self.publish_plane_marker()



    def fit_plane_and_normal(self, points):
        """拟合平面并计算法向量（单位化）和平整度误差"""
        def plane_func(params, points):
            """平面方程：Ax + By + Cz + D = 0，返回点到平面的距离"""
            A, B, C, D = params
            return (A * points[:, 0] + B * points[:, 1] + C * points[:, 2] + D) / np.sqrt(A**2 + B**2 + C**2)
        
        # 初始参数（假设平面接近z=常数）
        init_params = [0, 0, 1, -np.mean(points[:, 2])]
        # 最小二乘拟合平面
        result = least_squares(plane_func, init_params, args=(points,))
        A, B, C, D = result.x

        # 新增：缓存平面方程参数（用于Marker生成）
        self.plane_params = (A, B, C, D)
        
        # 计算法向量并单位化（平面方程系数(A,B,C)即为法向量）
        normal = np.array([A, B, C])
        self.plane_normal = normal / np.linalg.norm(normal)  # 单位化
        
        # 计算平整度误差（平均距离）
        distances = np.abs(plane_func(result.x, points))
        self.flatness_error = np.mean(distances)
    def publish_plane_marker(self):
        """发布拟合平面的Marker（修复数值异常问题）"""
        if self.plane_params is None or self.block_top_center_cam is None:
            return
        
        # 获取平面方程参数和中心点
        A, B, C, D = self.plane_params
        center_x, center_y, center_z = self.block_top_center_cam
        
        # 1. 创建Marker对象
        plane_marker = Marker()
        plane_marker.header.stamp = rospy.Time.now()
        plane_marker.header.frame_id = self.target_frame
        plane_marker.ns = "block_top_plane"
        plane_marker.id = 0
        plane_marker.type = Marker.TRIANGLE_LIST
        plane_marker.action = Marker.ADD
        plane_marker.lifetime = rospy.Duration(0)
        
        # 修复：显式设置scale为非零值（1.0不影响实际大小，仅避免被RViz忽略）
        plane_marker.scale.x = 1.0
        plane_marker.scale.y = 1.0
        plane_marker.scale.z = 1.0
        
        # 修复：显式初始化四元数（保持不变）
        plane_marker.pose.orientation.w = 1.0
        plane_marker.pose.orientation.x = 0.0
        plane_marker.pose.orientation.y = 0.0
        plane_marker.pose.orientation.z = 0.0
        plane_marker.frame_locked = True
        
        # 2. 设置平面大小（半边长10cm）
        size = 0.05
        # 修复：更稳定的平面内向量计算（避免零向量）
        normal = np.array([A, B, C])
        normal_norm = np.linalg.norm(normal)
        if normal_norm < 1e-6:  # 避免法向量为零
            rospy.logwarn("平面法向量异常，跳过发布Marker")
            return
        normal = normal / normal_norm  # 单位化法向量
        
        # 生成两个与法向量垂直的向量（更稳定的方法）
        # 找到与法向量不共线的参考向量
        if np.abs(normal[0]) < np.abs(normal[1]) and np.abs(normal[0]) < np.abs(normal[2]):
            ref_vec = np.array([1.0, 0.0, 0.0])  # 参考向量：x轴
        elif np.abs(normal[1]) < np.abs(normal[2]):
            ref_vec = np.array([0.0, 1.0, 0.0])  # 参考向量：y轴
        else:
            ref_vec = np.array([0.0, 0.0, 1.0])  # 参考向量：z轴
        
        # 计算平面内的两个正交向量
        vec1 = np.cross(normal, ref_vec)  # 与法向量垂直
        vec1 = vec1 / np.linalg.norm(vec1) * size  # 单位化并缩放
        vec2 = np.cross(normal, vec1)     # 与法向量和vec1都垂直
        vec2 = vec2 / np.linalg.norm(vec2) * size  # 单位化并缩放
        
        # 3. 计算平面4个顶点（确保无零向量）
        p1 = np.array([center_x, center_y, center_z]) + vec1 + vec2
        p2 = np.array([center_x, center_y, center_z]) - vec1 + vec2
        p3 = np.array([center_x, center_y, center_z]) - vec1 - vec2
        p4 = np.array([center_x, center_y, center_z]) + vec1 - vec2
        
        # 4. 用两个三角形组成平面
        plane_marker.points = [
            Point(p1[0], p1[1], p1[2]),
            Point(p2[0], p2[1], p2[2]),
            Point(p3[0], p3[1], p3[2]),
            Point(p1[0], p1[1], p1[2]),
            Point(p3[0], p3[1], p3[2]),
            Point(p4[0], p4[1], p4[2])
        ]
        
        # 5. 设置平面颜色（半透明蓝色）
        color = ColorRGBA(0.0, 0.0, 1.0, 0.3)  # r, g, b, a
        plane_marker.colors = [color] * 6  # 6个点对应6个颜色
        
        # 6. 发布Marker
        self.plane_marker_pub.publish(plane_marker)


    def publish_result_image(self):
        """发布叠加结果的图像"""
        with self.image_lock:
            if self.cv_color_img is None:
                return
            temp_img = self.cv_color_img.copy()
        
        # 绘制角点和轮廓（仅在采点完成时绘制轮廓）
        if self.clicked_corners:
            # 绘制角点和序号
            for i, (x, y) in enumerate(self.clicked_corners):
                cv2.circle(temp_img, (x, y), 5, (0, 0, 255), -1)
                cv2.putText(temp_img, str(i+1), (x+10, y), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            # 仅当4个角点都选完时，才绘制轮廓和中心
            if len(self.clicked_corners) == 4 and self.block_top_center_pixel is not None:
                corners = np.array(self.clicked_corners, dtype=np.int32)
                cv2.drawContours(temp_img, [corners], -1, (0, 255, 0), 3)
                
                # 绘制中心
                cx, cy = self.block_top_center_pixel
                cv2.circle(temp_img, (cx, cy), 7, (255, 0, 0), -1)
                cv2.putText(temp_img, "Center", (cx+10, cy), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                
                # 绘制平整度和法向量信息（仅当计算完成后）
                if self.flatness_error is not None and self.plane_normal is not None:
                    flat_text = f"Flatness: {self.flatness_error:.6f}m"
                    normal_text = f"Normal: ({self.plane_normal[0]:.3f}, {self.plane_normal[1]:.3f}, {self.plane_normal[2]:.3f})"
                    cv2.putText(temp_img, flat_text, (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    cv2.putText(temp_img, normal_text, (10, 60), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        
        self.result_img_pub.publish(self.cv_bridge.cv2_to_imgmsg(temp_img, "bgr8"))


    def publish_block_center_tf(self):
        """发布TF：X轴沿左上→右上，Y轴沿右上→右下，Z轴为法向量"""
        # if self.block_top_center_cam is None or self.plane_normal is None or len(self.corner_3d) < 4:
        #     return
        
        # # 1. 检查四个角点的3D坐标是否有效
        # if None in self.corner_3d[:3]:  # 至少需要前三个角点（左上、右上、右下）
        #     rospy.logwarn("角点3D坐标不全，使用默认XY轴")
        #     # 回退到原有逻辑
        #     z_axis = self.plane_normal
        #     if np.abs(np.dot(z_axis, [1, 0, 0])) < 0.9:
        #         x_approx = [1, 0, 0]
        #     else:
        #         x_approx = [0, 1, 0]
        #     x_axis = np.cross(x_approx, z_axis)
        #     x_axis = x_axis / np.linalg.norm(x_axis)
        #     y_axis = np.cross(z_axis, x_axis)
        # else:
        #     # 2. 从角点提取棱边向量（相机坐标系下）
        #     tl, tr, br, bl = self.corner_3d  # 左上、右上、右下、左下
        #     # 上边向量：左上→右上（X轴方向）
        #     edge_x = np.array(tr) - np.array(tl)
        #     # 右边向量：右上→右下（Y轴方向）
        #     edge_y = np.array(br) - np.array(tr)

        #     print(edge_x)
            
        #     # 3. 投影到平面内（去除法向量方向分量）
        #     z_axis = self.plane_normal
        #     edge_x = edge_x - np.dot(edge_x, z_axis) * z_axis  # 投影到平面
        #     edge_y = edge_y - np.dot(edge_y, z_axis) * z_axis

        #     print(edge_x)
            
        #     # 4. 单位化并确保正交
        #     x_axis = edge_x / np.linalg.norm(edge_x) if np.linalg.norm(edge_x) > 1e-6 else np.array([1,0,0])
        #     # 强制Y轴与X轴、Z轴正交（右手系）
        #     y_axis = np.cross(z_axis, x_axis)
        #     y_axis = y_axis / np.linalg.norm(y_axis)


        if self.block_top_center_cam is None or self.plane_normal is None:
            return


        corners = np.array(self.clicked_corners, dtype=np.int32)
        dx = corners[1, 0] - corners[0, 0]
        dy = corners[1, 1] - corners[0, 1]
        x_axis = [dx, dy, 0]
        x_axis = x_axis / np.linalg.norm(x_axis) if np.linalg.norm(x_axis) > 1e-6 else np.array([1,0,0])
        z_axis = self.plane_normal
        y_axis = np.cross(z_axis, x_axis)
        
        # 5. 构造旋转矩阵
        rotation_matrix = np.array([
            [x_axis[0], y_axis[0], z_axis[0]],
            [x_axis[1], y_axis[1], z_axis[1]],
            [x_axis[2], y_axis[2], z_axis[2]]
        ])
        
        # 6. 旋转矩阵→四元数
        quaternion = tf_trans.quaternion_from_matrix(
            np.vstack([np.hstack([rotation_matrix, [[0], [0], [0]]]), [0, 0, 0, 1]])
        )
        
        # 7. 发布TF
        center_tf = TransformStamped()
        center_tf.header.stamp = rospy.Time.now()
        center_tf.header.frame_id = self.target_frame
        center_tf.child_frame_id = "block_top_center"
        center_tf.transform.translation.x = self.block_top_center_cam[0]
        center_tf.transform.translation.y = self.block_top_center_cam[1]
        center_tf.transform.translation.z = self.block_top_center_cam[2]
        center_tf.transform.rotation.x = quaternion[0]
        center_tf.transform.rotation.y = quaternion[1]
        center_tf.transform.rotation.z = quaternion[2]
        center_tf.transform.rotation.w = quaternion[3]
        self.tf_broadcaster.sendTransform(center_tf)
        rospy.logdebug_throttle(1, "已发布TF（XY轴沿角点棱边）")


if __name__ == '__main__':
    try:
        # 解决OpenCV线程问题
        import sys
        if sys.platform == 'linux':
            cv2.setNumThreads(0)
        
        detector = BlockDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        pass
