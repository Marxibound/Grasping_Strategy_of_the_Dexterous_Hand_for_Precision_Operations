#!/usr/bin/env python3
import rospy
import tf2_ros
import json
import os
import numpy as np
import threading
import time
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, Quaternion
from std_msgs.msg import ColorRGBA
import sys
import traceback
from moveit_commander import PlanningSceneInterface
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive


class PointSampler:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('rgi_tip_sampler', anonymous=True)
        
        # 初始化MoveIt规划场景接口（用于添加障碍物）
        self.scene = PlanningSceneInterface()
        rospy.sleep(1.0)  # 等待接口初始化
        
        # 配置核心参数
        self.json_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sampled_objects.json')
        self.marker_pub = rospy.Publisher('table_markers', Marker, queue_size=10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 状态变量初始化
        self.sampled_points = []  # 当前采样点列表
        self.objects = self.load_objects()  # 从JSON加载的对象
        self.current_mode = None  # 'plane'或'rectangle'
        self.running = True  # 线程运行标志
        self.objects_changed = True  # 对象变化标志
        
        # 启动Marker持续发布线程
        self.marker_thread = threading.Thread(target=self.continuous_publish_markers)
        self.marker_thread.daemon = True
        self.marker_thread.start()
        
        # 启动时将已保存对象添加到规划场景
        self.add_all_objects_to_planning_scene()
        
        # 打印启动信息
        rospy.loginfo("="*60)
        rospy.loginfo("RGI采点程序启动成功（支持MoveIt障碍物规划）")
        rospy.loginfo("="*60)
        rospy.loginfo("支持命令：")
        rospy.loginfo("  1. plane / 1 → 1点平面模式（2mx2m扁平体）")
        rospy.loginfo("  2. rect  / 2 → 2点矩形模式（自定义长方体）")
        rospy.loginfo("  3. sample / 3 → 采集rgi_tip位置（绿色临时点）")
        rospy.loginfo("  4. save [名称] → 保存对象（如：save table1）")
        rospy.loginfo("  5. delete [名称] → 删除对象（如：delete table1）")
        rospy.loginfo("  6. exit  / 6 → 退出程序")
        rospy.loginfo("="*60)

    def add_object_to_planning_scene(self, obj):
        """将对象转为MoveIt障碍物并添加到规划场景"""
        if not obj or 'name' not in obj:
            rospy.logwarn("无效对象，无法添加到规划场景")
            return
        
        # 构造CollisionObject消息
        col_obj = CollisionObject()
        col_obj.id = obj['name']
        col_obj.header.frame_id = "base_link"
        col_obj.operation = CollisionObject.ADD
        
        # 设置障碍物形状（长方体）
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [
            obj['scale']['x'],
            obj['scale']['y'],
            obj['scale']['z']
        ]
        col_obj.primitives.append(primitive)
        
        # 设置障碍物位姿
        pose = Pose()
        pose.position.x = obj['position']['x']
        pose.position.y = obj['position']['y']
        pose.position.z = obj['position']['z']
        pose.orientation.x = obj['orientation']['x']
        pose.orientation.y = obj['orientation']['y']
        pose.orientation.z = obj['orientation']['z']
        pose.orientation.w = obj['orientation']['w']
        col_obj.primitive_poses.append(pose)
        
        # 提交到规划场景
        self.scene.add_object(col_obj)
        rospy.loginfo(f"✅ 障碍物 '{obj['name']}' 已添加到MoveIt规划场景")

    def add_all_objects_to_planning_scene(self):
        """批量添加所有已保存对象到规划场景"""
        if not self.objects:
            rospy.loginfo("当前无已保存对象，无需添加到规划场景")
            return
        
        for obj in self.objects:
            self.add_object_to_planning_scene(obj)
        rospy.loginfo(f"📊 共添加 {len(self.objects)} 个对象到规划场景")

    def load_objects(self):
        """从JSON文件加载对象"""
        if os.path.exists(self.json_file):
            try:
                with open(self.json_file, 'r', encoding='utf-8') as f:
                    objects = json.load(f)
                rospy.loginfo(f"📥 从 {self.json_file} 加载到 {len(objects)} 个对象")
                return objects
            except json.JSONDecodeError:
                rospy.logwarn("❌ JSON文件损坏，将创建新文件")
            except Exception as e:
                rospy.logwarn(f"❌ 加载JSON失败：{str(e)}，将创建新文件")
        else:
            rospy.loginfo(f"📄 未找到JSON文件，将在保存时创建")
        return []

    def save_objects(self):
        """保存对象到JSON文件"""
        try:
            with open(self.json_file, 'w', encoding='utf-8') as f:
                json.dump(self.objects, f, indent=2, ensure_ascii=False)
            self.objects_changed = True  # 触发Marker重发
            rospy.loginfo(f"📤 对象已保存到 {self.json_file}（共 {len(self.objects)} 个）")
        except Exception as e:
            rospy.logerr(f"❌ 保存JSON失败：{str(e)}")

    def delete_object(self, name):
        """删除对象（同步删除JSON、Marker和规划场景）"""
        original_count = len(self.objects)
        self.objects = [obj for obj in self.objects if obj['name'] != name]
        
        if len(self.objects) == original_count:
            rospy.logwarn(f"❌ 未找到名称为 '{name}' 的对象，删除失败")
            return
        
        # 从规划场景删除
        self.scene.remove_world_object(name)
        rospy.loginfo(f"❌ 已从规划场景删除障碍物 '{name}'")
        
        # 保存并更新可视化
        self.save_objects()
        rospy.loginfo(f"✅ 已删除对象 '{name}'（剩余 {len(self.objects)} 个）")

    def get_rgi_tip_pose(self):
        """获取rgi_tip在base_link下的位姿"""
        try:
            trans = self.tf_buffer.lookup_transform(
                'base_link', 'rgi_tip', rospy.Time(0), rospy.Duration(1.0)
            )
            return {
                'position': {
                    'x': round(trans.transform.translation.x, 5),
                    'y': round(trans.transform.translation.y, 5),
                    'z': round(trans.transform.translation.z, 5)
                },
                'orientation': {
                    'x': trans.transform.rotation.x,
                    'y': trans.transform.rotation.y,
                    'z': trans.transform.rotation.z,
                    'w': trans.transform.rotation.w
                }
            }
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"❌ 获取rgi_tip TF失败：{str(e)}（检查TF是否正常发布）")
            return None

    def publish_marker(self, marker_id, marker_type, color, pose=None, scale=None, points=None):
        """发布单个Marker"""
        if rospy.is_shutdown() or not self.running:
            return
        
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "sampled_objects"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        
        if pose:
            marker.pose = pose
        if scale:
            marker.scale = scale
        marker.points = points if points else []
        marker.color = color
        marker.lifetime = rospy.Duration(2.0)  # 延长生命周期避免闪烁
        # print('pub ',marker)
        self.marker_pub.publish(marker)

    def publish_all_objects(self):
        """发布所有对象的可视化Marker"""
        if not self.objects:
            return
        
        for obj_idx, obj in enumerate(self.objects):
            try:
                # 发布长方体
                obj_color = ColorRGBA(r=0.8, g=0.1, b=0.1, a=1.0)
                obj_pose = Pose()
                obj_pose.position.x = obj['position']['x']
                obj_pose.position.y = obj['position']['y']
                obj_pose.position.z = obj['position']['z']
                obj_pose.orientation = Quaternion(
                    x=obj['orientation']['x'],
                    y=obj['orientation']['y'],
                    z=obj['orientation']['z'],
                    w=obj['orientation']['w']
                )
                obj_scale = Point(
                    x=obj['scale']['x'],
                    y=obj['scale']['y'],
                    z=obj['scale']['z']
                )
                self.publish_marker(
                    obj_idx + 1, Marker.CUBE, obj_color, obj_pose, obj_scale
                )
                
                # 发布采样点（红色小球）
                point_color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
                point_scale = Point(x=0.01, y=0.01, z=0.01)
                for pt_idx, pt in enumerate(obj['sampled_points']):
                    pt_pose = Pose()
                    pt_pose.position.x = pt['x']
                    pt_pose.position.y = pt['y']
                    pt_pose.position.z = pt['z']
                    self.publish_marker(
                        1000 + obj_idx * 10 + pt_idx,
                        Marker.SPHERE,
                        point_color,
                        pt_pose,
                        point_scale
                    )
            except Exception as e:
                rospy.logerr(f"❌ 发布对象 '{obj.get('name', '未知')}' 失败：{str(e)}")

    def continuous_publish_markers(self):
        """持续发布Marker的线程（10Hz）"""
        rospy.loginfo("🔄 Marker发布线程已启动（10Hz）")
        rate = rospy.Rate(10)
        while self.running and not rospy.is_shutdown():
            try:
                if self.marker_pub.get_num_connections() == 0:
                    rate.sleep()
                    continue
                
                if self.objects_changed:
                    # 删除旧Marker
                    clear_marker = Marker()
                    clear_marker.header.frame_id = "base_link"
                    clear_marker.header.stamp = rospy.Time.now()
                    clear_marker.ns = "sampled_objects"
                    clear_marker.id = 0
                    clear_marker.action = Marker.DELETEALL
                    self.marker_pub.publish(clear_marker)
                    time.sleep(0.05)
                    
                    # 重新发布所有对象
                    self.publish_all_objects()
                    self.objects_changed = False
                else:
                    # 直接发布现有对象
                    self.publish_all_objects()
                
                rate.sleep()
            except Exception as e:
                tb = traceback.extract_tb(sys.exc_info()[2])
                last_tb = tb[-1] if tb else ("未知文件", 0, "未知函数")
                rospy.logerr(f"❌ Marker线程错误 | 文件：{os.path.basename(last_tb[0])} | 行号：{last_tb[1]} | 信息：{str(e)}")
                time.sleep(0.5)

    def calculate_plane(self, points):
        """计算1点平面的参数（2mx2m扁平体）"""
        center = points[0]
        return {
            'position': {
                'x': center['x'],
                'y': center['y'],
                'z': center['z']/2
            },
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
            'scale': {'x': 2.0, 'y': 2.0, 'z': center['z']},
            'volume': round(2.0 * 2.0 * 0.01, 6)
        }

    def calculate_rectangle(self, points):
        """计算2点矩形的参数（自定义长方体）"""
        p1 = np.array([points[0]['x'], points[0]['y'], points[0]['z']])
        p2 = np.array([points[1]['x'], points[1]['y'], points[1]['z']])
        
        center = (p1 + p2) / 2
        length = abs(p2[0] - p1[0])
        width = abs(p2[1] - p1[1])
        height = round((p1[2] + p2[2]) / 2, 3)
        
        return {
            'position': {
                'x': round(center[0], 3),
                'y': round(center[1], 3),
                'z': round(height / 2, 3)
            },
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
            'scale': {
                'x': round(length, 3),
                'y': round(width, 3),
                'z': round(height, 3)
            },
            'volume': round(length * width * height, 6)
        }

    def run(self):
        """主循环：处理用户输入"""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.running:
            try:
                command = input("\n请输入命令（输入 'exit' 退出）：").strip()
                if not command:
                    continue
                
                # 1. 切换到1点平面模式
                if command == 'plane' or command == '1':
                    self.current_mode = 'plane'
                    self.sampled_points = []
                    rospy.loginfo("🔄 已切换到【1点平面模式】，请采集1个点（输入'sample'）")
                
                # 2. 切换到2点矩形模式
                elif command == 'rect' or command == '2':
                    self.current_mode = 'rectangle'
                    self.sampled_points = []
                    rospy.loginfo("🔄 已切换到【2点矩形模式】，请采集2个点（输入'sample'）")
                
                # 3. 采集rgi_tip位置
                elif command == 'sample' or command == '3':
                    if not self.current_mode:
                        rospy.logwarn("⚠️ 请先选择模式（plane/rect 或输入1/2）")
                        continue
                    
                    pose = self.get_rgi_tip_pose()
                    if pose:
                        point = pose['position']
                        self.sampled_points.append(point)
                        rospy.loginfo(f"📌 已采集点 {len(self.sampled_points)}: ({point['x']:.5f}, {point['y']:.5f}, {point['z']:.5f})")
                        
                        # 发布绿色临时点
                        temp_color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
                        temp_pose = Pose()
                        temp_pose.position.x = point['x']
                        temp_pose.position.y = point['y']
                        temp_pose.position.z = point['z']
                        self.publish_marker(
                            999, Marker.SPHERE, temp_color,
                            temp_pose, Point(x=0.05, y=0.05, z=0.05)
                        )
                        
                        # 检查是否采集完成
                        required = 1 if self.current_mode == 'plane' else 2
                        if len(self.sampled_points) == required:
                            rospy.loginfo(f"✅ 已采集足够点数（{required}个），输入 'save [名称]' 保存")
                
                # 4. 保存对象
                elif command.startswith('save '):
                    parts = command.split(' ', 1)
                    if len(parts) < 2 or not parts[1].strip():
                        rospy.logwarn("⚠️ 请指定对象名称（格式：save 名称）")
                        continue
                    name = parts[1].strip()
                    
                    if not self.current_mode:
                        rospy.logwarn("⚠️ 请先选择模式并采集足够点数")
                        continue
                    required = 1 if self.current_mode == 'plane' else 2
                    if len(self.sampled_points) < required:
                        rospy.logwarn(f"⚠️ 需采集{required}个点（当前：{len(self.sampled_points)}个）")
                        continue
                    
                    # 覆盖同名对象
                    self.objects = [obj for obj in self.objects if obj['name'] != name]
                    
                    # 计算对象参数
                    if self.current_mode == 'plane':
                        obj_data = self.calculate_plane(self.sampled_points)
                    else:
                        obj_data = self.calculate_rectangle(self.sampled_points)
                    
                    # 保存新对象
                    new_obj = {
                        'name': name,
                        'type': self.current_mode,
                        'sampled_points': self.sampled_points,
                        'position': obj_data['position'],
                        'orientation': obj_data['orientation'],
                        'scale': obj_data['scale'],
                        'volume': obj_data['volume']
                    }
                    self.objects.append(new_obj)
                    self.save_objects()
                    
                    # 添加到规划场景
                    self.add_object_to_planning_scene(new_obj)
                    
                    rospy.loginfo(f"📦 已保存{self.current_mode}对象: {name}，体积: {obj_data['volume']:.5f} m³")
                    
                    # 重置状态
                    self.current_mode = None
                    self.sampled_points = []
                
                # 5. 删除对象
                elif command.startswith('delete '):
                    parts = command.split(' ', 1)
                    if len(parts) < 2 or not parts[1].strip():
                        rospy.logwarn("⚠️ 请指定对象名称（格式：delete 名称）")
                        continue
                    name = parts[1].strip()
                    self.delete_object(name)
                
                # 6. 退出程序
                elif command == 'exit' or command == '6':
                    rospy.loginfo("👋 程序正在退出...")
                    self.running = False
                    time.sleep(0.5)
                    rospy.loginfo("✅ 程序退出成功！")
                    sys.exit(0)
                
                # 未知命令
                else:
                    rospy.loginfo("❓ 未知命令，请参考支持的命令列表")
            
            except Exception as e:
                tb = traceback.extract_tb(sys.exc_info()[2])
                last_tb = tb[-1] if tb else ("未知文件", 0, "未知函数")
                rospy.logerr(f"❌ 命令处理错误 | 文件：{os.path.basename(last_tb[0])} | 函数：{last_tb[2]} | 行号：{last_tb[1]} | 信息：{str(e)}")
            
            rate.sleep()


if __name__ == '__main__':
    try:
        sampler = PointSampler()
        sampler.run()
    except rospy.ROSInterruptException:
        if 'sampler' in locals():
            sampler.running = False  # 确保线程停止
        rospy.loginfo("程序被中断，已退出")
