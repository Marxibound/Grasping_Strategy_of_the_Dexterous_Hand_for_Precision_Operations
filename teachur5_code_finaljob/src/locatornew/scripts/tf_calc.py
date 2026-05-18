import rospy
import tf2_ros
import numpy as np

if __name__ == '__main__':
    rospy.init_node('tf_matrix_extractor')
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    rospy.sleep(1)  # 等待TF缓存
    
    try:
        # 获取tool0到camera_color_optical_frame的变换
        transform = tf_buffer.lookup_transform('tool0', 'camera_color_optical_frame', rospy.Time(0))
        
        # 提取平移（米）
        trans = [
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ]
        
        # 提取四元数（x,y,z,w）
        quat = [
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ]
        
        # 将四元数转换为旋转矩阵（3x3）
        from scipy.spatial.transform import Rotation as R
        rot_matrix = R.from_quat(quat).as_matrix()
        
        # 组合为4x4矩阵（平移转换为毫米）
        matrix_4x4 = np.eye(4)
        matrix_4x4[:3, :3] = rot_matrix
        matrix_4x4[:3, 3] = [t*1000 for t in trans]  # 米→毫米
        
        # 按你的格式打印
        print("TF实际对应的4x4矩阵（毫米）：")
        for row in matrix_4x4:
            print(f"{row[0]:.4f},{row[1]:.4f},{row[2]:.4f},{int(row[3])}")
            
    except Exception as e:
        print(f"获取TF失败：{e}")
