#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
import numpy as np
import time

def quaternion_multiply(q1, q2):
    """四元数乘法：q1 * q2（用于旋转向量）"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return [
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ]

def invert_transform(transform):
    """手动计算变换的逆变换（ROS Noetic 兼容）"""
    tx = transform.translation.x
    ty = transform.translation.y
    tz = transform.translation.z
    qx = transform.rotation.x
    qy = transform.rotation.y
    qz = transform.rotation.z
    qw = transform.rotation.w

    # 旋转的逆（共轭）
    inv_q = (qw, -qx, -qy, -qz)  # (w, x, y, z) 格式

    # 平移的逆
    t_quat = (0.0, tx, ty, tz)
    rotated_t = quaternion_multiply(inv_q, quaternion_multiply(t_quat, (qw, qx, qy, qz)))
    inv_tx, inv_ty, inv_tz = -rotated_t[1], -rotated_t[2], -rotated_t[3]

    # 构造逆变换
    inv_transform = TransformStamped()
    inv_transform.transform.translation.x = inv_tx
    inv_transform.transform.translation.y = inv_ty
    inv_transform.transform.translation.z = inv_tz
    inv_transform.transform.rotation.x = inv_q[1]
    inv_transform.transform.rotation.y = inv_q[2]
    inv_transform.transform.rotation.z = inv_q[3]
    inv_transform.transform.rotation.w = inv_q[0]

    return inv_transform

def is_identity_transform(transform, trans_thresh=1e-3, rot_thresh=1e-3):
    """判断变换是否为单位变换（平移和旋转均接近0）"""
    # 检查平移（各轴误差小于阈值）
    trans_error = np.sqrt(
        transform.translation.x**2 +
        transform.translation.y** 2 +
        transform.translation.z**2
    )
    if trans_error > trans_thresh:
        return False, f"平移误差过大: {trans_error:.6f}m"

    # 检查旋转（四元数接近单位四元数，误差小于阈值）
    # 单位四元数的模长为1，这里计算与(0,0,0,1)的距离
    rot_error = np.sqrt(
        transform.rotation.x**2 +
        transform.rotation.y** 2 +
        transform.rotation.z**2 +
        (transform.rotation.w - 1)** 2
    )
    if rot_error > rot_thresh:
        return False, f"旋转误差过大: {rot_error:.6f}"

    return True, "完全重合"

def main():
    rospy.init_node('calibrated_tf_publisher', anonymous=True)
    
    # 手动指定link名称（可根据实际情况修改）
    base_link = "realsense_link"
    optical_link = "realsense_color_optical_frame"
    calibrated_link = "camera_color_optical_frame"
    
    rospy.loginfo(f"TF 链路目标: {calibrated_link} -> {base_link} -> {optical_link}")
    rospy.loginfo(f"等待 TF 树中 {base_link} -> {optical_link} 的变换...")
    
    # TF监听器和广播器
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    time.sleep(3)  # 等待TF树初始化

    rate = rospy.Rate(100)  # 100Hz发布频率

    try:
        # 先获取 base_link -> optical_link 的变换
        base_to_optical = tf_buffer.lookup_transform(
            base_link,
            optical_link,
            rospy.Time(0),
            rospy.Duration(5.0)
        )
        rospy.loginfo(f"成功获取 {base_link} -> {optical_link} 的变换")
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr(f"获取 {base_link} -> {optical_link} 变换失败: {e}，程序退出")
        return

    # 计算逆变换并构造发布消息
    optical_to_base = invert_transform(base_to_optical.transform)
    calibrated_to_base = TransformStamped()
    calibrated_to_base.header.frame_id = calibrated_link
    calibrated_to_base.child_frame_id = base_link
    calibrated_to_base.transform = optical_to_base.transform

    # 定期检查重合状态的计数器（每100次循环检查一次，约1秒）
    check_counter = 0
    trans_thresh = 1e-6  # 平移误差阈值（米）
    rot_thresh = 1e-6    # 旋转误差阈值

    while not rospy.is_shutdown():
        # 发布TF
        calibrated_to_base.header.stamp = rospy.Time.now()  # 更新时间戳
        print(calibrated_to_base)
        tf_broadcaster.sendTransform(calibrated_to_base)

        # 每1秒检查一次重合状态
        check_counter += 1
        if check_counter >= 100:
            check_counter = 0
            try:
                # 获取 calibrated_link -> optical_link 的变换
                calib_to_optical = tf_buffer.lookup_transform(
                    calibrated_link,
                    optical_link,
                    rospy.Time(0),
                    rospy.Duration(1.0)
                )
                # 判断是否重合
                is_identity, msg = is_identity_transform(
                    calib_to_optical.transform,
                    trans_thresh,
                    rot_thresh
                )
                if is_identity:
                    rospy.loginfo(f"[{calibrated_link} 与 {optical_link}] {msg} (平移阈值: {trans_thresh}m, 旋转阈值: {rot_thresh})")
                else:
                    rospy.logwarn(f"[{calibrated_link} 与 {optical_link}] 未重合: {msg}")
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(f"检查重合状态失败: {e}")

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
