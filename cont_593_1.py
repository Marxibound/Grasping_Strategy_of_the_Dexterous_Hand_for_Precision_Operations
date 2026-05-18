#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import argparse
import rospy
import actionlib
import math
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


def solve_quintic_coeffs(delta, T):
    """
    求解5次多项式系数，满足边界条件：
    p(0)=0, p(T)=delta, v(0)=v(T)=0, a(0)=a(T)=0

    p(t) = c3*t^3 + c4*t^4 + c5*t^5

    返回: (c3, c4, c5)
    """
    T2 = T * T
    T3 = T2 * T
    T4 = T3 * T
    T5 = T4 * T

    # 方程组:
    # c3*T^3 + c4*T^4 + c5*T^5 = delta
    # 3*c3*T^2 + 4*c4*T^3 + 5*c5*T^4 = 0
    # 6*c3*T + 12*c4*T^2 + 20*c5*T^3 = 0

    # 高斯消元
    A = [
        [T3,   T4,   T5  ],
        [3*T2, 4*T3, 5*T4],
        [6*T,  12*T2,20*T3]
    ]
    b = [delta, 0.0, 0.0]

    for i in range(3):
        pivot = A[i][i]
        for j in range(i, 3):
            A[i][j] /= pivot
        b[i] /= pivot
        for k in range(i+1, 3):
            factor = A[k][i]
            for j in range(i, 3):
                A[k][j] -= factor * A[i][j]
            b[k] -= factor * b[i]

    c = [0.0] * 3
    for i in range(2, -1, -1):
        c[i] = b[i]
        for j in range(i+1, 3):
            c[i] -= A[i][j] * c[j]

    return c[0], c[1], c[2]


def eval_quintic(t, c3, c4, c5):
    """计算5次多项式在时刻t的位置、速度、加速度。"""
    p = c3*t**3 + c4*t**4 + c5*t**5
    v = 3*c3*t**2 + 4*c4*t**3 + 5*c5*t**4
    a = 6*c3*t + 12*c4*t**2 + 20*c5*t**3
    return p, v, a


def find_optimal_T(delta, v_max, a_max, T_guess):
    """
    找到满足速度/加速度约束的最小T。
    使用二分搜索。
    """
    if abs(delta) < 1e-12:
        return 0.0

    d = abs(delta)

    # 下界: 梯形速度最短时间
    # 如果 2*(v_max^2)/(2*a_max) >= d, 即 v_max^2/a_max >= d
    if v_max * v_max / a_max >= d:
        T_min = 2 * math.sqrt(d / a_max)
    else:
        T_min = d / v_max + v_max / a_max

    # 5次多项式比梯形慢，需要更长时间
    T_low = T_min * 1.5  # 保守下界
    T_high = max(T_guess, T_min * 10)

    c3, c4, c5 = solve_quintic_coeffs(d, T_high)
    # 检查T_high是否满足
    max_v, max_a = 0.0, 0.0
    for i in range(101):
        t = i * T_high / 100
        _, v, a = eval_quintic(t, c3, c4, c5)
        max_v = max(max_v, abs(v))
        max_a = max(max_a, abs(a))

    if max_v > v_max or max_a > a_max:
        # T_high不够，继续增大
        for _ in range(20):
            T_high *= 2
            c3, c4, c5 = solve_quintic_coeffs(d, T_high)
            max_v, max_a = 0.0, 0.0
            for i in range(101):
                t = i * T_high / 100
                _, v, a = eval_quintic(t, c3, c4, c5)
                max_v = max(max_v, abs(v))
                max_a = max(max_a, abs(a))
            if max_v <= v_max and max_a <= a_max:
                break

    # 二分搜索精确T
    for _ in range(30):
        T_mid = (T_low + T_high) / 2
        c3, c4, c5 = solve_quintic_coeffs(d, T_mid)
        max_v, max_a = 0.0, 0.0
        for i in range(101):
            t = i * T_mid / 100
            _, v, a = eval_quintic(t, c3, c4, c5)
            max_v = max(max_v, abs(v))
            max_a = max(max_a, abs(a))

        if max_v <= v_max and max_a <= a_max:
            T_high = T_mid
        else:
            T_low = T_mid

    return T_high


def generate_quintic_trajectory(start_positions, target_positions,
                                 v_max_list, a_max, duration_max, dt):
    """
    生成5次多项式轨迹，各关节可独立设置最大速度。

    参数:
        v_max_list: 各关节最大速度列表 [v0, v1, v2, v3, v4, v5]
        a_max:      全局最大加速度（或也改为列表）
        ...
    """
    deltas = [t - s for s, t in zip(start_positions, target_positions)]

    # 为每个关节找最优T（使用各自的最大速度）
    T_list = []
    coeff_list = []
    for idx, d in enumerate(deltas):
        if abs(d) < 1e-12:
            T_list.append(0.0)
            coeff_list.append((0.0, 0.0, 0.0))
            continue

        v_max = v_max_list[idx]
        T = find_optimal_T(d, v_max, a_max, duration_max)
        c3, c4, c5 = solve_quintic_coeffs(abs(d), T)
        T_list.append(T)
        coeff_list.append((c3, c4, c5))

    # 统一用最长的时间
    actual_duration = max(T_list)

    # 如果时间短的关节需要延长，重新缩放
    for idx, (d, T, coeffs) in enumerate(zip(deltas, T_list, coeff_list)):
        if abs(d) < 1e-12:
            continue
        if T < actual_duration:
            c3, c4, c5 = solve_quintic_coeffs(abs(d), actual_duration)
            coeff_list[idx] = (c3, c4, c5)

    # 如果超过duration_max，统一缩放
    if actual_duration > duration_max:
        scale = duration_max / actual_duration
        actual_duration = duration_max
        for idx, d in enumerate(deltas):
            if abs(d) < 1e-12:
                continue
            c3, c4, c5 = solve_quintic_coeffs(abs(d), actual_duration)
            coeff_list[idx] = (c3, c4, c5)

    num_points = int(actual_duration / dt)
    points = []

    for i in range(num_points + 1):
        t = i * dt
        positions, velocities, accelerations = [], [], []

        for idx, (delta, coeffs) in enumerate(zip(deltas, coeff_list)):
            if abs(delta) < 1e-12:
                positions.append(start_positions[idx])
                velocities.append(0.0)
                accelerations.append(0.0)
                continue

            c3, c4, c5 = coeffs
            sign = 1.0 if delta > 0 else -1.0
            p, v, a = eval_quintic(t, c3, c4, c5)
            positions.append(start_positions[idx] + sign * p)
            velocities.append(sign * v)
            accelerations.append(sign * a)

        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = velocities
        point.accelerations = accelerations
        point.time_from_start = rospy.Duration(t)
        points.append(point)

    return points, actual_duration


def print_trajectory(points, joint_names, dt, start_positions):
    """打印轨迹信息。"""
    print("=" * 80)
    print("轨迹预览（5次多项式，不执行）")
    print("=" * 80)
    print(f"总点数: {len(points)}")
    print(f"时间间隔: {dt*1000:.1f} ms")
    print(f"总时长: {points[-1].time_from_start.to_sec():.3f} s")
    print()

    header = f"{'时间(s)':>8}"
    for name in joint_names:
        header += f" {name[:8]:>10}"
    header += f" {'max_v':>8} {'max_a':>8}"
    print(header)
    print("-" * len(header))

    print_step = max(1, len(points) // 20)

    for i in range(0, len(points), print_step):
        pt = points[i]
        t = pt.time_from_start.to_sec()
        max_v = max(abs(v) for v in pt.velocities)
        max_a = max(abs(a) for a in pt.accelerations)

        row = f"{t:8.3f}"
        for j, name in enumerate(joint_names):
            pos = pt.positions[j]
            row += f" {pos:10.4f}"
        row += f" {max_v:8.4f} {max_a:8.4f}"
        print(row)

    if (len(points) - 1) % print_step != 0:
        pt = points[-1]
        t = pt.time_from_start.to_sec()
        max_v = max(abs(v) for v in pt.velocities)
        max_a = max(abs(a) for a in pt.accelerations)
        row = f"{t:8.3f}"
        for j, name in enumerate(joint_names):
            pos = pt.positions[j]
            row += f" {pos:10.4f}"
        row += f" {max_v:8.4f} {max_a:8.4f}"
        print(row)

    print()
    print("-" * 40)
    print("极值统计:")
    for j, name in enumerate(joint_names):
        positions = [pt.positions[j] for pt in points]
        velocities = [pt.velocities[j] for pt in points]
        accelerations = [pt.accelerations[j] for pt in points]

        print(f"  {name}:")
        print(f"    位移: {start_positions[j]:.4f} -> {positions[-1]:.4f} "
              f"(Δ={positions[-1]-start_positions[j]:.4f})")
        print(f"    最大速度: {max(abs(v) for v in velocities):.6f} rad/s")
        print(f"    最大加速度: {max(abs(a) for a in accelerations):.6f} rad/s^2")

    print()
    max_v_all = max(max(abs(v) for v in pt.velocities) for pt in points)
    max_a_all = max(max(abs(a) for a in pt.accelerations) for pt in points)
    print(f"全局最大速度: {max_v_all:.6f} rad/s")
    print(f"全局最大加速度: {max_a_all:.6f} rad/s^2")
    print("=" * 80)


def parse_args():
    parser = argparse.ArgumentParser(
        description='Aubo 5th-order trajectory planner with command line target input.'
    )
    parser.add_argument(
        '--target', nargs=6, type=float,
        metavar=('t0', 't1', 't2', 't3', 't4', 't5'),
        help='Target joint positions for 6 joints in radians.'
    )
    parser.add_argument(
        '--target-str', type=str,
        help='Comma-separated target positions, e.g. "-0.81,0.0,0.0,0.0,0.0,0.0".'
    )
    parser.add_argument(
        '--execute', action='store_true',
        help='Execute trajectory instead of previewing only.'
    )
    parser.add_argument(
        '--duration-max', type=float, default=60.0,
        help='Maximum trajectory duration in seconds.'
    )
    parser.add_argument(
        '--a-max', type=float, default=0.5,
        help='Maximum acceleration in rad/s^2.'
    )
    parser.add_argument(
        '--dt', type=float, default=0.01,
        help='Trajectory sample interval in seconds.'
    )
    parser.add_argument(
        '--v-max', nargs=6, type=float,
        metavar=('v0', 'v1', 'v2', 'v3', 'v4', 'v5'),
        default=[0.1, 0.2, 0.3, 0.5, 0.5, 0.5],
        help='Maximum velocity limits for each joint in rad/s.'
    )
    for idx in range(6):
        parser.add_argument(
            f'--j{idx}', type=float,
            help=f'Target angle for joint {idx} in radians; keep other joints unchanged.'
        )

    args, unknown = parser.parse_known_args(rospy.myargv(argv=sys.argv)[1:])
    if args.target_str:
        target_values = [float(x) for x in args.target_str.split(',') if x.strip()]
        if len(target_values) != 6:
            parser.error('target-str must contain exactly 6 comma-separated values')
        args.target = target_values

    return args


def plan_aubo_trajectory(start_positions, target_positions, 
                          v_max_list, a_max=1.0, 
                          duration_max=30.0, dt=0.01):
    """仅规划轨迹并打印，不执行运动。"""
    joint_names = [
        'shoulder', 'upperArm', 'foreArm',
        'wrist1', 'wrist2', 'wrist3'
    ]

    points, actual_duration = generate_quintic_trajectory(
        start_positions, target_positions, v_max_list, a_max, duration_max, dt
    )

    print_trajectory(points, joint_names, dt, start_positions)

    return points, actual_duration


def execute_aubo_trajectory(start_positions, target_positions, 
                             v_max_list, a_max=1.0, 
                             duration_max=30.0, dt=0.01):
    """执行轨迹运动。"""
    client = actionlib.SimpleActionClient(
        '/aubo_i5_controller/follow_joint_trajectory',
        FollowJointTrajectoryAction
    )

    rospy.loginfo("等待Action服务器...")
    if not client.wait_for_server(timeout=rospy.Duration(10)):
        rospy.logerr("连接失败")
        return False

    goal = FollowJointTrajectoryGoal()
    goal.trajectory = JointTrajectory()
    goal.trajectory.joint_names = [
        'shoulder_joint', 'upperArm_joint', 'foreArm_joint',
        'wrist1_joint', 'wrist2_joint', 'wrist3_joint'
    ]

    points, actual_duration = generate_quintic_trajectory(
        start_positions, target_positions, v_max_list, a_max, duration_max, dt
    )
    goal.trajectory.points = points

    max_v = max(max(abs(v) for v in pt.velocities) for pt in points)
    max_a = max(max(abs(a) for a in pt.accelerations) for pt in points)

    rospy.loginfo("发送5次多项式轨迹: %d 点, 实际耗时 %.2f 秒", len(points), actual_duration)
    rospy.loginfo("约束: a_max=%.2f, duration_max=%.1f", a_max, duration_max)
    rospy.loginfo("预估最大速度: %.4f rad/s, 最大加速度: %.4f rad/s^2", max_v, max_a)

    client.send_goal(goal)
    finished = client.wait_for_result(timeout=rospy.Duration(actual_duration + 20))

    if finished:
        state = client.get_state()
        rospy.loginfo("完成，状态: %d", state)
        return state == 3
    else:
        rospy.logwarn("超时")
        client.cancel_goal()
        return False

if __name__ == '__main__':
    args = parse_args()

    rospy.init_node('aubo_safe_move')

    js = rospy.wait_for_message('/joint_states', JointState, timeout=5)
    current = list(js.position)
    rospy.loginfo("当前: %s", current)

    if args.target is not None:
        target = list(args.target)
        rospy.loginfo("使用命令行目标: %s", target)
    else:
        target = current[:]
        for idx in range(6):
            joint_arg = getattr(args, f'j{idx}')
            if joint_arg is not None:
                target[idx] = joint_arg
        if any(getattr(args, f'j{idx}') is not None for idx in range(6)):
            rospy.loginfo("使用单关节/部分关节目标: %s", target)
        else:
            target = current[:]
            rospy.loginfo("未提供 target 或 jN 参数，使用默认目标: %s", target)

    v_max_list = args.v_max

    if args.execute:
        rospy.loginfo("3秒后开始执行...")
        rospy.sleep(3)
        execute_aubo_trajectory(
            current, target,
            v_max_list=v_max_list,
            a_max=args.a_max,
            duration_max=args.duration_max,
            dt=args.dt
        )
    else:
        rospy.loginfo("轨迹预览模式（不执行）...")
        plan_aubo_trajectory(
            current, target,
            v_max_list=v_max_list,
            a_max=args.a_max,
            duration_max=args.duration_max,
            dt=args.dt
        )
