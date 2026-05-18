#!/usr/bin/env python3
"""
robot_gui_v2.py
基于 manager_v26510_1.py 的整合 GUI，具备：
- 深色现代界面
- 实时显示所有子进程终端输出（机械臂终端 / 灵巧手终端）
- 可一键执行完整抓取序列
- 自定义命令输入（支持 arm / hand 前缀）
"""

import sys
import time
import threading
import subprocess
import signal
import io
import pexpect
import faulthandler
faulthandler.enable()

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QTextEdit, QGroupBox, QSplitter, QLineEdit,
    QStatusBar
)
from PySide6.QtCore import Qt, Signal, QObject, QMetaObject, Q_ARG
from PySide6.QtGui import QFont, QColor, QTextCharFormat


# ================== 日志发射器 ==================
class LogEmitter(QObject):
    """用于跨线程安全地发送日志到 GUI"""
    arm_log = Signal(str)
    hand_log = Signal(str)


# 全局发射器，方便控制器内部调用
emitter = LogEmitter()


# ================== 日志 Writer（用于 pexpect logfile_read） ==================
class CallbackWriter(io.TextIOBase):
    """将写入内容转发到日志回调"""
    def __init__(self, callback):
        super().__init__()
        self.callback = callback

    def write(self, s):
        if s:
            self.callback(s)
        return len(s)

    def flush(self):
        pass

import json
import re
import openai

def parse_command(user_text, model="deepseek-chat", api_key="sk-c3f1b6069cb34056b17a2c3f92fa4725"):
    """
    调用 DeepSeek API 将自然语言指令解析为动作 JSON 序列。
    """
    client = openai.OpenAI(
        api_key=api_key,
        base_url="https://api.deepseek.com"  # DeepSeek API 地址
    )
    
    system_prompt = """你是一个机器人控制专家。请将用户的自然语言指令转换为精确的动作基元序列。
只返回 JSON 数组，不要解释。可用的动作基元：
- {"action": "arm_move_joints", "joints": [j0,...,j5]}  // 机械臂关节移动，6个弧度值
- {"action": "arm_move_joint", "joint_index": int, "value": float}  // 单关节增量移动
- {"action": "hand_set_angles", "angles": [a1,...,a6]}  // 灵巧手手指角度(0-1000)
- {"action": "hand_open"}  // 张开手
- {"action": "hand_close"}  // 闭合手
- {"action": "wait", "seconds": float}  // 等待
- {"action": "grasp_sequence"}  // 执行完整抓取序列（后备）
灵巧手张开: [999,999,999,999,999,100], 闭合: [50,50,50,50,900,100]。
根据任务合理生成动作序列。"""

    response = client.chat.completions.create(
        model=model,
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_text}
        ],
        temperature=0.1
    )
    
    content = response.choices[0].message.content
    # 提取 JSON（去除可能的 markdown 代码块标记）
    json_str = re.sub(r'```(?:json)?|```', '', content).strip()
    return json.loads(json_str)


# ================== 修改后的控制器类（支持日志回调和线程安全） ==================
class AuboController:
    """管理 Aubo 机械臂的 ROS launch 和关节移动，所有输出通过 log_cb 发送"""

    def __init__(self, log_cb=None):
        self.log = log_cb if log_cb else print
        self.ros_proc = None
        self.child = None
        self._stop_stdout_thread = False

        # 启动 ROS launch 在单独的线程中进行，避免阻塞调用者
        self._init_thread = threading.Thread(target=self._initialize, daemon=True)
        self._init_thread.start()

    def _initialize(self):
        try:
            self.ros_proc = self._start_roslaunch()
            self.log("[Arm] ROS launch 已启动，等待初始化...")
            # 启动线程读取 subprocess 输出
            threading.Thread(target=self._read_roslaunch_output, daemon=True).start()
            time.sleep(10)  # 等待 ROS 就绪

            self.child = pexpect.spawn('/bin/bash', encoding='utf-8', timeout=30)
            # 将 pexpect 读取的所有输出转发到日志
            self.child.logfile_read = CallbackWriter(self.log)

            self.child.sendline('cd ~/aubo_ws')
            self.child.expect(r'\$')
            self.child.sendline('source devel/setup.bash')
            self.child.expect(r'\$')
            self.child.sendline('cd ~/aubo_ws/src/scripts260505/scripts')
            self.child.expect(r'\$')
            self.log("[Arm] AuboController 初始化完成，环境已加载。")
        except Exception as e:
            self.log(f"[Arm] 初始化失败: {e}")

    def _start_roslaunch(self):
        cmd = (
            "cd ~/aubo_ws && "
            "source devel/setup.bash && "
            "roslaunch aubo_i5_moveit_config moveit_planning_execution.launch robot_ip:=127.0.0.1"
        )
        proc = subprocess.Popen(
            ['bash', '-c', cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN)
        )
        return proc

    def _read_roslaunch_output(self):
        """持续读取 roslaunch 的 stdout 并发送到日志"""
        try:
            for line in iter(self.ros_proc.stdout.readline, b''):
                if self._stop_stdout_thread:
                    break
                decoded = line.decode('utf-8', errors='replace').rstrip()
                if decoded:
                    self.log(decoded)
        except Exception:
            pass

    def move_to_joints(self, target_list, execute=False):
        if self.child is None or not self.child.isalive():
            self.log("[Arm] 机械臂未就绪，无法执行关节移动")
            return
        if len(target_list) != 6:
            raise ValueError("target_list 必须包含 6 个关节值")
        target_str = ','.join(f'{x:.6f}' for x in target_list)
        cmd = f'python3 cont_593_1.py --target-str "{target_str}"'
        if execute:
            cmd += ' --execute'
        self.log(f"[Arm] 执行: {cmd}")
        self.child.sendline(cmd)
        try:
            self.child.expect(r'\$', timeout=120)
            self.log("[Arm] 关节移动完成")
        except pexpect.TIMEOUT:
            self.log("[Arm] 命令超时，可能执行失败")

    def move_joints_custom(self, joint_dict, script_name='cont_593_1.py', execute=True):
        if self.child is None or not self.child.isalive():
            self.log("[Arm] 机械臂未就绪")
            return
        if not joint_dict:
            self.log("[Arm] joint_dict 为空，不执行操作")
            return
        args = []
        for j, angle in joint_dict.items():
            if not (0 <= j <= 5):
                raise ValueError(f"关节索引必须为 0~5，收到 {j}")
            args.append(f'--j{j} {angle:.6f}')
        cmd = f'python3 {script_name} ' + ' '.join(args)
        if execute:
            cmd += ' --execute'
        self.log(f"[Arm] 执行: {cmd}")
        self.child.sendline(cmd)
        try:
            self.child.expect(r'\$', timeout=120)
            self.log("[Arm] 自定义关节移动完成")
        except pexpect.TIMEOUT:
            self.log("[Arm] 命令超时")

    def close(self):
        self._stop_stdout_thread = True
        if self.child and self.child.isalive():
            self.child.close()
        if self.ros_proc:
            try:
                self.ros_proc.send_signal(signal.SIGINT)
                self.ros_proc.wait(timeout=5)
            except:
                pass
        self.log("[Arm] 机械臂环境已关闭")


class InspireHandController:
    """管理灵巧手，所有输出通过 log_cb 发送"""

    def __init__(self, log_cb=None, hand_side='left', test_flag=1, wait_time=10):
        self.log = log_cb if log_cb else print
        self.hand_side = hand_side
        self.service = f'/{hand_side}_inspire_hand/set_angle'
        self.ros_proc = None
        self.child = None
        self._stop_stdout_thread = False

        threading.Thread(target=self._initialize, args=(test_flag, wait_time), daemon=True).start()

    def _initialize(self, test_flag, wait_time):
        try:
            self.ros_proc = self._start_roslaunch(test_flag)
            self.log(f"[Hand] Inspire 手 ROS launch 已启动（{self.hand_side}），等待初始化...")
            threading.Thread(target=self._read_roslaunch_output, daemon=True).start()
            time.sleep(wait_time)

            self.child = pexpect.spawn('/bin/bash', encoding='utf-8', timeout=30)
            self.child.logfile_read = CallbackWriter(self.log)

            self.child.sendline('cd ~/inspire_ws')
            self.child.expect(r'\$')
            self.child.sendline('source devel/setup.bash')
            self.child.expect(r'\$')
            self.log(f"[Hand] InspireHandController 初始化完成，服务 {self.service} 可用。")
        except Exception as e:
            self.log(f"[Hand] 初始化失败: {e}")

    def _start_roslaunch(self, test_flag):
        cmd = (
            "cd ~/inspire_ws && "
            "source devel/setup.bash && "
            "sudo chmod a+rw /dev/ttyUSB0 && "
            f"roslaunch inspire_hand hand_control.launch test_flag:={test_flag}"
        )
        proc = subprocess.Popen(
            ['bash', '-c', cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN)
        )
        return proc

    def _read_roslaunch_output(self):
        try:
            for line in iter(self.ros_proc.stdout.readline, b''):
                if self._stop_stdout_thread:
                    break
                decoded = line.decode('utf-8', errors='replace').rstrip()
                if decoded:
                    self.log(decoded)
        except Exception:
            pass

    def set_angles(self, angles):
        if self.child is None or not self.child.isalive():
            self.log("[Hand] 灵巧手未就绪")
            return
        if len(angles) != 6:
            raise ValueError("angles 必须包含 6 个值")
        angle_str = ' '.join(str(a) for a in angles)
        cmd = f'rosservice call {self.service} {angle_str}'
        self.log(f"[Hand] 执行: {cmd}")
        self.child.sendline(cmd)
        try:
            self.child.expect(r'\$', timeout=30)
            self.log("[Hand] 手指角度设置完成")
        except pexpect.TIMEOUT:
            self.log("[Hand] 警告: 服务调用超时")

    def open_hand(self):
        self.log("[Hand] 执行张开手动作...")
        self.set_angles([999, 999, 999, 999, 999, 100])

    def close_hand(self):
        self.log("[Hand] 执行闭合手动作...")
        self.set_angles([50, 50, 50, 50, 900, 100])
        time.sleep(0.5)
        self.set_angles([50, 50, 50, 50, 100, 100])

    def close(self):
        self._stop_stdout_thread = True
        if self.child and self.child.isalive():
            self.child.close()
        if self.ros_proc:
            try:
                self.ros_proc.send_signal(signal.SIGINT)
                self.ros_proc.wait(timeout=5)
            except:
                pass
        self.log("[Hand] 灵巧手环境已关闭")


class RobotSystem:
    """统一调度机械臂和灵巧手（互斥运行）"""

    def __init__(self, arm_log_cb=None, hand_log_cb=None):
        self.arm_log = arm_log_cb if arm_log_cb else print
        self.hand_log = hand_log_cb if hand_log_cb else print
        self.arm = None
        self.hand = None

    def start_arm(self):
        """启动机械臂环境，若手正在运行则先关闭"""
        self.stop_hand()
        if self.arm is None:
            self.arm = AuboController(log_cb=self.arm_log)
            self.arm_log("[RobotSystem] 机械臂环境正在启动...")
        else:
            self.arm_log("[RobotSystem] 机械臂环境已在运行中")

    def start_hand(self, hand_side='left', test_flag=1, wait_time=10):
        """启动灵巧手环境，若臂正在运行则先关闭"""
        self.stop_arm()
        if self.hand is None:
            self.hand = InspireHandController(log_cb=self.hand_log,
                                              hand_side=hand_side,
                                              test_flag=test_flag,
                                              wait_time=wait_time)
            self.hand_log("[RobotSystem] 灵巧手环境正在启动...")
        else:
            self.hand_log("[RobotSystem] 灵巧手环境已在运行中")

    def stop_arm(self):
        if self.arm:
            self.arm.close()
            self.arm = None
            self.arm_log("[RobotSystem] 机械臂环境已停止")

    def stop_hand(self):
        if self.hand:
            self.hand.close()
            self.hand = None
            self.hand_log("[RobotSystem] 灵巧手环境已停止")

    def switch_to_arm(self):
        self.arm_log("\n>>> 切换到机械臂环境...")
        self.start_arm()
        # 注意：start_arm 内部会调用 stop_hand，所以只需记录

    def switch_to_hand(self, hand_side='left'):
        self.hand_log("\n>>> 切换到灵巧手环境...")
        self.start_hand(hand_side=hand_side)

    def shutdown(self):
        self.arm_log("\n=== 关闭所有环境 ===")
        self.hand_log("\n=== 关闭所有环境 ===")
        self.stop_arm()
        self.stop_hand()


# ================== GUI 主窗口 ==================
class MainWindow(QMainWindow):
    # 自定义信号
    plan_ready = Signal(str)        # 将 LLM 计划 JSON 文本传给主线程显示
    execute_enabled = Signal()      # 启用“确认执行”按钮
    execute_finished = Signal()     # 执行结束后禁用按钮
    status_msg = Signal(str)        # 安全更新状态栏

    def __init__(self):
        super().__init__()
        self.setWindowTitle("机械臂与灵巧手控制面板 v2.0 (LLM)")
        self.resize(1100, 700)

        # 系统实例
        self.system = RobotSystem(arm_log_cb=self.log_arm, hand_log_cb=self.log_hand)

        # 当前 LLM 计划
        self.current_plan = []

        # ⚠️ 先构建界面（创建 status_bar、plan_view 等所有控件）
        self.init_ui()

        # 然后再连接信号（此时 self.status_bar 等已存在）
        emitter.arm_log.connect(self.on_arm_log)
        emitter.hand_log.connect(self.on_hand_log)
        self.plan_ready.connect(self.on_plan_ready)
        self.execute_enabled.connect(lambda: self.btn_nl_execute.setEnabled(True))
        self.execute_finished.connect(lambda: self.btn_nl_execute.setEnabled(False))
        self.status_msg.connect(self.status_bar.showMessage)

        # 启动提示
        self.log_arm("控制面板已启动，等待用户操作...")
        self.log_hand("控制面板已启动，等待用户操作...")

    # ================== 日志方法（线程安全） ==================
    def log_arm(self, msg):
        emitter.arm_log.emit(msg)

    def log_hand(self, msg):
        emitter.hand_log.emit(msg)

    def on_arm_log(self, text):
        self.arm_console.append(text)
        self.scroll_to_bottom(self.arm_console)

    def on_hand_log(self, text):
        self.hand_console.append(text)
        self.scroll_to_bottom(self.hand_console)

    def scroll_to_bottom(self, console):
        scrollbar = console.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    # ================== UI 构建 ==================
    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)

        # 左侧控制面板
        left_panel = self.create_left_panel()
        left_panel.setFixedWidth(320)
        main_layout.addWidget(left_panel)

        # 右侧终端区域（上下分割）
        right_splitter = QSplitter(Qt.Vertical)

        # 机械臂终端
        arm_group = QGroupBox("机械臂终端 (ROS/MoveIt)")
        arm_layout = QVBoxLayout()
        self.arm_console = QTextEdit()
        self.arm_console.setReadOnly(True)
        self.arm_console.setFont(QFont("Courier New", 9))
        self.arm_console.setStyleSheet("background-color: #4169E1; color: #dcdcdc;")
        arm_layout.addWidget(self.arm_console)
        arm_group.setLayout(arm_layout)
        right_splitter.addWidget(arm_group)

        # 灵巧手终端
        hand_group = QGroupBox("灵巧手终端 (Inspire Hand)")
        hand_layout = QVBoxLayout()
        self.hand_console = QTextEdit()
        self.hand_console.setReadOnly(True)
        self.hand_console.setFont(QFont("Courier New", 9))
        self.hand_console.setStyleSheet("background-color: #4169E1; color: #dcdcdc;")
        hand_layout.addWidget(self.hand_console)
        hand_group.setLayout(hand_layout)
        right_splitter.addWidget(hand_group)

        right_splitter.setStretchFactor(0, 1)
        right_splitter.setStretchFactor(1, 1)
        main_layout.addWidget(right_splitter, 1)

        # 状态栏
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("就绪")

        # 全局样式
        self.setStyleSheet("""
            QMainWindow {
                background-color: #2b1a3d;
            }
            QGroupBox {
                font-weight: bold;
                color: #d4af37;
                border: 1px solid #8a2be2;
                border-radius: 5px;
                margin-top: 10px;
                padding-top: 15px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
            QPushButton {
                background-color: #8a2be2;
                color: white;
                border: 1px solid #555;
                border-radius: 4px;
                padding: 8px;
            }
            QPushButton:hover {
                background-color: #a45ee6;
            }
            QPushButton:pressed {
                background-color: #2a2a2a;
            }
            QLineEdit, QDoubleSpinBox {
                background-color: #3c2a4d;
                color: white;
                border: 1px solid #8a2be2;
                padding: 4px;
            }
            QLabel {
                color: #e0d0ff;
            }
        """)


    def create_left_panel(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)

        # ---------- 系统控制 ----------
        sys_group = QGroupBox("系统控制")
        sys_layout = QVBoxLayout()

        self.btn_connect_arm = QPushButton("启动 / 切换至机械臂")
        self.btn_connect_arm.clicked.connect(self.on_start_arm)
        sys_layout.addWidget(self.btn_connect_arm)

        self.btn_connect_hand = QPushButton("启动 / 切换至灵巧手")
        self.btn_connect_hand.clicked.connect(self.on_start_hand)
        sys_layout.addWidget(self.btn_connect_hand)

        self.btn_shutdown = QPushButton("关闭所有系统")
        self.btn_shutdown.clicked.connect(self.on_shutdown)
        sys_layout.addWidget(self.btn_shutdown)

        sys_group.setLayout(sys_layout)
        layout.addWidget(sys_group)

        # ---------- 自然语言指令 (LLM) ----------
        nl_group = QGroupBox("自然语言指令 (LLM)")
        nl_layout = QVBoxLayout()

        self.nl_input = QLineEdit()
        self.nl_input.setPlaceholderText("例如：把蓝色方块捏起来，旋转90度后放入红盒")
        self.nl_input.returnPressed.connect(self.on_nl_send)
        nl_layout.addWidget(self.nl_input)

        nl_layout.addWidget(QLabel("动作计划预览："))
        self.plan_view = QTextEdit()
        self.plan_view.setReadOnly(True)
        self.plan_view.setMaximumHeight(120)
        self.plan_view.setFont(QFont("Courier New", 8))
        nl_layout.addWidget(self.plan_view)

        btn_layout = QHBoxLayout()
        self.btn_nl_send = QPushButton("解析指令")
        self.btn_nl_send.clicked.connect(self.on_nl_send)
        self.btn_nl_execute = QPushButton("确认执行")
        self.btn_nl_execute.setEnabled(False)
        self.btn_nl_execute.clicked.connect(self.on_nl_execute)
        btn_layout.addWidget(self.btn_nl_send)
        btn_layout.addWidget(self.btn_nl_execute)
        nl_layout.addLayout(btn_layout)

        nl_group.setLayout(nl_layout)
        layout.addWidget(nl_group)

        # ---------- 快速操作 ----------
        seq_group = QGroupBox("快速操作")
        seq_layout = QVBoxLayout()

        self.btn_sequence = QPushButton("执行完整抓取放置序列")
        self.btn_sequence.clicked.connect(self.on_sequence)
        seq_layout.addWidget(self.btn_sequence)

        seq_group.setLayout(seq_layout)
        layout.addWidget(seq_group)

        # ---------- 自定义命令 ----------
        cmd_group = QGroupBox("自定义命令")
        cmd_layout = QVBoxLayout()

        cmd_hint = QLabel(
            "格式: arm move_joints j0..j5\n"
            "或 arm move_custom j_index val ...\n"
            "或 hand set_angles a1..a6\n"
            "或 hand open/close"
        )
        cmd_hint.setWordWrap(True)
        cmd_layout.addWidget(cmd_hint)

        self.cmd_input = QLineEdit()
        self.cmd_input.setPlaceholderText("输入命令，例如: arm move_joints 0 0 0 0 0 0")
        self.cmd_input.returnPressed.connect(self.on_send_command)
        cmd_layout.addWidget(self.cmd_input)

        self.btn_send = QPushButton("发送")
        self.btn_send.clicked.connect(self.on_send_command)
        cmd_layout.addWidget(self.btn_send)

        cmd_group.setLayout(cmd_layout)
        layout.addWidget(cmd_group)

        layout.addStretch()
        return panel

    # ================== 系统控制（原功能，已改为安全信号） ==================
    def on_start_arm(self):
        self.status_msg.emit("正在启动机械臂环境...")
        threading.Thread(target=self._start_arm, daemon=True).start()

    def _start_arm(self):
        try:
            self.system.switch_to_arm()
            time.sleep(0.5)
            self.status_msg.emit("机械臂环境已激活")
        except Exception as e:
            self.log_arm(f"启动机械臂失败: {e}")

    def on_start_hand(self):
        self.status_msg.emit("正在启动灵巧手环境...")
        threading.Thread(target=self._start_hand, daemon=True).start()

    def _start_hand(self):
        try:
            self.system.switch_to_hand()
            time.sleep(0.5)
            self.status_msg.emit("灵巧手环境已激活")
        except Exception as e:
            self.log_hand(f"启动灵巧手失败: {e}")

    def on_shutdown(self):
        self.status_msg.emit("正在关闭所有系统...")
        threading.Thread(target=self._shutdown, daemon=True).start()

    def _shutdown(self):
        self.system.shutdown()
        self.status_msg.emit("系统已关闭")

    def on_sequence(self):
        self.status_msg.emit("抓取序列执行中...")
        threading.Thread(target=self._run_sequence, daemon=True).start()

    def _run_sequence(self):
        try:
            # 第一步：手闭合，然后臂移动到观察位置
            self.log_hand(">>> 第一步：手闭合")
            self.system.switch_to_hand()
            if self.system.hand:
                self.system.hand.close_hand()
            else:
                self.log_hand("error: hand not available")
                return

            self.system.switch_to_arm()
            if self.system.arm:
                self.system.arm.move_to_joints(
                    [-0.868, -0.562, -1.692, -0.105, -1.775, 1.588], execute=True
                )
            else:
                self.log_arm("error: arm not available")
                return

            # 第二步：手张开
            self.log_hand(">>> 第二步：手张开")
            self.system.switch_to_hand()
            if self.system.hand:
                self.system.hand.open_hand()
            else:
                return

            # 第三步：臂移动到预抓取位置
            self.log_arm(">>> 第三步：臂移动序列")
            self.system.switch_to_arm()
            arm = self.system.arm
            if not arm:
                return
            arm.move_joints_custom({0: -1.0}, execute=True)
            arm.move_joints_custom({0: -1.315}, execute=True)
            arm.move_joints_custom({3: -0.831, 4: -0.011, 5: 1.588}, execute=True)
            arm.move_joints_custom({1: 0.0}, execute=True)
            arm.move_joints_custom({1: 0.45}, execute=True)
            arm.move_joints_custom({2: -1.642}, execute=True)
            arm.move_joints_custom({1: 0.827}, execute=True)

            # 第四步：手抓取
            self.log_hand(">>> 第四步：手抓取")
            self.system.switch_to_hand()
            if self.system.hand:
                self.system.hand.close_hand()
            else:
                return

            # 第五步：臂完成剩余动作
            self.log_arm(">>> 第五步：臂归位序列")
            self.system.switch_to_arm()
            arm = self.system.arm
            if not arm:
                return
            arm.move_joints_custom({1: 0.2}, execute=True)
            arm.move_joints_custom({1: -0.3}, execute=True)
            arm.move_joints_custom({1: -0.562}, execute=True)
            arm.move_joints_custom({0: -1.0}, execute=True)
            arm.move_joints_custom({0: -0.868}, execute=True)
            arm.move_joints_custom({2: -1.692}, execute=True)
            arm.move_joints_custom({3: -0.105, 4: -1.775}, execute=True)

            self.log_arm("\n=== 任务完成 ===")
            self.status_msg.emit("序列执行完毕")
        except Exception as e:
            self.log_arm(f"序列执行异常: {e}")

    def on_send_command(self):
        cmd = self.cmd_input.text().strip()
        if not cmd:
            return
        self.cmd_input.clear()
        threading.Thread(target=self._execute_command, args=(cmd,), daemon=True).start()

    def _execute_command(self, cmd):
        parts = cmd.split()
        if not parts:
            return
        target = parts[0].lower()
        if target not in ('arm', 'hand'):
            self.log_arm(f"未知目标: {target}，请以 'arm' 或 'hand' 开头")
            return

        action = parts[1].lower() if len(parts) > 1 else ''

        if target == 'arm':
            arm = self.system.arm
            if not arm:
                self.log_arm("[错误] 机械臂未启动，请先点击“启动机械臂”")
                return
            if action == 'move_joints':
                try:
                    joints = [float(x) for x in parts[2:8]]
                    arm.move_to_joints(joints, execute=True)
                except Exception as e:
                    self.log_arm(f"命令参数错误: {e}")
            elif action == 'move_custom':
                try:
                    kv = parts[2:]
                    if len(kv) % 2 != 0:
                        raise ValueError("需要偶数个参数（索引 值 成对）")
                    jdict = {}
                    for i in range(0, len(kv), 2):
                        j = int(kv[i])
                        val = float(kv[i+1])
                        jdict[j] = val
                    arm.move_joints_custom(jdict, execute=True)
                except Exception as e:
                    self.log_arm(f"自定义关节命令错误: {e}")
            else:
                self.log_arm(f"未知机械臂命令: {action}")
        elif target == 'hand':
            hand = self.system.hand
            if not hand:
                self.log_hand("[错误] 灵巧手未启动，请先点击“启动灵巧手”")
                return
            if action == 'set_angles':
                try:
                    angles = [float(x) for x in parts[2:8]]
                    hand.set_angles(angles)
                except Exception as e:
                    self.log_hand(f"命令参数错误: {e}")
            elif action == 'open':
                hand.open_hand()
            elif action == 'close':
                hand.close_hand()
            else:
                self.log_hand(f"未知灵巧手命令: {action}")

    # ================== LLM 自然语言指令处理 ==================
    def on_nl_send(self):
        text = self.nl_input.text().strip()
        if not text:
            return
        self.status_msg.emit("正在解析指令...")
        threading.Thread(target=self._parse_nl, args=(text,), daemon=True).start()

    def _parse_nl(self, text):
        """在工作线程中调用 LLM，不触碰 GUI 控件"""
        try:
            # parse_command 需自行实现（调用大模型）
            actions = parse_command(text)
            plan_json = json.dumps(actions, indent=2, ensure_ascii=False)
            self.current_plan = actions
            self.plan_ready.emit(plan_json)      # 主线程更新预览
            self.execute_enabled.emit()          # 主线程启用“确认执行”
            self.status_msg.emit("解析完成")
        except Exception as e:
            self.status_msg.emit(f"解析失败: {e}")

    def on_plan_ready(self, json_text):
        self.plan_view.setPlainText(json_text)

    def on_nl_execute(self):
        if not self.current_plan:
            return
        self.btn_nl_execute.setEnabled(False)
        self.status_msg.emit("正在执行指令...")
        threading.Thread(target=self._execute_plan, args=(self.current_plan,), daemon=True).start()

    def _execute_plan(self, plan):
        """在工作线程中执行 LLM 生成的动作序列"""
        try:
            for step in plan:
                action = step["action"]
                if action == "arm_move_joints":
                    self.system.arm.move_to_joints(step["joints"], execute=True)
                elif action == "arm_move_joint":
                    self.system.arm.move_joints_custom(
                        {step["joint_index"]: step["value"]}, execute=True
                    )
                elif action == "hand_set_angles":
                    self.system.hand.set_angles(step["angles"])
                elif action == "hand_open":
                    self.system.hand.open_hand()
                elif action == "hand_close":
                    self.system.hand.close_hand()
                elif action == "wait":
                    time.sleep(step["seconds"])
                elif action == "grasp_sequence":
                    # 可调用已有的抓取序列，或自行扩展
                    pass
                else:
                    self.log_arm(f"未知动作: {action}")
            self.log_arm("\n=== LLM 指令执行完毕 ===")
            self.status_msg.emit("执行完成")
        except Exception as e:
            self.log_arm(f"执行异常: {e}")
            self.status_msg.emit("执行出错")
        finally:
            self.execute_finished.emit()  # 安全地禁用按钮

    # ================== 窗口关闭 ==================
    def closeEvent(self, event):
        self.system.shutdown()
        event.accept()
        
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())