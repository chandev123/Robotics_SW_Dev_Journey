#!/usr/bin/env python3
"""
Task Runner UI Node with Firebase Integration
터미널 기반 로봇 모니터링 및 태스크 제어 UI + Firebase 연동
- 로봇 조인트 상태 실시간 표시
- 로봇 상태 모니터링
- 태스크 시작/정지 제어
- Firebase Realtime Database 연동
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool, Int32
from dsr_msgs2.srv import SetRobotMode  # Safe Stop 해제용
import firebase_admin
from firebase_admin import credentials, db
import threading
import sys
import math
import os
import time
from datetime import datetime

# Robot Mode Constants
ROBOT_MODE_AUTONOMOUS = 1
ROBOT_MODE_MANUAL = 0

# ANSI 색상 코드
class Colors:
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class TaskRunnerUI(Node):
    """
    Task Runner 모니터링 및 제어 UI + Firebase 연동
    
    Subscriptions:
        - /dsr01/joint_states: 로봇 조인트 상태
        - /dsr01/error: 로봇 에러 메시지
        - /dsr01/robot_disconnection: 로봇 연결 상태
        - /rt_topic/external_tcp_force: 힘/토크 센서
        
    Publishers:
        - /dsr01/task_command: 태스크 제어 명령 (START, STOP, PAUSE)
    
    Firebase:
        - /task_runner: 실시간 데이터 업로드
        - /task_runner/command: 웹 UI 명령 수신
    """

    def add_log(self, msg):
        """로그 메시지 출력 (타임스탬프 포함)"""
        timestamp = datetime.now().strftime('%H:%M:%S')
        print(f"[{timestamp}] {msg}")
        sys.stdout.flush()

    def print_banner(self):
        """시작 시 배너 한 번 출력"""
        print(f"\n{Colors.BOLD}{Colors.CYAN}" + "=" * 50)
        print("    🤖 Project Hit - Task Runner UI")
        print("=" * 50 + f"{Colors.ENDC}")
        print(f"\n{Colors.BOLD}⌨️  키보드 명령:{Colors.ENDC}")
        print(f"  {Colors.GREEN}[S]{Colors.ENDC} START   {Colors.YELLOW}[P]{Colors.ENDC} PAUSE   {Colors.RED}[E]{Colors.ENDC} EMERGENCY")
        print(f"  {Colors.CYAN}[H]{Colors.ENDC} HOME    {Colors.CYAN}[R]{Colors.ENDC} RESET   {Colors.BLUE}[Q]{Colors.ENDC} QUIT")
        print(f"\n{Colors.CYAN}" + "-" * 50 + f"{Colors.ENDC}")
        print(f"{Colors.BOLD}📋 명령 로그:{Colors.ENDC}\n")
        sys.stdout.flush()
    
    def __init__(self):
        super().__init__('task_runner_ui')
        
        # ========== Firebase Configuration ==========
        # 여러 경로 시도 (소스 경로 -> 설치 경로)
        possible_paths = [
            os.path.expanduser("~/cobot_ws/src/project_hit/rokey-b-3-firebase-adminsdk-fbsvc-9a6aa4e40c.json"),
            "./src/project_hit/rokey-b-3-firebase-adminsdk-fbsvc-9a6aa4e40c.json",
        ]
        SERVICE_ACCOUNT_KEY_PATH = None
        for p in possible_paths:
            if os.path.exists(p):
                SERVICE_ACCOUNT_KEY_PATH = p
                break
        
        DATABASE_URL = "https://rokey-b-3-default-rtdb.firebaseio.com/"
        
        # ========== Firebase Initialization ==========
        self.firebase_enabled = False
        try:
            if SERVICE_ACCOUNT_KEY_PATH is None:
                raise FileNotFoundError("Firebase 키 파일을 찾을 수 없습니다")
            
            print(f"[Firebase] Using key: {SERVICE_ACCOUNT_KEY_PATH}")
            
            if not firebase_admin._apps:
                cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
                firebase_admin.initialize_app(cred, {'databaseURL': DATABASE_URL})
            self.ref = db.reference('/task_runner')
            self.command_ref = db.reference('/task_runner/command')
            self.firebase_enabled = True
            self.get_logger().info(f"[Firebase Connected] {DATABASE_URL}")
            print(f"[Firebase] ✅ Connected to {DATABASE_URL}")
        except Exception as e:
            self.get_logger().warn(f"[Firebase Init Failed] {e} - 오프라인 모드로 실행")
            print(f"[Firebase] ❌ Init Failed: {e}")
        
        # ========== QoS 설정 ==========
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        command_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        
        # ========== 데이터 저장 ==========
        self.joint_positions_deg = [0.0] * 6  # J1-J6 (degrees)
        self.joint_velocities = [0.0] * 6
        self.joint_names = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
        self.robot_connected = True
        self.last_error = ""
        self.task_status = "IDLE"
        self.last_update_time = None
        self.running = True
        

        
        # ========== Subscribers ==========
        self.create_subscription(
            JointState, 
            '/dsr01/joint_states', 
            self.cb_joint_states, 
            qos_profile
        )
        
        self.create_subscription(
            String,
            '/dsr01/error',
            self.cb_error,
            10
        )
        
        self.create_subscription(
            Bool,
            '/dsr01/robot_disconnection',
            self.cb_disconnection,
            10
        )
        
        
        # ========== Publishers ==========
        self.task_cmd_pub = self.create_publisher(String, '/dsr01/task_command', command_qos_profile)
        self.drl_stop_pub = self.create_publisher(Int32, '/drl_stop_cmd', command_qos_profile)
        
        # ========== Service Clients (Safe Stop 해제용) ==========
        self.set_robot_mode_cli = self.create_client(SetRobotMode, '/dsr01/system/set_robot_mode')
        
        # ========== Firebase 명령 추적 ==========
        self.last_firebase_command = None
        self.last_firebase_timestamp = None  # timestamp로 중복 감지
        
        # 표시 타이머 제거 - 배너만 한 번 출력
        
        # ========== Firebase 업로드 타이머 (5Hz) ==========
        if self.firebase_enabled:
            self.create_timer(0.2, self.upload_to_firebase)
            self.create_timer(1.0, self.poll_firebase_command)
        
        self.get_logger().info("Task Runner UI 시작됨")
    
    def cb_joint_states(self, msg: JointState):
        """조인트 상태 콜백 - 라디안을 도(degree)로 변환"""
        try:
            self.last_update_time = datetime.now()
            
            # joint_states의 순서가 다를 수 있으므로 이름으로 매핑
            name_to_idx = {}
            for i, name in enumerate(msg.name):
                # joint_1, joint_2, ... 형식
                if 'joint_' in name:
                    joint_num = int(name.split('_')[1])
                    name_to_idx[joint_num] = i
            
            # J1-J6 순서대로 저장
            for j in range(1, 7):
                if j in name_to_idx:
                    idx = name_to_idx[j]
                    if idx < len(msg.position):
                        self.joint_positions_deg[j-1] = math.degrees(msg.position[idx])
                    if idx < len(msg.velocity):
                        self.joint_velocities[j-1] = msg.velocity[idx]
                        
        except Exception as e:
            self.get_logger().warn(f"Joint states 처리 오류: {e}")
    
    def cb_error(self, msg: String):
        """에러 메시지 콜백"""
        self.last_error = msg.data
        self.get_logger().error(f"로봇 에러: {msg.data}")
    
    def cb_disconnection(self, msg: Bool):
        """연결 상태 콜백"""
        self.robot_connected = not msg.data
        if msg.data:
            self.get_logger().warn("로봇 연결 끊김!")
    

    
    def send_command(self, cmd: str):
        """태스크 명령 전송"""
        # RESET 명령은 별도 처리 (Safe Stop 해제)
        if cmd.upper() == "RESET":
            self.release_safe_stop()
            return
            
        msg = String()
        msg.data = cmd.upper()
        self.task_cmd_pub.publish(msg)
        self.get_logger().info(f"📤 [UI -> Robot] 명령 전송: {cmd}")
        self.add_log(f"📤 명령 발신: {cmd}")

    def release_safe_stop(self):
        """Safe Stop (Protective Stop) 상태 해제 - set_robot_mode(AUTONOMOUS) 호출"""
        if not self.set_robot_mode_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("❌ set_robot_mode 서비스를 찾을 수 없습니다")
            self.add_log("❌ RESET 실패: 서비스 없음")
            return
        
        req = SetRobotMode.Request()
        req.robot_mode = ROBOT_MODE_AUTONOMOUS  
        
        future = self.set_robot_mode_cli.call_async(req)
        future.add_done_callback(self._on_reset_complete)
        
        self.get_logger().info("🔄 [UI -> Robot] Safe Stop 해제 요청...")
        self.add_log("🔄 Safe Stop 해제 요청...")
    
    def _on_reset_complete(self, future):
        """Safe Stop 해제 완료 콜백"""
        
        try:
            result = future.result()
            
            if result:
                self.get_logger().info("✅ Safe Stop 해제 완료")
                self.add_log("✅ Safe Stop 해제 완료")
            else:
                self.get_logger().warn("⚠️ Safe Stop 해제 응답 없음")
        except Exception as e:
            self.get_logger().error(f"❌ Safe Stop 해제 실패: {e}")
            self.add_log(f"❌ Safe Stop 해제 실패: {e}")

    def send_drl_stop(self):
        """DRL 즉시 정지 명령 전송"""
        msg = Int32()
        msg.data = 1
        self.drl_stop_pub.publish(msg)
        self.get_logger().warn("🛑 [UI -> Robot] DRL STOP 전송 (/drl_stop_cmd)")
        self.add_log("🛑 DRL STOP 발신")


    def upload_to_firebase(self):
        """Firebase에 현재 데이터 업로드 (5Hz)"""
        if not self.firebase_enabled:
            return
        try:
            data = {
                "status": self.task_status,
                "joints": [round(j, 2) for j in self.joint_positions_deg],
                "connected": self.robot_connected,
                "error": self.last_error,
                "timestamp": time.time(),
            }
            self.ref.update(data)
        except Exception as e:
            self.get_logger().debug(f"Firebase upload error: {e}")
    
    def poll_firebase_command(self):
        """Firebase에서 명령 폴링 (1Hz)"""
        if not self.firebase_enabled:
            return
        try:
            command_data = self.command_ref.get()
            if command_data and isinstance(command_data, dict):
                cmd = command_data.get("cmd", "")
                timestamp = command_data.get("timestamp", "")
                
                # timestamp가 다르면 새로운 명령으로 인식 (같은 명령도 재전송 가능)
                if cmd and timestamp != self.last_firebase_timestamp:
                    self.last_firebase_timestamp = timestamp
                    self.last_firebase_command = cmd
                    self.get_logger().info(f"🔔 [Firebase] 명령 수신: {cmd} (ts: {timestamp})")
                    self.add_log(f"🔔 Firebase 명령: {cmd}")
                    self.send_command(cmd)
                    self.task_status = "RUNNING" if cmd == "START" else cmd
        except Exception as e:
            self.get_logger().warn(f"Firebase poll error: {e}")
    

    
    def input_loop(self):
        """사용자 입력 처리 루프 (이벤트 발생 시에만 로그 출력)"""
        import termios
        import tty
        import select
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while self.running and rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    ch = sys.stdin.read(1).lower()
                    if ch == 's':
                        self.send_command('START')
                        self.task_status = "RUNNING"
                        self.add_log("[키입력] START 명령")
                    elif ch == 'p':
                        self.send_command('PAUSE')
                        self.task_status = "PAUSED"
                        self.add_log("[키입력] PAUSE 명령")
                    elif ch == 'e':
                        self.send_drl_stop()
                        self.send_command('EMERGENCY_STOP')
                        self.task_status = "STOPPED"
                        self.add_log("[키입력] EMERGENCY_STOP 명령")
                    elif ch == 'h':
                        self.send_command('HOME')
                        self.add_log("[키입력] HOME 명령")
                    elif ch == 'r':
                        self.send_command('RESET')
                        self.add_log("[키입력] RESET 명령")
                    elif ch == 'c':
                        self.send_command('RECOVER') 
                        self.add_log("[키입력] 칩 복구(RECOVER) 명령")
                    elif ch == 'q':
                        self.running = False
                        self.add_log("[키입력] UI 종료(q)")
                        break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main(args=None):
    rclpy.init(args=args)
    
    node = TaskRunnerUI()
    
    # 스핀 쓰레드
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()
    
    try:
        # 시작 배너 출력
        node.print_banner()
        # 메인 쓰레드에서 입력 처리
        node.input_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        print(f"\n{Colors.YELLOW}UI 종료 중...{Colors.ENDC}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
