import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from datetime import datetime  # 시간 출력

from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtCore import QThread, Slot

from turtle_move_ui import Ui_Form
from move_turtle_pub import Move_turtle

class RclpyThread(QThread):
    def __init__(self, executor):
        super().__init__()
        self.executor = executor

    def run(self):
        self.executor.spin()

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_Form()
        self.ui.setupUi(self)

        # ROS2 초기화
        if not rclpy.ok():
            rclpy.init()

        self.executor = MultiThreadedExecutor()
        self.pub_move = Move_turtle()
        self.executor.add_node(self.pub_move)

        self.rclpy_thread = RclpyThread(self.executor)
        self.rclpy_thread.start()

        self.velocity = 0.0
        self.angular = 0.0
        self.is_paused = False

        self.pub_move.create_timer(0.1, self.turtle_move)

        # Gazebo 서비스 클라이언트
        self.reset_client = self.pub_move.create_client(Empty, '/reset_world')
        self.pause_client = self.pub_move.create_client(Empty, '/pause_physics')
        self.unpause_client = self.pub_move.create_client(Empty, '/unpause_physics')

        # 버튼 이벤트 연결
        self.ui.btn_go.clicked.connect(self.btn_go_clicked)
        self.ui.btn_back.clicked.connect(self.btn_back_clicked)
        self.ui.btn_right.clicked.connect(self.btn_right_clicked)
        self.ui.btn_left.clicked.connect(self.btn_left_clicked)
        self.ui.btn_stop.clicked.connect(self.btn_stop_clicked)
        self.ui.btn_reset.clicked.connect(self.btn_reset_clicked)
        self.ui.btn_pause.clicked.connect(self.btn_pause_clicked)

        # 시작 메시지 출력
        self.add_log("시스템이 시작되었습니다.")

    # ==============================
    # 로그 출력 함수 (ListWidget)
    # ==============================
    def add_log(self, message):
        current_time = datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{current_time}] {message}"

        self.ui.listWidget.addItem(log_entry)
        self.ui.listWidget.scrollToBottom()  # 최신 로그로 자동 스크롤

    # ==============================
    # Topic: cmd_vel publish
    # ==============================
    def turtle_move(self):
        msg = Twist()
        msg.linear.x = self.velocity
        msg.angular.z = self.angular
        self.pub_move.move_turtle.publish(msg)

    # ==============================
    # 제어 버튼 콜백
    # ==============================
    def btn_go_clicked(self):
        self.velocity += 0.2
        self.add_log(f"전진: 현재 선속도 {self.velocity:.1f}")

    def btn_back_clicked(self):
        self.velocity -= 0.2
        self.add_log(f"후진: 현재 선속도 {self.velocity:.1f}")

    def btn_right_clicked(self):
        self.angular -= 0.2
        self.add_log(f"우회전: 현재 각속도 {self.angular:.1f}")

    def btn_left_clicked(self):
        self.angular += 0.2
        self.add_log(f"좌회전: 현재 각속도 {self.angular:.1f}")

    def btn_stop_clicked(self):
        self.velocity = 0.0
        self.angular = 0.0
        self.add_log("정지 버튼 클릭 (속도 초기화)")

    # ==============================
    # 서비스 호출 관련
    # ==============================
    def call_empty_service(self, client, action_name):
        if not client.wait_for_service(timeout_sec=1.0):
            self.add_log(f"오류: {action_name} 서비스를 찾을 수 없습니다.")
            return

        req = Empty.Request()
        future = client.call_async(req)
        # 람다를 이용해 액션 이름을 콜백에 전달
        future.add_done_callback(lambda f: self.service_response_callback(f, action_name))

    def service_response_callback(self, future, action_name):
        try:
            future.result()
            self.add_log(f"성공: {action_name} 완료")
        except Exception as e:
            self.add_log(f"실패: {action_name} 도중 오류 발생")

    def btn_reset_clicked(self):
        self.add_log("월드 초기화 요청 중...")
        self.call_empty_service(self.reset_client, "리셋")

    def btn_pause_clicked(self):
        if not self.is_paused:
            self.call_empty_service(self.pause_client, "일시정지")
            self.ui.btn_pause.setText("Resume")
            self.is_paused = True
        else:
            self.call_empty_service(self.unpause_client, "재개")
            self.ui.btn_pause.setText("Pause")
            self.is_paused = False

    def closeEvent(self, event):
        self.executor.shutdown()
        self.rclpy_thread.quit()
        self.rclpy_thread.wait()
        rclpy.shutdown()
        super().closeEvent(event)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

