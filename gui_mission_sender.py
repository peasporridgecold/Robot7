import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image  # 이미지 메시지 타입 추가
from cv_bridge import CvBridge     # OpenCV-ROS 변환 도구 추가
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel, QListWidget, QListWidgetItem, QHBoxLayout
from PyQt5.QtGui import QImage, QPixmap
import cv2
from threading import Thread

# 사용자 패키지 경로에 맞춰 import 확인
try:
    from multi_waypoint_nav_pkg.ocr_engine import OcrEngine
except ImportError:
    # 경로가 다를 경우를 대비한 예외 처리
    print("OcrEngine을 로드할 수 없습니다. 패키지 경로를 확인하세요.")

class MissionGUI(Node, QWidget):
    def __init__(self):
        # ROS 2 노드와 PyQt5 위젯 초기화
        Node.__init__(self, 'mission_gui')
        QWidget.__init__(self)

        # 1. ROS 2 통신 설정
        self.publisher = self.create_publisher(String, '/goal_sequence', 10)
        # 제스처 노드가 구독할 수 있도록 이미지를 발행하는 Publisher 추가
        self.image_pub = self.create_publisher(Image, '/image_raw', 10)
        self.bridge = CvBridge()

        # 2. GUI 설정
        self.setWindowTitle("Robot Mission Control Center")
        self.setGeometry(200, 200, 600, 700)
        self.selected_waypoints = []
        self.init_ui()

        # 3. OCR 엔진 시작 (카메라 프레임을 update_frame으로 전달)
        self.get_logger().info("📷 카메라 및 OCR 엔진을 시작합니다...")
        self.ocr_engine = OcrEngine(self.update_frame)
        self.ocr_engine.start()

    def init_ui(self):
        layout = QVBoxLayout()

        # 카메라 화면 표시 레이블
        self.camera_label = QLabel("카메라 로딩 중...")
        self.camera_label.setFixedSize(580, 400)
        self.camera_label.setStyleSheet("background-color: black; color: white; border: 2px solid gray;")
        self.camera_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.camera_label)

        # 선택된 리스트 표시
        self.label = QLabel("📌 전송 대기 중인 목적지 순서:")
        layout.addWidget(self.label)

        self.list_widget = QListWidget()
        layout.addWidget(self.list_widget)

        # 목적지 버튼들
        button_layout = QHBoxLayout()
        destinations = ["영업팀", "개발팀", "기획팀", "인사팀", "마케팅팀", "회계팀", "희정님"]
        for name in destinations:
            btn = QPushButton(name)
            btn.clicked.connect(lambda checked, n=name: self.add_waypoint(n))
            button_layout.addWidget(btn)
        layout.addLayout(button_layout)

        # 제어 버튼들
        self.start_btn = QPushButton("🚀 미션 시작 (Start Mission)")
        self.start_btn.setMinimumHeight(50)
        self.start_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        self.start_btn.clicked.connect(self.send_mission)
        layout.addWidget(self.start_btn)

        self.clear_btn = QPushButton("🧹 초기화 (Clear)")
        self.clear_btn.clicked.connect(self.clear_list)
        layout.addWidget(self.clear_btn)

        self.setLayout(layout)

    def update_frame(self, frame, target):
        """OcrEngine에서 카메라 프레임을 읽을 때마다 호출됨"""
        if frame is not None:
            # A. 다른 노드(제스처 등)를 위해 ROS 2 토픽으로 이미지 발행
            try:
                # 성능을 위해 크기를 조금 줄여서 전송할 수도 있습니다.
                # frame_resized = cv2.resize(frame, (640, 480))
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.image_pub.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f"이미지 토픽 발행 실패: {e}")

            # B. GUI 화면(PyQt) 업데이트
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
            self.camera_label.setPixmap(QPixmap.fromImage(qt_image).scaled(
                self.camera_label.width(), self.camera_label.height(), aspectRatioMode=1))

        # OCR 결과가 있을 경우 리스트에 추가
        if target:
            self.receive_ocr_target(target)

    def receive_ocr_target(self, target):
        """OCR로 인식된 텍스트를 리스트에 자동 추가"""
        if target not in self.selected_waypoints:
            self.get_logger().info(f"🔍 OCR 인식됨: {target}")
            self.add_waypoint(target)

    def add_waypoint(self, name):
        self.selected_waypoints.append(name)
        self.list_widget.addItem(QListWidgetItem(name))

    def send_mission(self):
        """목적지 리스트를 콤마로 구분하여 네비게이터로 전송"""
        if not self.selected_waypoints:
            self.get_logger().warn("목적지가 선택되지 않았습니다.")
            return

        msg = String()
        msg.data = ",".join(self.selected_waypoints)
        self.publisher.publish(msg)
        self.get_logger().info(f"🛰️ 미션 데이터 전송 완료: {msg.data}")

    def clear_list(self):
        self.selected_waypoints = []
        self.list_widget.clear()

def main():
    rclpy.init()
    app = QApplication(sys.argv)

    gui = MissionGUI()
    gui.show()

    # ROS 2 스핀을 별도 스레드에서 실행 (GUI 응답성 유지)
    ros_thread = Thread(target=lambda: rclpy.spin(gui), daemon=True)
    ros_thread.start()

    try:
        sys.exit(app.exec_())
    except SystemExit:
        gui.get_logger().info("GUI 종료 중...")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
