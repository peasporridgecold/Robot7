import sys
import rclpy
from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtCore import QFile
from turtle_move_ui import Ui_Form
from move_turtle_pub import Move_turtle
from geometry_msgs.msg import Twist
from PySide6.QtCore import QThread, Signal, Slot
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
import rclpy
from rclpy.node import Node


class RclpyThread(QThread):
    def __init__(self, executor):
        super().__init__()
        self.executor = executor

    def run(self):
        try:
            self.executor.spin()
        finally:
            rclpy.shutdown()


class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.ui = Ui_Form()
        # setupUi 함수를 호출해 MainWindow에 있는 위젯을 배치한다.
        self.ui.setupUi(self)
        # button clicked 이벤트 핸들러로 button_clicked 함수와 연결한다.
        self.ui.btn_go.clicked.connect(self.btn_go_clicked)
        self.ui.btn_back.clicked.connect(self.btn_back_clicked)
        self.ui.btn_right.clicked.connect(self.btn_right_clicked)
        self.ui.btn_left.clicked.connect(self.btn_left_clicked)
        self.ui.btn_stop.clicked.connect(self.btn_stop_clicked)

				#ros 실행
        rclpy.init()
        #쓰레드 선언
        self.executor = MultiThreadedExecutor()
        self.rclpy_thread = RclpyThread(self.executor)
        #기존의 cmd_vel 발행하는 클래스 사용
        self.pub_move = Move_turtle()

        #현재 클래스에서 timer 함수 재정의 (gui에 출력하기 위해서)
        self.pub_move.timer = self.pub_move.create_timer(1, self.turtle_move)

        #버튼을 눌렀을 때 속도 변화해주기 위해 인스턴스 변수 선언
        self.velocity = 0.0
        self.angular = 0.0
        self.rclpy_thread.start()

        #만들어진 쓰레드에 노드 publish 노드 추가
        self.executor.add_node(self.pub_move)

		#timer에서 반복적으로 실행할 함수
    def turtle_move(self):
        msg = Twist()
        msg.linear.x = self.velocity
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.angular
        #cmd_vel 정의후 발행
        self.pub_move.move_turtle.publish(msg)
        self.pub_move.get_logger().info(f'Published mesage: {msg.linear}, {msg.angular}')
        #list에 로그 추가
        self.ui.listWidget.addItem(f'{msg.linear._x}, {msg.angular._z}')

    def btn_stop_clicked(self):
        self.velocity = 0.0
        self.angular = 0.0

    def btn_go_clicked(self):
        self.velocity += 0.2

    def btn_back_clicked(self):
        self.velocity -= 0.2

    def btn_right_clicked(self):
        self.angular -= 0.2

    def btn_left_clicked(self):
        self.angular += 0.2


    def ros_executer(self):
        self.executor.spin()

    def closeEvent(self, event):
        # 종료 시 리소스 정리
        self.executor.shutdown()
        self.rclpy_thread.quit()
        self.rclpy_thread.wait()
        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)

    window = MainWindow()
    window.show()

    sys.exit(app.exec())
