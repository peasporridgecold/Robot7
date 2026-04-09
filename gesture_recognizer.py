import cv2
import math
import mediapipe as mp
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

class GestureRecognizer(Node):
    def __init__(self):
        super().__init__('gesture_recognizer')
        self.bridge = CvBridge()
        self.active = False

        # 1. 카메라 토픽 확인: 터틀봇 시뮬레이션은 보통 /camera/image_raw 입니다.
        # 실제 환경에 맞춰 수정하세요.
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        self.gesture_pub = self.create_publisher(String, '/gesture', 10)
        self.control_sub = self.create_subscription(
            String,
            '/gesture_control',
            self.control_callback,
            10
        )

        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        self.ok_count = 0
        self.get_logger().info("✅ Gesture Recognizer Node Ready")

    def control_callback(self, msg):
        if msg.data == "START":
            self.active = True
            self.ok_count = 0
            self.get_logger().info("🚀 제스처 인식 시작 (START 수신)")
        elif msg.data == "STOP":
            self.active = False
            cv2.destroyAllWindows()
            self.get_logger().info("🛑 제스처 인식 중지 (STOP 수신)")

    def image_callback(self, msg):
        # 활성화 상태가 아니면 창을 닫고 리턴
        if not self.active:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        # 성능을 위해 이미지 크기 조정 (선택 사항)
        # frame = cv2.resize(frame, (640, 480))

        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(image_rgb)

        display_text = "Waiting for Hand..."

        if results.multi_hand_landmarks:
            for hl in results.multi_hand_landmarks:
                # 손가락 랜드마크 그리기 (인식 확인용)
                self.mp_drawing.draw_landmarks(frame, hl, self.mp_hands.HAND_CONNECTIONS)

                # 엄지(4)와 검지(8) 거리 측정
                d = math.sqrt((hl.landmark[4].x - hl.landmark[8].x)**2 +
                              (hl.landmark[4].y - hl.landmark[8].y)**2)

                if d < 0.05: # OK 제스처 판정
                    self.ok_count += 1
                    display_text = f"OK Detecting... {self.ok_count}/10"
                else:
                    self.ok_count = 0
                    display_text = "Hand Detected"

                if self.ok_count >= 10: # 10프레임 동안 유지되면 성공
                    self.get_logger().info("✅ OK Gesture Confirmed!")
                    self.gesture_pub.publish(String(data="OK"))
                    self.active = False
                    cv2.destroyAllWindows()
                    return

        # 화면에 상태 텍스트 표시
        cv2.putText(frame, display_text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow("Robot Eye - Gesture Recognition", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = GestureRecognizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
