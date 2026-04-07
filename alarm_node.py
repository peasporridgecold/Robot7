import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.time import Time
from datetime import datetime
# 패키지 경로 확인
from multi_waypoint_nav_pkg.config import alarms

class AlarmNode(Node):
    def __init__(self):
        super().__init__('alarm_node')
        self.pub = self.create_publisher(String, '/goal_sequence', 10)
        # 1초마다 체크하여 정밀도를 높입니다.
        self.timer = self.create_timer(1.0, self.check)
        self.get_logger().info("🔔 알람 노드가 시작되었습니다.")

    def check(self):
        now_dt = datetime.now()
        now_str = now_dt.strftime("%H:%M")

        # 1. 현재 시스템 시간 출력 (로그로 확인 필수!)
        self.get_logger().info(f"현재 시간: {now_str}", once=False)

        for alarm in alarms:
            # 리스트에 든 설정 시간 확인 로그
            # self.get_logger().info(f"비교 중: 설정({alarm.time}) vs 현재({now_str})")

            # 시간 비교 (양쪽 다 문자열로 확실히 변환)
            if str(alarm.time).strip() == now_str and not alarm.done:
                self.get_logger().info(f"🚨 알람 조건 일치! 목적지: {alarm.target}")

                # 네비게이터로 보낼 메시지 생성
                msg = String()
                msg.data = f"{alarm.target}_ALARM"
                self.pub.publish(msg)

                alarm.done = True
                self.get_logger().info(f"✅ {alarm.target}으로 이동 명령 전송 완료")

        for alarm in alarms:
            if alarm.time == now_str and not alarm.done:
                self.get_logger().info(f"⏰ 알람 발생: {alarm.name} -> 목적지: {alarm.target}")

                msg = String()
                msg.data = f"{alarm.target}_ALARM"
                self.pub.publish(msg)

                alarm.done = True
                self.get_logger().info(f"✅ 전송 완료: {msg.data}")

def main():
    rclpy.init()
    node = AlarmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
