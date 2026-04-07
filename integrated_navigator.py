import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from multi_waypoint_nav_pkg.waypoints import WAYPOINTS
from multi_waypoint_nav_pkg.player_voice import play_voice

class IntegratedNavigator(Node):
    def __init__(self):
        super().__init__('integrated_navigator')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 목적지 큐 구독
        self.subscription = self.create_subscription(String, '/goal_sequence', self.add_waypoint_callback, 10)
        # 제스처 결과 구독
        self.gesture_sub = self.create_subscription(String, '/gesture', self.gesture_callback, 10)
        # 제스처 노드 제어 (START/STOP)
        self.gesture_control_pub = self.create_publisher(String, '/gesture_control', 10)

        self.goal_queue = []
        self.currently_moving = False
        self.waiting_for_ok = False
        self.current_goal_is_alarm = False

    def add_waypoint_callback(self, msg):
        self.get_logger().info(f"📩 새 목적지 수신: {msg.data}")

        # 쉼표로 구분된 목적지들을 큐에 추가
        for item in msg.data.split(','):
            self.goal_queue.append(item.strip())

        self.get_logger().info(f"📋 현재 대기 큐: {self.goal_queue}")

        # 로봇이 멈춰 있고, 제스처 대기 중도 아닐 때만 이동 시작
        if not self.currently_moving and not self.waiting_for_ok:
            self.get_logger().info("🚀 로봇이 대기 중이므로 즉시 이동을 시작합니다.")
            self.send_next_goal()
        else:
            self.get_logger().info("⏳ 로봇이 현재 작업 중입니다. 작업 완료 후 다음 목적지로 이동합니다.")

    def send_next_goal(self):
        if not self.goal_queue:
            self.get_logger().info("🎉 모든 미션 완료!")
            self.currently_moving = False
            return

        # 이동 시작 전 플래그 고정
        self.currently_moving = True

        raw_name = self.goal_queue.pop(0)
        self.current_goal_is_alarm = "_ALARM" in raw_name
        self.current_goal_name = raw_name.replace("_ALARM", "")

        wp = WAYPOINTS.get(self.current_goal_name)
        if not wp:
            self.get_logger().error(f"❌ 목적지 없음: {self.current_goal_name}")
            self.currently_moving = False # 다시 대기상태로
            self.send_next_goal()
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose(wp)

        self.get_logger().info(f"🚀 이동 시작: {self.current_goal_name} (알람모드: {self.current_goal_is_alarm})")

        # 서버 연결 확인 대기 (중요)
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("❌ Nav2 액션 서버를 찾을 수 없습니다!")
            self.currently_moving = False
            return

        send_goal_future = self.client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ 목표가 거부되었습니다.")
            self.currently_moving = False
            self.send_next_goal()
            return

        self.get_logger().info("✅ 목표가 수락되었습니다. 주행 중...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        status = future.result().status
        self.currently_moving = False

        if status == 4: # 성공적으로 도착
            self.get_logger().info(f"🎯 {self.current_goal_name} 도착 완료!")

            if self.current_goal_is_alarm:
                # 🚀 [추가] 도착 시 TTS 음성 재생
                self.get_logger().info("📢 TTS 재생: 안내 메시지 송출")
                try:
                    # 알람 설정에 맞는 텍스트를 보내거나 고정 메시지 송출
                    play_voice(f"{self.current_goal_name}에 도착했습니다. 수령하시고 오케이 제스처를 해주세요.")
                except Exception as e:
                    self.get_logger().error(f"❌ TTS 재생 실패: {e}")

                self.waiting_for_ok = True
                self.gesture_control_pub.publish(String(data="START"))
                self.get_logger().info("⏳ 알람 모드: 제스처 대기 시작")
            else:
                self.get_logger().info("일반 이동 완료. 다음 큐를 확인합니다.")
                self.send_next_goal()
        else:
            self.get_logger().warn(f"⚠️ 이동 중단/실패 (Status: {status}). 다음 목적지를 시도합니다.")
            self.send_next_goal()

    def gesture_callback(self, msg):
        if self.waiting_for_ok and msg.data == "OK":
            self.get_logger().info("✅ OK Received. Moving Next.")
            self.gesture_control_pub.publish(String(data="STOP"))
            self.waiting_for_ok = False
            self.send_next_goal()

    def create_pose(self, wp):
        pose = PoseStamped()
        pose.header.frame_id, pose.header.stamp = 'map', self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = wp["x"], wp["y"]
        pose.pose.orientation.z = math.sin(wp["theta"] / 2)
        pose.pose.orientation.w = math.cos(wp["theta"] / 2)
        return pose

def main():
    rclpy.init()
    rclpy.spin(IntegratedNavigator())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
