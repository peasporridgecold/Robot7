import cv2
import easyocr
import threading
import time

from multi_waypoint_nav_pkg.waypoints import WAYPOINTS


class OcrEngine(threading.Thread):
    def __init__(self, callback_func):
        super().__init__(daemon=True)

        self.cap = cv2.VideoCapture(0)
        self.reader = easyocr.Reader(['ko', 'en'], gpu=False)

        self.callback = callback_func

        self.is_running = False
        self.is_processing = False

        self.last_sent = ""          # 마지막으로 보낸 결과
        self.last_sent_time = 0      # 마지막 전송 시간

        # 🔥 중복 방지 시간 (초)
        self.SEND_INTERVAL = 3.0

    def run(self):
        self.is_running = True
        last_scan_time = 0

        while self.is_running:
            ret, frame = self.cap.read()
            if not ret:
                continue

            display_img = cv2.resize(frame, (640, 480))

            # 🔥 OCR 실행 타이밍 제한 (CPU 보호)
            if not self.is_processing and (time.time() - last_scan_time > 1.5):
                self.is_processing = True
                last_scan_time = time.time()

                threading.Thread(
                    target=self._ocr_task,
                    args=(display_img,),
                    daemon=True
                ).start()

            # 🔥 GUI는 항상 최신 화면만 받음
            self.callback(display_img, "")

            time.sleep(0.03)

    def _ocr_task(self, img):
        try:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            results = self.reader.readtext(gray, detail=0)

            # 🔥 텍스트 정리
            text = "".join(results).replace(" ", "")

            # 🔥 WAYPOINT 기준 매칭 (핵심)
            match = next((k for k in WAYPOINTS.keys() if k in text), "")

            current_time = time.time()

            # 🔥 전송 조건
            if match:
                # 같은 값 연속 전송 방지
                if match != self.last_sent or (current_time - self.last_sent_time > self.SEND_INTERVAL):

                    self.last_sent = match
                    self.last_sent_time = current_time

                    # 🔥 GUI로 목표 전달
                    self.callback(img, match)

        except Exception as e:
            print(f"OCR 오류: {e}")

        finally:
            self.is_processing = False

    def stop(self):
        self.is_running = False
        self.cap.release()
