from gtts import gTTS
import os
import subprocess

def play_voice(text):
    try:
        filename = "/tmp/tts.mp3"
        tts = gTTS(text=text, lang='ko')
        tts.save(filename)

        # 소리 재생을 백그라운드에서 실행 (코드 멈춤 방지)
        subprocess.Popen(["mpg123", filename])
    except Exception as e:
        print(f"TTS 오류: {e}")
