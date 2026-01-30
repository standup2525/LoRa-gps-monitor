import serial
import time
import base64
from datetime import datetime

import cv2
from picamera2 import Picamera2

import asyncio
import websockets
import threading
import json

current_env = 0          # 0=실내, 1=실외, 2=세미아웃도어
current_lat = None
current_lng = None

# ------------------------------------------
# RYLR998 UART 설정 (라파이 쪽)
# ------------------------------------------
ser = serial.Serial(
    port='/dev/ttyAMA1',   # RYLR998 연결된 UART 포트
    baudrate=115200,
    timeout=1.0,
)

DEST_ADDR = 3   # PC 모듈 ADDRESS
CHUNK_B64_LEN = 160  # Base64 조각 길이

# ----- 영상 관련 설정 -----
VIDEO_DURATION_SEC = 3.0   # 3초 영상
VIDEO_FPS = 20             # 1초에 20프레임
FRAMES_PER_BURST = int(VIDEO_DURATION_SEC * VIDEO_FPS)  # 60프레임


# 추가: 첫 조각 보호용 딜레이
META_TO_FIRST_CHUNK_DELAY = 0.2   # META 보낸 후 첫 chunk 전까지 쉬는 시간 (초)


async def sensor_listener():
    global current_env, current_lat, current_lng
    uri = "ws://127.0.0.1:8765"   # websocket.py 서버
    while True:
        try:
            async with websockets.connect(uri) as ws:
                async for msg in ws:
                    data = json.loads(msg)
                    state = data.get("state", "실내")
                    gps_lat = data.get("lat")
                    gps_lng = data.get("lng")

                    # 상태를 숫자로 변환 (LoRa META 전송용)
                    if state == "실내": current_env = 0
                    elif state == "실외": current_env = 1
                    else: current_env = 2     # 전이 = 세미아웃도어

                    current_lat = gps_lat
                    current_lng = gps_lng

        except:
            time.sleep(1)

def start_sensor_thread():
    asyncio.run(sensor_listener())

# ------------------------------------------
# AT 명령 보내기 헬퍼
# ------------------------------------------
def send_at(cmd: str, wait: float = 1.0):
    """
    RYLR998에 AT 명령 한 줄 보내고 응답 읽기.
    로라 듀티 때문에 기본 wait=1초.
    """
    full = (cmd + '\r\n').encode('ascii')
    ser.write(full)
    ser.flush()
    time.sleep(wait)
    resp = ser.read_all()
    if resp:
        print(f"[RESP] {resp.decode(errors='ignore').strip()}")
    else:
        print("[RESP] (no response)")


def send_data_line(dest_addr: int, payload: str):
    """
    AT+SEND=dest,len,payload 한 번 보내고 1초 쉼.
    payload는 ASCII 문자열 (M|... 또는 D|...) 이어야 함.
    """
    length = len(payload)
    cmd = f"AT+SEND={dest_addr},{length},{payload}"
    print(f"[TX] {cmd[:80]}{'...' if len(cmd) > 80 else ''}")
    send_at(cmd, wait=1)  # 패킷 하나 보낼 때마다 1초 대기


# ------------------------------------------
# 카메라 초기화 & 한 장 캡쳐 → 160x90 GRAY JPEG
# ------------------------------------------
def init_camera():
    picam2 = Picamera2()
    config = picam2.create_still_configuration()
    picam2.configure(config)
    picam2.start()
    time.sleep(0.5)
    return picam2


def capture_color_160x90_jpeg(picam2):
    """
    Picamera2에서 컬러(BGR) 프레임을 받아서
    160x90으로 리사이즈한 뒤 컬러 JPEG로 인코딩해서 bytes 리턴.
    """
    # Picamera2에서 BGR 이미지 한 프레임 캡쳐
    frame = picam2.capture_array()  # shape: (H, W, 3), BGR 컬러

    # 160x90으로 리사이즈
    resized = cv2.resize(frame, (160, 90), interpolation=cv2.INTER_AREA)

    # 컬러 그대로 JPEG 인코딩 (품질은 일단 60으로; 필요하면 조절)
    ret, jpeg_buf = cv2.imencode('.jpg', resized, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
    if not ret:
        raise RuntimeError("JPEG 인코딩 실패")

    img_bytes = jpeg_buf.tobytes()
    return img_bytes

def capture_video_burst(picam2, duration_sec=VIDEO_DURATION_SEC, fps=VIDEO_FPS):
    """
    duration_sec 동안 fps로 프레임을 연속 촬영해서
    JPEG bytes 리스트로 리턴.
    
    예) duration_sec=3, fps=20 → 60프레임 리스트 리턴
    """
    frames = []
    frame_interval = 1.0 / fps
    num_frames = int(duration_sec * fps)

    print(f"[INFO] Start burst capture: duration={duration_sec}s, fps={fps}, frames={num_frames}")

    for n in range(num_frames):
        t0 = time.time()

        img_bytes = capture_color_160x90_jpeg(picam2)
        frames.append(img_bytes)

        # 목표 프레임 간격 맞추기
        elapsed = time.time() - t0
        sleep_time = frame_interval - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

    print(f"[INFO] Burst capture done: captured={len(frames)} frames")
    return frames


# ------------------------------------------
# GPS + 환경값 (임시 더미)
# ------------------------------------------
def get_gps_env():
    # TODO: 실제 GPS 모듈에서 읽도록 수정
    # GPS + env 변환 (정수 스케일)
    lat_int = int(current_lat * 1e7) if current_lat else 0
    lon_int = int(current_lng * 1e7) if current_lng else 0
    env = current_env

    return lat_int, lon_int, env


# ------------------------------------------
# 한 프레임 전송 (Meta + Data chunks)
# ------------------------------------------
def send_frame(frame_id: int, img_bytes: bytes, total_video_bytes: int):
    lat_int, lon_int, env = get_gps_env()
    img_size = len(img_bytes)

    # 1. base64 변환
    b64_str = base64.b64encode(img_bytes).decode("ascii")
    total_len = len(b64_str)
    chunk_total = (total_len + CHUNK_B64_LEN - 1) // CHUNK_B64_LEN

    # 2. META 전송 (total_video_bytes 포함)
    meta_payload = f"M|{frame_id}|{env}|{lat_int}|{lon_int}|{img_size}|{total_video_bytes}"
    send_data_line(DEST_ADDR, meta_payload)
    time.sleep(META_TO_FIRST_CHUNK_DELAY)

    # 3. DATA 전송
    for i in range(chunk_total):
        chunk = b64_str[i*CHUNK_B64_LEN : (i+1)*CHUNK_B64_LEN]
        payload = f"D|{frame_id}|{i}|{chunk_total}|{chunk}"
        send_data_line(DEST_ADDR, payload)
        send_data_line(DEST_ADDR, payload)

    print(f"[FRAME SEND] frame={frame_id}, size={img_size}, chunks={chunk_total}")


# ------------------------------------------
# 메인 루프
# ------------------------------------------
def main():
    threading.Thread(target=start_sensor_thread, daemon=True).start()
    picam2 = init_camera()
    frame_id = 0  # LoRa 프로토콜 상의 frame_id (0~255 순환)

    print("[INFO] Start 3s/20fps video burst → LoRa send loop. Ctrl+C to stop.")

    try:
        # 모듈 살아 있는지 테스트
        send_at("AT")

        while True:
            # 1) 3초짜리 20fps 버스트 촬영 (라파이 메모리에 JPEG 리스트로 저장)
            frames = capture_video_burst(picam2, duration_sec=VIDEO_DURATION_SEC, fps=VIDEO_FPS)
            b64_frames = [base64.b64encode(f).decode('ascii') for f in frames]
            total_video_bytes = sum(len(b64) for b64 in b64_frames)
            #otal_video_bytes = sum(len(f) for f in frames)

            # 2) 방금 찍은 프레임들을 LoRa로 차례대로 전송
            print(f"[INFO] Start sending burst: {len(frames)} frames")

            for i, img_bytes in enumerate(frames):
                print(f"\n[INFO] Sending burst frame {i+1}/{len(frames)} (FRAME_ID={frame_id})...")
                send_frame(frame_id, img_bytes, total_video_bytes)
                frame_id = (frame_id + 1) % 60

            print("[INFO] Burst send done. Next burst will start after short pause.")
            # 필요하면 버스트 사이에 쉬는 시간 조금 줄 수 있음
            time.sleep(2.0)

    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")
    finally:
        picam2.stop()
        ser.close()
        print("[INFO] Camera & Serial closed.")


if __name__ == "__main__":
    main()
