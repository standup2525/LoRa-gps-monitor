import asyncio
import websockets
import threading
import serial
import base64
import time
import uuid
import os
from datetime import datetime
import json

SERIAL_PORT = "COM3"
BAUD = 9600

WS_PORT = 8899
connected_clients = set()

# -----------------------
#   전역 세션 변수
# -----------------------
current_marker_id = None               # 영상 세션 ID (UUID)
current_capture_time = None            # 영상 캡처 시작 시간
current_video_total_bytes = 0          # 전체 영상 바이트 수 (송신측 META에서 옴)
current_video_sent_bytes = 0           # 현재까지 누적된 바이트 수

frames = {}                            # { frame_id: {"chunk_total":N, "chunks":{idx:payload}} }
frame_order = []                       # 조립 순서를 유지하는 프레임 ID 리스트

thumbnail_path = None                  # 이 영상의 썸네일 파일 경로
thumbnail_created = False              # 썸네일 생성 여부
lowest_frame_id_received = None        # 가장 작은 frame_id 기록 → 썸네일 결정용

video_dir = None                       # 영상 파일 저장 디렉토리



# ---------------------------
#   WebSocket Broadcast
# ---------------------------
async def ws_broadcast(data: dict):
    if not connected_clients:
        return
    text = json.dumps(data, ensure_ascii=False)
    await asyncio.gather(*(c.send(text) for c in connected_clients))


async def ws_handler(ws):
    connected_clients.add(ws)
    print("[WS] Client connected")
    try:
        async for _ in ws:
            pass
    except:
        pass
    finally:
        connected_clients.remove(ws)
        print("[WS] Client disconnected")


def start_ws():
    async def ws_main():
        async with websockets.serve(ws_handler, "0.0.0.0", WS_PORT):
            print(f"[WS] WebSocket running at ws://0.0.0.0:{WS_PORT}")
            await asyncio.Future()
    asyncio.run(ws_main())


# ---------------------------
#   프레임 완성 → JPEG 저장
# ---------------------------
def finalize_frame(frame_id):
    global frames, thumbnail_path, thumbnail_created
    global lowest_frame_id_received, video_dir

    state = frames.get(frame_id)
    if not state:
        return None

    chunk_total = state["chunk_total"]
    if len(state["chunks"]) < chunk_total:
        return None  # 아직 미완성

    # Base64 조립
    b64_all = "".join(state["chunks"][i] for i in range(chunk_total))
    try:
        img_bytes = base64.b64decode(b64_all)
    except:
        print("[DECODE ERROR]", frame_id)
        return None

    # 프레임 저장
    frame_filename = f"{frame_id:04}.jpg"
    frame_path = os.path.join(video_dir, frame_filename)

    with open(frame_path, "wb") as f:
        f.write(img_bytes)

    print(f"[FRAME] saved → {frame_path}")

    # ------------------------------
    #   썸네일 선택 로직 (중요)
    #   → 가장 작은 frame_id를 썸네일로 사용
    # ------------------------------
    if lowest_frame_id_received is None or frame_id < lowest_frame_id_received:
        lowest_frame_id_received = frame_id

        # 썸네일 덮어쓰기 (영상 단위 폴더 내부)
        thumb_path = os.path.join(video_dir, "thumbnail.jpg")
        with open(thumb_path, "wb") as t:
            t.write(img_bytes)

        thumbnail_path = thumb_path
        thumbnail_created = True

        print(f"[THUMBNAIL] updated → frame {frame_id}")

    return frame_path


# ---------------------------
#   New Video Session Reset
# ---------------------------
def reset_video_session(video_total_bytes):
    """
    새로운 영상 세션이 시작될 때 호출됨.
    영상폴더 생성 + 모든 변수 초기화.
    """
    global current_marker_id, current_capture_time
    global current_video_total_bytes, current_video_sent_bytes
    global frames, frame_order, thumbnail_created, thumbnail_path
    global lowest_frame_id_received, video_dir

    # 새로운 영상 세션 ID (UUID)
    current_marker_id = str(uuid.uuid4())
    current_capture_time = datetime.now().isoformat()

    # byte counter
    current_video_total_bytes = video_total_bytes
    current_video_sent_bytes = 0

    # 프레임 관련 구조 초기화
    frames = {}
    frame_order = []
    thumbnail_path = None
    thumbnail_created = False
    lowest_frame_id_received = None

    # 영상 전용 폴더 생성
    video_dir = os.path.join("videos", current_marker_id)
    os.makedirs(video_dir, exist_ok=True)

    print(f"[VIDEO RESET] new video session → {current_marker_id}")

def build_video(video_dir, marker_id):
    """
    저장된 JPG 프레임을 사용해 mp4 영상 생성
    """
    # 프레임 목록 파일 생성
    list_path = os.path.join(video_dir, "frames.txt")
    with open(list_path, "w", encoding="utf-8") as f:
        # jpg만 정렬해서 영상 순서대로 넣기
        frames = sorted([x for x in os.listdir(video_dir) if x.endswith(".jpg") and x != "thumbnail.jpg"])
        for fr in frames:
            f.write(f"file '{os.path.join(video_dir, fr)}'\n")

    # 출력 mp4 경로
    out_path = os.path.join(video_dir, f"{marker_id}.mp4")

    # ffmpeg 실행
    cmd = f"ffmpeg -y -f concat -safe 0 -i {list_path} -vcodec libx264 -pix_fmt yuv420p {out_path}"
    print("[FFMPEG]", cmd)
    os.system(cmd)

    print("[VIDEO CREATED]", out_path)
    return out_path

# ---------------------------
#   LoRa 수신 스레드
# ---------------------------
def lora_thread():
    global frames
    global current_marker_id, current_capture_time
    global current_video_sent_bytes, current_video_total_bytes

    ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.1)
    print("[LoRa] Receiver started")

    last_video_total_bytes = None

    while True:
        try:
            raw = ser.readline().decode(errors="ignore").strip()
            if not raw:
                continue

            print("[RAW]", raw)

            # ---------------------------
            #   META 패킷 처리
            # ---------------------------
            if ",M|" in raw:
                try:
                    # META 구문 파싱
                    body = raw.split(",M|", 1)[1]
                    meta_part, _ = body.split(",", 1)
                    parts = meta_part.split("|")

                    frame_id = int(parts[0])
                    env_code = int(parts[1])
                    lat = float(parts[2])
                    lon = float(parts[3])
                    img_size = int(parts[4])
                    video_total_bytes = int(parts[5])

                    # 새로운 영상인지 판별:
                    # ① frame_id == 0 이거나
                    # ② video_total_bytes가 이전 영상과 다르면 새 영상
                    if frame_id == 0 or video_total_bytes != last_video_total_bytes:
                        reset_video_session(video_total_bytes)
                        last_video_total_bytes = video_total_bytes
                        if video_dir is not None and current_marker_id is not None:
                            build_video(video_dir, current_marker_id)

                    print(f"[META] frame={frame_id}, total={video_total_bytes}")

                except Exception as e:
                    print("[META ERROR]", e)

                continue

            # ---------------------------
            #   DATA (chunk) 패킷 처리
            # ---------------------------
            if ",D|" in raw:
                try:
                    body = raw.split(",D|", 1)[1]
                    data_part, _ = body.split(",", 1)

                    f_id, idx, total, b64_payload = data_part.split("|")

                    frame_id = int(f_id)
                    idx = int(idx)
                    total = int(total)

                    # 프레임 구조 없으면 생성
                    if frame_id not in frames:
                        frames[frame_id] = {"chunk_total": total, "chunks": {}}
                        frame_order.append(frame_id)

                    frames[frame_id]["chunks"][idx] = b64_payload

                    # 바이트 누적
                    current_video_sent_bytes += len(b64_payload)

                    # WS 브로드캐스트 (진행률)
                    asyncio.run(ws_broadcast({
                        "type": "senser",
                        "data": [
                            {
                                "serialNumber": "지게차(A12)",
                                "marker": [
                                    {
                                        "id": current_marker_id,
                                        "lat": lat,
                                        "lng": lon, 
                                        "environment": "실내",#수정
                                        "video": {
                                            "thumbnail": f"/videos/{current_marker_id}/thumbnail.jpg" if thumbnail_created else "",
                                            "video": ""  
                                        },
                                        "progress": {
                                            "sentBytes": current_video_sent_bytes,
                                            "totalBytes": current_video_total_bytes*2
                                        },
                                        "capturedAt": current_capture_time
                                    }
                                ]
                            }
                        ]
                    }))

                    # 프레임 완성되면 즉시 저장 + 썸네일 업데이트
                    if len(frames[frame_id]["chunks"]) == total:
                        finalize_frame(frame_id)

                except Exception as e:
                    print("[DATA ERROR]", e)

                continue

        except Exception as e:
            print("[SERIAL ERROR]", e)


# ---------------------------
#   Main
# ---------------------------
if __name__ == "__main__":
    os.makedirs("videos", exist_ok=True)

    threading.Thread(target=start_ws, daemon=True).start()
    lora_thread()
