import time
import board
import busio
import serial
import adafruit_ltr390
from adafruit_bme280 import basic as adafruit_bme280
import datetime
import sys, os, statistics, asyncio, json
import websockets
import sqlite3
from math import radians, sin, cos, sqrt, atan2

DB_PATH = "/home/fboe/main/totallogs/sensor.db"
LOG_PATH = "/home/fboe/main/totallogs"

# ====== 초기화 ======
conn = sqlite3.connect(DB_PATH)
conn.execute("""
CREATE TABLE IF NOT EXISTS sensor_log (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp TEXT,
    gps TEXT,
    uv REAL,
    lux REAL,
    temperature REAL,
    humidity REAL,
    pressure REAL,
    altitude REAL,
    label TEXT DEFAULT NULL   --
)
""")
conn.commit()
conn.close()

# =========================================================
# 로그 파일 설정
# =========================================================
log_dir = "/home/fboe/main/totallogs"
os.makedirs(log_dir, exist_ok=True)
log_filename = os.path.join(log_dir, "sensor.log")

class DualLogger:
    def __init__(self, file):
        self.terminal = sys.stdout
        self.log = open(file, "a", buffering=1)
    def write(self, message):
        self.terminal.write(message)
        self.log.write(message)
    def flush(self):
        self.terminal.flush()
        self.log.flush()

sys.stdout = DualLogger(log_filename)
print(f"=== Sensor Fusion Start === (log → {log_filename})")

# =========================================================
# I2C 센서 초기화
# =========================================================
i2c = board.I2C()
ltr = adafruit_ltr390.LTR390(i2c)
bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=0x76)
bme280.sea_level_pressure = 1013.25

# =========================================================
# GPS (UART)
# =========================================================
GPS_PORT = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"

GPS_BAUD = 9600
gps = serial.Serial(GPS_PORT, GPS_BAUD, timeout=2)
gps.reset_input_buffer()

gps.write(b"$PMTK220,1000*1F\r\n")  # 1Hz (1000ms)
gps.write(b"$PMTK300,1000,0,0,0,0*1C\r\n")


def get_gps_valid():
    """
    GPS의 $GPRMC/$GNRMC/$GPGGA/$GNGGA 문장에서
    유효 여부(A/V)와 위도(lat), 경도(lon)를 추출한다.
    (비차단형, 안정화 버전)
    """
    gps.timeout = 0.3
    lat = lon = None
    valid = "V"

    for _ in range(5):
        try:
            line = gps.readline().decode(errors="ignore").strip()
            if not line:
                continue

            # ✅ 디버그용
            # print(f"[DEBUG GPS] {line}")

            # -----------------------------
            # RMC 계열 (GPS 주요 문장)
            # -----------------------------
            if line.startswith("$GPRMC") or line.startswith("$GNRMC"):
                parts = line.split(",")
                if len(parts) > 6:
                    valid = parts[2]
                    if valid == "A":  # 신호가 유효할 때만 좌표 파싱
                        lat_raw = float(parts[3])
                        lon_raw = float(parts[5])
                        lat_deg = int(lat_raw / 100)
                        lon_deg = int(lon_raw / 100)
                        lat = lat_deg + (lat_raw - lat_deg * 100) / 60
                        lon = lon_deg + (lon_raw - lon_deg * 100) / 60
                        if parts[4] == "S":
                            lat = -lat
                        if parts[6] == "W":
                            lon = -lon
                        return valid, lat, lon

            # -----------------------------
            # GGA 계열 (보조 문장, 저위에꺼 잘들어오면 나중에지워도되긴하는데 고도쓸일잇으면 냅두고)
            # -----------------------------
            elif line.startswith("$GPGGA") or line.startswith("$GNGGA"):
                parts = line.split(",")
                if len(parts) > 6:
                    fix_quality = parts[6]
                    valid = "A" if fix_quality and fix_quality.isdigit() and int(fix_quality) > 0 else "V"
                    if valid == "A" and len(parts) > 5 and parts[2] and parts[4]:
                        lat_raw = float(parts[2])
                        lon_raw = float(parts[4])
                        lat_deg = int(lat_raw / 100)
                        lon_deg = int(lon_raw / 100)
                        lat = lat_deg + (lat_raw - lat_deg * 100) / 60
                        lon = lon_deg + (lon_raw - lon_deg * 100) / 60
                        if parts[3] == "S":
                            lat = -lat
                        if parts[5] == "W":
                            lon = -lon
                        return valid, lat, lon

        except Exception as e:
            print(f"[GPS ERROR] {e}")
            continue

    # 5번 시도해도 데이터 없으면
    return valid, lat, lon


# =========================================================
# LoRa (UART)
# =========================================================
try:
    LORA_PORT = "/dev/ttyAMA1" #아니면 바꿔야함
    LORA_BAUD = 115200
    lora = serial.Serial(LORA_PORT, LORA_BAUD, timeout=1)
    lora_connected = True
    print("[LORA] 연결 성공")
except Exception as e:
    print(f"[LORA] 연결 실패: {e}")
    lora_connected = False


def send_lora_message(msg: str):
    """LoRa로 문자열 송신 (에러 무시)"""
    if not lora_connected:
        return
    try:
        lora.write((msg + "\r\n").encode())
    except Exception as e:
        print(f"[LORA SEND ERROR] {e}")


def read_lora_message():
    """LoRa 수신 메시지 읽기 (없으면 None)"""
    if not lora_connected:
        return None
    try:
        line = lora.readline().decode(errors="ignore").strip()
        if line:
            return line
    except Exception as e:
        print(f"[LORA RECV ERROR] {e}")
    return None

# =========================================================
# 거리 계산 (Haversine)
# =========================================================
def calc_distance(lat1, lon1, lat2, lon2):
    """두 GPS 좌표(lat/lon) 사이의 거리(m)를 계산"""
    if None in (lat1, lon1, lat2, lon2):
        return None
    R = 6371000.0  # 지구 반지름 (m)
    dlat = radians(lat2 - lat1)
    dlon = radians(lon2 - lon1)
    a = sin(dlat / 2) ** 2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon / 2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    return R * c


# =========================================================
# 실내/외 판별
# =========================================================
pressure_history = []

def get_outdoor_score(gps_fix, uv, lux, pressure):
    global pressure_history

    G = 1.0 if gps_fix == 'A' else 0.0
    U = min(uv / 0.3, 1.0)
    hour = datetime.datetime.now().hour
    if 6 <= hour <= 18:
        L = min(lux / 5000.0, 1.0)
    else:
        L = 1.0 if lux < 50 else 0.0

    pressure_history.append(pressure)
    if len(pressure_history) > 30:
        pressure_history.pop(0)
    avg_pres = statistics.mean(pressure_history)
    diff = pressure - avg_pres

    if diff > 1.5:
        P = 0.0
    elif diff < -1.0:
        P = 1.0
    else:
        P = 0.5

    if 6 <= hour <= 18:
        return 0.5*G + 0.3*U + 0.15*L + 0.05*P
    else:
        return 0.7*G + 0.2*L + 0.1*P

def classify_environment(score):
    if score > 0.6:
        return "실외"
    elif score < 0.4:
        return "실내"
    else:
        return "전이(출입 중)"
# =========================================================
# DB 저장 함수 
# =========================================================
def save_to_db(timestamp, gps_fix, uv, lux, temperature, humidity, pressure, altitude, state, score):
    try:
        conn = sqlite3.connect(DB_PATH)
        conn.execute("""
            INSERT INTO sensor_log
            (timestamp, gps, gps_fix, uv, lux, temperature, humidity, pressure, altitude, state, score)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """, (timestamp, gps_fix, gps_fix, uv, lux, temperature, humidity, pressure, altitude, state, score))
        conn.commit()
        conn.close()
    except Exception as e:
        print(f"[DB ERROR] {e}")

# =========================================================
# LoRa 단방향 속도 테스트 (WebSocket 명령으로 실행)
# =========================================================
is_testing = False   # 최고속도 테스트 중 여부
def lora_speed_test_send(packet_size=64, count=200, interval=0.05):
    global is_testing

    if not lora_connected:
        print("[LORA TEST] ❌ LoRa 미연결 상태입니다.")
        return {"error": "lora not connected"}

    if is_testing:
        print("[LORA TEST] 이미 테스트 중입니다.")
        return {"error": "already testing"}

    is_testing = True  # 🚫 LoRa 점유 시작
    import csv
    from datetime import datetime

    log_path = "/home/fboe/main/totallogs/lora_speed_log.csv"
    with open(log_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["seq", "send_time", "elapsed_ms"])

    payload = "X" * packet_size
    print(f"[LORA TEST] LoRa 단방향 전송 시작 ({packet_size}B × {count}개, 간격 {interval}s)")
    start_time = time.time()

    for i in range(count):
        t0 = time.time()
        msg = f"{payload}{i:04d}\r\n"
        try:
            lora.write(msg.encode())
        except Exception as e:
            print(f"[LORA SEND ERROR] {e}")
            break

        elapsed_ms = (time.time() - t0) * 1000
        now = datetime.now().strftime("%H:%M:%S.%f")[:-3]

        with open(log_path, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([i, now, f"{elapsed_ms:.3f}"])

        print(f"[{i+1}/{count}] 전송완료 ({elapsed_ms:.2f} ms)")
        time.sleep(interval)

    total_elapsed = time.time() - start_time
    total_bytes = count * packet_size
    bps = total_bytes / total_elapsed
    kbps = bps * 8 / 1000.0

    print("\n========== LoRa 단방향 테스트 결과 ==========")
    print(f"총 패킷 수 : {count}")
    print(f"총 데이터량 : {total_bytes} Byte")
    print(f"총 소요시간 : {total_elapsed:.2f} s")
    print(f"평균 속도   : {bps:.1f} B/s ({kbps:.2f} kbps)")
    print(f"로그 저장   : {log_path}")

    is_testing = False  # ✅ 테스트 종료 후 다시 해제

    return {
        "packets": count,
        "bytes": total_bytes,
        "time": round(total_elapsed, 2),
        "bps": round(bps, 1),
        "kbps": round(kbps, 2),
        "log_path": log_path
    }


# =========================================================
# WebSocket 서버
# =========================================================
connected_clients = set()
async def handler(websocket):
    connected_clients.add(websocket)
    print(f"[WS] 클라이언트 접속 ({len(connected_clients)}명)")
    try:
        async for message in websocket:
            # 클라이언트에서 "start_test" 명령이 오면 테스트 실행
            if message == "start_test":
                print("[WS] 클라이언트 요청 → LoRa 단방향 속도 테스트 실행")
                result = lora_speed_test_send(packet_size=64, count=200, interval=0.05)
                await websocket.send(json.dumps({"test_done": True, "lora_result": result}))
    finally:
        connected_clients.remove(websocket)
        print(f"[WS] 클라이언트 종료 ({len(connected_clients)}명 남음)")

async def broadcast_data():
    global is_testing
    while True:
        try:
            if is_testing:
                await asyncio.sleep(1)
                continue
            gps_fix = get_gps_valid()
            uv = ltr.uvi
            lux = ltr.light
            temperature = bme280.temperature
            humidity = bme280.humidity
            pressure = bme280.pressure
            altitude = bme280.altitude

            score = get_outdoor_score(gps_fix, uv, lux, pressure)
            state = classify_environment(score)
            now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")


            # ✅ LoRa에서 수신도 확인 (수신된 메시지 있을 경우)
            incoming = read_lora_message()
            lora_distance = None  # 기본값
            
            if incoming:
                print(f"[LORA RX] {incoming}")
                try:
                    parts = incoming.split(",")
                    # 예상 포맷: "2025-10-22 12:30:12,37.523514,127.029821"
                    if len(parts) >= 3:
                        ts, lat_str, lon_str = parts[:3]
                        lat_remote = float(lat_str)
                        lon_remote = float(lon_str)

                        # ✅ 내 GPS 좌표
                        valid, my_lat, my_lon = get_gps_valid()
                        if valid == "A" and my_lat and my_lon:
                            lora_distance = calc_distance(my_lat, my_lon, lat_remote, lon_remote)
                            print(f"[LORA DIST] {lora_distance:.1f} m")
                            print(f"[LORA DIST] {lora_distance:.1f} m")

                            # ✅ 계산된 거리값을 LoRa로 다시 송신 (상대에게 응답)
                            msg = f"{now},{lora_distance:.1f}"
                            send_lora_message(msg)
                            print(f"[LORA TX → 송신] {msg}")

                except Exception as e:
                    print(f"[LORA PARSE ERROR] {e}")


            print(
                f"[{now}] {state} | Score:{score:.3f} | GPS:{gps_fix} | "
                f"조도:{lux:8.0f} | UVI:{uv:4.2f} | "
                f"온도:{temperature:5.2f}°C | 습도:{humidity:5.2f}% | "
                f"기압:{pressure:7.2f}hPa | 고도:{altitude:7.2f}m"
            )

            payload = {
                "timestamp": now,
                "state": state,
                "score": round(score, 3),
                "gps_fix": gps_fix,
                "lux": round(lux, 2),
                "uv": round(uv, 2),
                "temperature": round(temperature, 2),
                "humidity": round(humidity, 2),
                "pressure": round(pressure, 2),
                "altitude": round(altitude, 2),
                "lora": "연결됨" if lora_connected else "연결안됨",
                "lora_rx": incoming or "",
                "lora_dist": round(lora_distance, 1) if lora_distance else None
            }

            save_to_db(now, gps_fix, uv, lux, temperature, humidity, pressure, altitude, state, score)


            if connected_clients:
                msg = json.dumps(payload)
                await asyncio.gather(*[c.send(msg) for c in connected_clients])

            await asyncio.sleep(1)


        except Exception as e:
            print(f"[ERROR] {e}")
            await asyncio.sleep(1)

async def main():
    print("[WS] WebSocket 서버 시작 (포트 8765)")
    async with websockets.serve(handler, "0.0.0.0", 8765):
        await broadcast_data()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[STOP] User interrupt")
