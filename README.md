# LoRa-gps-monitor
# 🌍 RPi SensorHub (라즈베리파이 실시간 센서 융합 시스템)

> **Raspberry Pi 5 + GPS + LoRa + LTR390 + BME280 기반 실시간 환경 데이터 수집·시각화 플랫폼**

본 프로젝트는 라즈베리파이에서 다중 센서 데이터를 수집하고, 실내·실외 환경을 판별하며,  
Flask + WebSocket 서버를 통해 실시간으로 웹 대시보드에 시각화하는 IoT 시스템입니다.  

---

## 🧩 시스템 개요

### 구성도
[센서 모듈]
├─ LTR390 (자외선/조도)

├─ BME280 (온도/습도/기압)

├─ GPS (L80-R, CP2102 USB-UART)

└─ LoRa 모듈 (RYLR998 / E22-230T22U 등)

[라즈베리파이5]
├─ websocket.py → 실시간 데이터 수집, GPS/LoRa 통신, WebSocket 송신

├─ flask.py → DB 관리 및 REST API, CSV Export, 웹서버

└─ new.html → 실시간 대시보드 UI

[웹 클라이언트]
└─ 실시간 상태, DB 로그 조회, CSV 다운로드, 라벨링(Indoor/Outdoor)

yaml
코드 복사

---

## ⚙️ 주요 기능

| 구분 | 설명 |
|------|------|
| 🌐 **실시간 WebSocket 송신** | 1초 간격으로 센서 데이터를 웹으로 전송 |

| 📊 **Flask REST API** | DB 조회, 통계, CSV Export 기능 제공 |

| 💾 **SQLite 로컬 DB** | `sensor_log` 테이블에 모든 센서값 자동 저장 |

| 🛰️ **GPS 기반 좌표 수집** | NMEA 파싱으로 유효성(A/V) 및 위도·경도 추출 |

| 📡 **LoRa 통신** | 송수신 데이터 처리 및 거리 계산 (Haversine 공식) |

| 🏠 **실내/실외 판별 알고리즘** | 조도·기압·자외선·GPS fix 상태 기반 환경 분류 |

| 📈 **웹 대시보드 시각화** | 실시간 센서값 표시 + DB 검색/CSV 다운로드 UI |

| ⚡ **LoRa 속도 테스트 모드** | WebSocket 명령(`start_test`)으로 패킷 전송 성능 측정 |

---

## 🗂️ 디렉터리 구조

📁 main/
├── websocket.py # WebSocket + 센서 수집 서버

├── flask.py # Flask REST API 서버

├── web/

│ └── new.html # 대시보드 UI (TailwindCSS)

└── totallogs/

├── sensor.db # SQLite 데이터베이스

└── sensor.log # 실행 로그

yaml
코드 복사

---

## 🧠 실내/실외 판별 알고리즘

센서 데이터와 시간대 정보를 결합하여 점수(`OutdoorScore`)를 계산합니다.

\[
\text{Score} =
\begin{cases}
0.5G + 0.3U + 0.15L + 0.05P, & \text{(주간)} \\
0.7G + 0.2L + 0.1P, & \text{(야간)}
\end{cases}
\]

| 항목 | 설명 |
|------|------|
| G | GPS Fix 상태 (A=1, V=0) |
| U | UV Index (0~1.0 정규화) |
| L | 조도(Lux, 주간/야간 가중치 다름) |
| P | 기압 변동 기반 실내 추정 (ΔP > 1.5 → 실내) |

판별 기준  
- **Score > 0.6 → 실외**  
- **Score < 0.4 → 실내**  
- **그 외 → 전이(출입 중)**

---

## 💻 실행 방법

### 1️⃣ Python 환경 구성

    sudo apt update
    sudo apt install python3-pip python3-venv
    python3 -m venv ltr390env
    source ltr390env/bin/activate
    pip install flask websockets adafruit-circuitpython-ltr390 adafruit-circuitpython-bme280 pyserial

2️⃣ Flask 서버 실행

    python3 flask.py
    
# → http://라즈`베리파이IP:8080 접속
3️⃣ WebSocket 센서 서버 실행

    python3 websocket.py

# → 센서 수집 + 실시간 송신 시작
📊 웹 대시보드 주요 화면
실시간 센서 모니터링 (GPS, UV, Lux, Pressure)

LoRa 상태 및 거리 표시

DB 조회 및 CSV 내보내기

실내/실외 라벨링 버튼 (Indoor / Outdoor / SemiOutdoor)

최고속도(LoRa 송신 속도) 측정 기능

📁 주요 API
Method	Endpoint	설명
GET	/api/data?start=&end=	시간 구간별 데이터 조회
GET	/api/summary	센서 데이터 통계 요약
GET	/api/export_csv	CSV 다운로드
POST	/api/label	현재 센서데이터에 라벨 저장
GET	/api/label_data?label=	특정 라벨 데이터 조회

🧪 데이터 예시
timestamp	gps_fix	uv	lux	pressure	state	score
2025-10-22 13:11:42	A	0.32	558	1019.78	실외	0.742
2025-10-22 13:12:43	V	0.02	430	1019.99	실내	0.181

📎 향후 계획
 TensorFlow Lite 기반 실내/외 자동 학습 모델 적용

 LoRa 양방향 신호 품질 분석 (RSSI/SNR)

 대시보드 다중 클라이언트 지원 및 시각화 개선

 MQTT / InfluxDB / Grafana 연동

🧑‍💻 개발환경
항목	내용
보드	Raspberry Pi 5
언어	Python 3.11
웹프레임워크	Flask, WebSocket
데이터베이스	SQLite3
프론트엔드	TailwindCSS + JS (Vanilla)
센서	LTR390 / BME280 / GPS(L80-R) / LoRa(E22-230T22U)

📜 라이선스
본 프로젝트는 MIT License 하에 배포됩니다.
