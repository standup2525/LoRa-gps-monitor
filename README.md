# LoRa GPS Monitor

라즈베리파이 기반 LoRa + GPS + 환경 센서 데이터를 **실시간으로 수집/표시하고**, DB 조회 및 CSV 내보내기를 제공하는 모니터링 대시보드입니다.

## 주요 기능
- **실시간 WebSocket 대시보드**
  - GPS / UV / Lux / Pressure 상태 표시
  - LoRa 상태 및 거리 표시
  - LoRa 속도 테스트 모드 지원 (`start_test`)
- **Flask REST API**
  - 기간 조회, 요약 통계, CSV 다운로드
- **라벨링 기능**
  - Indoor / Outdoor / SemiOutdoor 라벨 저장 및 조회
- **SQLite 로그 저장**
  - `sensor_log` 테이블에 자동 기록

## 구성
- `websocket.py` : 센서 수집 + WebSocket 송신
- `flask.py` : REST API / DB 관리 / CSV Export
- `new.html` : 대시보드 UI
- `sensor.db` : SQLite 데이터베이스

## 실행 방법
### 1) Python 환경 준비
```bash
sudo apt update
sudo apt install python3-pip python3-venv
python3 -m venv ltr390env
source ltr390env/bin/activate
pip install flask websockets adafruit-circuitpython-ltr390 adafruit-circuitpython-bme280 pyserial
```

### 2) Flask 서버 실행
```bash
python3 flask.py
```
웹 브라우저에서 접속
```
http://<RaspberryPi-IP>:8080
```

### 3) WebSocket 센서 서버 실행
```bash
python3 websocket.py
```

## API 요약
| Method | Endpoint | 설명 |
|--------|----------|------|
| GET | `/api/data?start=&end=` | 기간 데이터 조회 |
| GET | `/api/summary` | 통계 요약 |
| GET | `/api/export_csv` | CSV 다운로드 |
| POST | `/api/label` | 라벨 저장 |
| GET | `/api/label_data?label=` | 라벨별 데이터 조회 |

## 데이터 포맷 예시
```
timestamp,gps_fix,uv,lux,pressure,state,score
2025-10-22 13:11:42,A,0.32,558,1019.78,Outdoor,0.742
2025-10-22 13:12:43,V,0.02,430,1019.99,Indoor,0.181
```

## 하드웨어 구성
- Raspberry Pi 5
- GPS (L80-R, CP2102 USB-UART)
- LTR390 (UV / Lux)
- BME280 (온도 / 습도 / 기압)
- LoRa 모듈 (RYLR998 / E22-230T22U)

## 향후 계획
- LoRa RSSI/SNR 분석 대시보드 추가
- MQTT / InfluxDB / Grafana 연동
- TFLite 기반 실내/실외 분류 고도화

---

### LICENSE
필요 시 추가해주세요.
