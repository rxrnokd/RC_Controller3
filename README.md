# Raspberry Pi 4 RC Controller — 초기 버전

Raspberry Pi 4 한 대가 MDD3A를 통해 후륜 DC 모터 2개를 제어하고, 엔코더·MT6701·US-100 4개·RPLIDAR C1 데이터를 수집합니다. Windows/macOS 노트북은 같은 Wi-Fi에서 브라우저만 열면 됩니다. 별도 MCU, ROS, Node.js, 빌드 도구, 시뮬레이션 프로그램은 필요하지 않습니다.

**구현된 초기 버전이며 실차 검증은 아직 하지 않았습니다.** 현재 개발 환경은 Windows이며 실제 Pi/GPIO 장비가 연결되어 있지 않습니다. 출고 설정에서는 모터·서보가 움직이지 않습니다. 아래 배선·엔코더·서보 보정을 확인한 뒤 설정의 확인 항목을 변경해야 구동을 활성화할 수 있습니다. 센서 수집과 웹 모니터링은 구동 비활성 상태에서도 동작합니다.

## 1. 구현 범위와 구성

| 기능 | 동작 |
|---|---|
| DC 모터 2개 | 모터별 A/B x4 디코딩, 출력축 RPM 기준 PI, 목표 속도 증가 제한, PWM 상한 |
| MT6701 | A/B 누적 카운트, Z 상승 횟수·당시 카운트, 설정된 PPR 기준 상대각, 수동 영점 |
| US-100 4개 | Trig/Echo 모드, 순차 측정, 응답 시간 제한, 거리·원시 펄스 폭·데이터 나이 |
| RPLIDAR C1 | 사용자가 설치한 `ultra_simple`을 자식 프로세스로 실행하여 stdout 처리 |
| 서보 | 중심 기준 명령각, 펄스 폭, 각도/회전 속도 제한. 실제 조향각은 미측정 |
| 무선 조종 | Wi-Fi + TCP. HTTP POST로 명령, SSE로 센서 스트림. 인증 키 및 단일 조종 탭 |
| 웹 화면 | 가장 큰 LiDAR 2D 점군, RPM·거리·엔코더·서보·초음파, 출력 상한 막대, 키보드 조종 |
| 기록 | Pi의 `logs/telemetry.jsonl`에 센서 데이터 기록, 크기 제한 및 순환 보관 |

통신은 TCP 기반 HTTP/SSE를 선택했습니다. 브라우저에서 UDP 소켓이나 Bluetooth 직렬 포트를 따로 설정할 필요가 없고 Windows/macOS 공통으로 사용할 수 있습니다. 제어 명령과 센서 스트림은 별도 연결이며, 오래된 명령은 거부합니다. 서버 측 명령 시간 제한으로 TCP 재전송 중에도 계속 주행하지 않게 했습니다. 명령은 약 10Hz, 모니터링은 약 10Hz, PI 루프는 50Hz 목표입니다. Linux/Python은 실시간 OS가 아니므로 주기는 부하에 따라 달라집니다.

초기 버전은 두 후륜에 동일한 목표 RPM을 줍니다. 좌우 차동 속도 보정, Ackermann 계산, 자동 장애물 제동은 포함하지 않았습니다. 초음파와 MT6701은 요청대로 독립적인 데이터 수집에만 사용합니다.

필수 파일은 아래 4개입니다. 모든 설명은 이 README에 모았습니다.

```text
controller.py       GPIO, 제어, 센서, LiDAR 프로세스, 통신, 기록
index.html          CSS/JavaScript를 포함한 브라우저 화면
config.json         핀 배치, 센서 및 제어 설정
README.md           설치·실행·보정·구조·제한사항
test_controller.py  선택: 하드웨어 없이 실행하는 회귀 테스트
```

`test_controller.py`는 함수·통신·정지 조건 검사용이며 가상 RC카나 센서 시뮬레이터가 아닙니다. GPIO가 없으면 실제 서버는 실행되지 않습니다.

## 2. 첨부 자료 분석과 확정/미확정 값

| 자료 | 확인 내용과 반영 |
|---|---|
| [GPIO_Pin.txt](GPIO_Pin.txt) | 모든 번호를 BCM GPIO로 해석. 아래 배선표와 `config.json`에 반영 |
| [모터 PNG](스크린샷%202026-11-08%20081911.png) | JGB37-520R30-12, 12V, 감속 1:30, 정격 280 RPM, 무부하 320 RPM, 11라인 엔코더, 스톨 3.2A |
| [서보 PNG](스크린샷%202026-11-08%20081923.png) | LD-1501MG, 6–8.4V, 500–2500µs, 0–180°, 스톨 약 2.4–3A. 기구 보호를 위해 초기 펄스 범위는 1300–1700µs로 축소 |
| [MDD3A_Datasheet.pdf](MDD3A_Datasheet.pdf) | 문서 p.2–3: 4–16V, 채널당 연속 3A/피크 5A(<5초), PWM 최대 20kHz, 논리 HIGH ≥1.7V, A/B 모두 LOW이면 Brake |
| [C1 데이터시트](SLAMTEC_rplidar_datasheet_C1_v1.2_en.pdf) | p.9,11,13–14: UART 460800, 0.05–12m, 5k samples/s, 통상 10Hz, 시계방향 각도 증가, 5V 전원·기동 약 800mA |
| [C1 키트 사용 설명서](SLAMTEC_rplidarkit_usermanual_C1_v1.2_en.pdf) | USB 어댑터 연결, 스캔 시작/중지, 전원·핀 정의, 정확도를 위한 2분 이상 예열 권장 |
| [SDK V2 설명서](LR002_SLAMTEC_rplidar_sdk_v2.0_en.pdf) | p.5,7–8: Linux 실행 파일 위치와 `--channel --serial <port> [baudrate]`, 콘솔의 theta/Dist/Q 형식 |

PDF 전체의 추출 텍스트를 검토했고 전원 표, 진리표, 좌표계, 실행법은 해당 페이지를 이미지로 렌더링해 확인했습니다. 두 PNG도 직접 확인했습니다. SDK 문서의 A2/A3 예제 전송속도 대신 C1 데이터시트의 **460800**을 사용합니다.

사용자가 확인한 값: **바퀴 지름 75mm, 후륜 구동, MDD3A, 초음파 US-100, MT6701 PPR 미확인**.

미확정 항목은 다음처럼 처리합니다.

- 모터 엔코더: 11라인을 모터축 A상 11주기/회전으로 해석하면 출력축 x4 카운트는 `11 × 4 × 30 = 1320`입니다. 이는 **보정 전 가정**입니다. 실제 출력축을 한 바퀴 돌려 카운트를 확인해야 합니다.
- 출력축→바퀴 추가 감속은 우선 `wheel_ratio=1`로 설정했습니다. 별도 감속이 있으면 수정합니다.
- MT6701 PPR은 `null`로 두었습니다. 카운트·A/B·Z는 수집하되 각도는 `—`입니다. 임의 PPR로 잘못된 각도를 보여주지 않습니다.
- M1/M2가 실제 좌/우 어느 쪽인지, 조향 펄스 방향과 링크비는 미확인입니다. PWM 및 엔코더 부호를 각각 보정합니다.
- Pi OS와 전원 구성은 미확인입니다. 이 코드는 **Pi 4 Linux + pigpiod** 전용이며 Pi 5용이 아닙니다. OS 패키지 제공 여부는 아래 설치 단계에서 확인합니다.

## 3. 배선과 전원

**표는 물리 핀 번호가 아니라 BCM 번호입니다.** 외부 모터·서보 전원을 끈 상태에서 연결하세요.

| 장치 | 신호 | BCM GPIO |
|---|---|---|
| MDD3A M1 | M1A / M1B | 16 / 20 |
| 모터 1 엔코더 | A / B | 4 / 14 |
| MDD3A M2 | M2A / M2B | 21 / 26 |
| 모터 2 엔코더 | A / B | 15 / 18 |
| 서보 | PWM 신호 | 13 |
| MT6701 | A / B / Z | 19 / 10 / 9 |
| US-100 1 | Trig / Echo | 17 / 27 |
| US-100 2 | Trig / Echo | 22 / 23 |
| US-100 3 | Trig / Echo | 24 / 25 |
| US-100 4 | Trig / Echo | 5 / 6 |
| LiDAR | 키트 USB 어댑터 | Pi USB, 일반적으로 `/dev/ttyUSB0` |

전원 구성:

- Pi 4에는 안정적인 전용 5V 전원 공급. 모터는 MDD3A를 통해 12V 계통 사용.
- 서보는 별도 6–8.4V 전원/BEC 사용. 첨부 서보 스톨 전류를 감당할 수 있어야 합니다. 서보 전원을 Pi GPIO 전원에 연결하지 마세요.
- **MDD3A의 5VO는 최대 200mA**입니다. Pi, 서보, LiDAR 전원으로 사용하지 마세요.
- Pi·드라이버·엔코더·센서·서보 신호 기준 GND를 공통으로 연결합니다. 모터 고전류 귀환 경로가 Pi 신호 접지를 통과하지 않도록 배선합니다.
- Pi GPIO 입력은 3.3V 신호만 허용합니다. 모터 엔코더/MT6701 보드가 5V 출력이면 레벨 변환이 필요합니다. 전원 전압만으로 출력 레벨을 추측하지 마세요.
- **US-100은 뒤쪽 점퍼를 제거하여 Trig/Echo 모드로 사용**합니다. Pi에는 3.3V 전원/논리로 연결하는 구성을 권장합니다. 5V로 공급한다면 실제 Echo 전압을 확인하고 필요시 레벨 변환합니다. 핀 인쇄를 보고 VCC/GND를 구분합니다.
- MDD3A 입력 4개에는 부팅/재부팅 중 부유 방지를 위한 외부 풀다운(예: 10kΩ)을 권장합니다. 하드웨어 비상 스위치는 모터 전원을 직접 끊도록 구성하세요.
- 모터 스톨 3.2A는 MDD3A 연속 정격 3A를 넘습니다. 바퀴를 장시간 붙잡거나 기구를 막지 마세요. 초기 출력 제한·정지 감지는 전류 제한 회로를 대신하지 않습니다.

GPIO 14/15는 UART, 9/10은 SPI, 4는 1-Wire와 충돌할 수 있습니다. `sudo raspi-config`에서 **Serial login shell과 Serial hardware, SPI, 1-Wire를 비활성화**하고 재부팅합니다. I2S/PCM 오디오 HAT, 다른 GPIO 프로그램, PWM 오버레이가 표의 핀을 사용하지 않게 하세요. C1은 GPIO UART 대신 USB 어댑터를 사용합니다.

## 4. 설치 및 가장 짧은 실행 순서

### Pi에서 한 번 설치

Pi와 노트북을 같은 신뢰할 수 있는 Wi-Fi에 연결합니다. Windows PowerShell 또는 macOS Terminal에서 SSH로 Pi에 접속하거나 Pi 터미널을 직접 사용합니다. 아래 명령은 **Pi에서 실행**합니다.

```bash
sudo apt update
sudo apt install -y pigpio python3-pigpio coreutils
python3 -c "import pigpio; print(pigpio.VERSION)"
sudo usermod -aG dialout "$USER"
```

`dialout` 변경 후 로그아웃/로그인합니다. 특정 OS 저장소에 pigpio 패키지가 없다면 오류를 무시하고 진행하지 마세요. [pigpio 공식 저장소](https://github.com/joan2937/pigpio)의 설치 절차를 사용하거나 OS 버전을 확인해야 합니다. 이 프로젝트의 Python 외부 의존성은 `pigpio` 하나입니다. `python3-pigpio`를 설치했다면 별도 pip/가상환경은 필요 없습니다.

필수 파일 4개를 Pi의 같은 폴더(예: `~/RC_Controller3`)에 복사합니다. 기존 첨부 자료는 제어 실행에 필요하지 않습니다. 노트북 프로젝트 폴더에서 복사하는 예:

```bash
scp controller.py index.html config.json README.md 사용자명@Pi주소:~/RC_Controller3/
```

대상 폴더는 먼저 Pi에서 `mkdir -p ~/RC_Controller3`로 만드세요. `사용자명`과 `Pi주소`는 실제 값으로 바꿉니다.

### 매번 실행

```bash
cd ~/RC_Controller3
python3 controller.py --check-config
sudo pigpiod -l -s 5
python3 controller.py
```

`-l`은 pigpio의 원격 소켓 접속을 막고 Pi 내부 연결만 허용합니다. `-s 5`는 5µs GPIO 샘플링입니다. pigpiod가 이미 서비스로 실행 중이면 중복 실행하지 말고 해당 서비스가 로컬 접속 전용으로 구성되어 있는지 확인하세요. 샘플 주기나 옵션을 바꾸려면 **모터 전원을 끄고** 기존 pigpiod를 정상 종료한 뒤 다시 시작합니다. 제어 서버는 `sudo` 없이 실행합니다.

터미널에 아래처럼 표시됩니다.

```text
브라우저: http://<Pi IP>:8080
접속 키: 실행 시 생성되는 무작위 키
```

Pi에서 `hostname -I`로 Wi-Fi IP를 확인하고 노트북 Chrome/Edge/Firefox/Safari에서 `http://실제Pi주소:8080`을 엽니다. 터미널의 키를 입력하여 연결합니다. Windows/macOS 노트북에 Python을 설치할 필요는 없습니다. 키는 메모리에만 유지하고 페이지를 다시 열면 재입력합니다. 필요하면 시작 전 `export RC_TOKEN='12자-이상의-임의-긴-키'`로 고정할 수 있으나 공개 저장소에 넣지 마세요.

HTTP는 암호화되지 않습니다. 사설 Wi-Fi에서 사용하고 공유기 외부 포트 포워딩을 하지 마세요. 원격 인터넷 조종은 초기 버전 범위가 아닙니다. 공유기 AP isolation/게스트 네트워크가 장치 간 통신을 막으면 같은 일반 네트워크로 옮깁니다.

종료는 Pi 터미널에서 **Ctrl+C**입니다. 모터 PWM 0, 서보 펄스 중지, LiDAR SDK에 SIGINT 후 종료 대기 순서로 정리합니다. 기본적으로 자동 부팅/주행 서비스는 설치하지 않습니다.

## 5. LiDAR 실행 파일 — 사용자가 설치할 위치와 실행법

SDK 설치·컴파일은 요청대로 이 프로젝트에서 수행하지 않습니다. **Pi의 CPU/OS에 맞게 빌드된 Linux ARM 실행 파일**을 준비하세요. Windows `.exe`나 x86 Linux 파일은 Pi에서 실행되지 않습니다. C1을 지원하는 SDK 버전을 사용합니다.

SDK의 일반적인 빌드 결과 경로:

```text
/home/실제사용자/rplidar_sdk/output/Linux/Release/ultra_simple
```

위치는 자유지만 `config.json`에 **실제 절대 경로**를 입력해야 합니다. 기본 `/home/pi/...`는 예시이고 사용자명이 pi가 아니면 변경해야 합니다.

먼저 **Pi에서**, 제어 서버를 종료한 상태로 단독 확인합니다.

```bash
ls -l /dev/serial/by-id/
chmod +x /home/실제사용자/rplidar_sdk/output/Linux/Release/ultra_simple
stdbuf -oL -eL /home/실제사용자/rplidar_sdk/output/Linux/Release/ultra_simple --channel --serial /dev/ttyUSB0 460800
```

정상 출력 예는 `theta: 123.45 Dist: 001234.00 Q: 47` 형태입니다. `Dist`의 단위는 mm이며 서버는 m로 변환합니다. `Ctrl+C`로 단독 실행을 끝낸 뒤 `config.json`을 변경합니다.

```json
"lidar": {
  "enabled": true,
  "executable": "/home/실제사용자/rplidar_sdk/output/Linux/Release/ultra_simple",
  "port": "/dev/ttyUSB0",
  "baud": 460800,
  "min_m": 0.05,
  "max_m": 12,
  "min_quality": 1,
  "angle_offset_deg": 0,
  "stale_s": 1.0
}
```

이후 `python3 controller.py` **한 번만 실행**하면 서버가 SDK를 자동 시작하고 stdout/stderr를 읽습니다. 별도 터미널에서 SDK를 동시에 실행하면 직렬 포트가 충돌합니다. USB 포트 이름 변경을 피하려면 확인한 `/dev/serial/by-id/...` 경로를 `port`에 넣으세요.

`stdbuf`는 파이프 연결 시 출력 버퍼 때문에 스캔이 늦어지는 것을 줄입니다. 실행 파일이 정적으로 링크되어 `stdbuf`가 효과가 없으면 출력이 지연될 수 있으므로 실제 출력 갱신을 확인합니다. 파서는 각도 정렬의 360→0 경계로 스캔을 나누고 0.5° 구간당 최근 한 점, 최대 720점을 보냅니다. 품질 0, 거리 0, 범위 밖 점은 제외합니다. 1초 넘게 새 스캔이 없으면 화면 점군을 비웁니다. 프로세스 오류는 UI에 표시하며 자동 재시작하지 않습니다.

차량 전방과 LiDAR 0°가 다르면 `angle_offset_deg`를 조절하세요. 전방 0°, 오른쪽 90°로 그립니다. 현재 차량 기준의 **2D 스캔**이며, 이동 중 쌓아가는 SLAM 지도나 실제 차량 궤적은 아닙니다. 센서의 정확도를 위해 시작 후 2분 정도 예열할 수 있습니다.

## 6. 첫 구동 전 보정

1. 모터·서보 전원은 끄고 Pi와 센서만 켭니다. 바퀴를 손으로 돌려 각 모터 엔코더 카운트가 변하는지 확인합니다. 출력축 정확히 한 회전의 x4 카운트를 `encoder_counts_per_rev`에 입력합니다. 기본 1320을 실측 없이 확정하지 마세요.
2. 차량 전진에 해당하는 바퀴 회전에서 두 엔코더가 모두 양수가 되도록 `encoder_sign`을 1 또는 -1로 정합니다. 좌우 거울 배치이면 서로 다를 수 있습니다. 모터 구동 극성 `motor_sign`은 별개이며 PWM A/B 배선·모터 극성으로 확인합니다.
3. 바퀴 75mm와 추가 감속비를 확인합니다. `wheel_ratio`는 **바퀴 1회전당 모터 출력축 회전 수**입니다. 직결이면 1입니다.
4. 서보 링크를 분리하거나 바퀴를 들어 기구 간섭 없이 확인합니다. `center_us=1500`은 시작값이지 기구 중립이 보장된 값이 아닙니다. 중립·허용 범위·방향을 확인하고 `min_us`, `center_us`, `max_us`, `sign`, `max_angle_deg`를 맞춥니다. `us_per_degree=11.111111`은 첨부 2000µs/180°에서 산출한 서보축 환산값입니다. 바퀴 조향 링크비를 뜻하지 않습니다.
5. 배선·전압 검토 후 `hardware_confirmed=true`, 모터별 `encoder_confirmed=true`, 서보 `calibrated=true`로 변경하고 서버를 재시작합니다. 설정은 실행 중 자동으로 다시 읽지 않습니다.
6. **바퀴를 지면에서 띄워 고정하고**, 목표 속도 10–20 RPM, 출력 상한을 낮게 시작합니다. 구동 활성화 후 Shift+W를 짧게 눌러 전진 방향과 양의 RPM을 확인합니다. 반대이면 즉시 정지하고 해당 부호를 수정합니다. 낮은 출력에서 회전하지 않으면 계속 붙잡거나 높은 출력으로 밀지 말고 배선·극성·전원을 먼저 확인합니다.
7. 키 해제, Space, 창 전환, Wi-Fi 단절, Ctrl+C를 각각 시험하여 정지를 확인한 뒤 지상에서 낮은 속도로 시험합니다. 실제 정지 거리와 출력 제한은 적재량/노면에 따라 조정해야 합니다.

보정 전에도 RPM·거리 추정은 표시되지만 UI에 보정 전임을 표시합니다. 초기 확인 플래그는 물리 검증을 대신하지 않으며 단순히 true로 바꾼다고 장치가 검증되는 것은 아닙니다.

## 7. 조종 방법

- **구동 활성화**: 해당 브라우저 탭에 조종 권한을 부여합니다. 다른 탭은 모니터링과 정지만 가능하며 권한을 가로채지 못합니다.
- **Shift + W/S 또는 ↑/↓**: 전진/후진. 키를 놓으면 구동 목표와 출력은 0이 됩니다. MDD3A의 A/B LOW 동작은 브레이크입니다.
- **Shift + A/D 또는 ←/→**: 설정한 최대 조향 명령각. 양수는 화면에서 오른쪽이며 실제 방향은 `servo.sign`으로 맞춥니다.
- **출력 상한 막대**: PI 제어의 최대 PWM 비율. 속도 막대는 목표 RPM입니다. 출력이 낮으면 부하를 이기지 못해 목표 RPM에 도달하지 않을 수 있습니다.
- **Space / 비상정지**: 정지 상태를 잠급니다. 원인을 확인하고 ‘정지 잠금 해제’ → ‘구동 활성화’를 눌러야 재출발합니다.
- **구동 해제**: 모터 정지와 서보 펄스 중지. 통신 끊김·포커스 이탈 후 자동 재출발은 없습니다.
- **MT 상대 영점**: 구동 해제 상태에서 현재 센서 카운트를 상대각의 0°로 설정합니다. 물리 중앙이나 Z 절대 위치를 자동 인식하는 기능은 아닙니다.

슬라이더/입력칸에 키보드 포커스가 있으면 방향키는 해당 입력 조작에 쓰입니다. 주행하려면 빈 영역을 클릭하세요. 브라우저의 ‘구동 활성화’ 버튼은 클릭 후 포커스를 해제하므로 바로 키보드 조종할 수 있습니다.

## 8. 센서 값과 계산

모터별 PI 오차는 `목표 출력축 RPM - 측정 RPM`입니다. PWM 단위는 0..1, `kp` 단위는 PWM/RPM, `ki`는 PWM/(RPM·s)입니다. 출력 포화 시 적분 누적을 제한하고 정지 시 적분을 초기화합니다. 목표속도 도달 전 출력이 포화되면 상한을 확인하세요. 떨림이 있으면 우선 `ki`를 낮추고 이후 `kp`를 조절합니다. 현재 80 RPM/35%는 검증된 최적값이 아니라 낮은 시작 제한입니다.

```text
출력축 RPM = Δ카운트 / 출력축 counts_per_rev / Δ시간 × 60
바퀴 RPM   = 출력축 RPM / wheel_ratio
거리(m)    = 누적 카운트 / counts_per_rev / wheel_ratio × π × 지름(mm) / 1000
MT 상대각  = (현재 카운트 - 영점 카운트) / (4 × PPR) × 360°
초음파(m)  = Echo 폭(µs) × 343 / 2,000,000
```

- RPM은 약 0.12초 시정수로 필터링합니다. PWM은 1kHz/1000단계입니다.
- 누적 이동거리는 전후진 모두 더하는 바퀴 회전 경로이며 두 바퀴 평균입니다. 부호가 있는 변위는 기록의 `signed_distance_m`입니다. 슬립·제자리 회전·타이어 변형을 보정한 실제 차체 위치는 아닙니다. 프로그램 재시작 시 0으로 시작합니다.
- MT6701은 PPR을 확인한 후 `mt6701.ppr`에 입력합니다. 제조사 ABZ PPR 범위는 1..1024입니다. PPR과 x4 카운트를 혼동하지 마세요. 예를 들어 256 PPR은 1024 counts/rev입니다.
- MT6701을 손으로 정확히 1회전시킬 수 있다면 A/B 카운트 차이의 절댓값÷4로 PPR을 확인할 수 있습니다. 여러 회전에서 일관되는지 확인하세요. 이 프로그램은 MT6701 EEPROM 설정을 읽거나 바꾸지 않습니다.
- Z는 상승 횟수와 해당 카운트만 기록합니다. Z 검출 시 누적 카운트나 사용자가 지정한 영점을 덮어쓰지 않습니다. `since_index_deg`는 마지막 Z 이후 변화량이며 절대 조향각이 아닙니다. 센서를 한 바퀴 돌릴 수 없는 조향 기구라면 Z를 못 지날 수 있습니다.
- A/B/Z만으로 전원 투입 직후 절대각은 알 수 없습니다. PPR 설정 전에는 raw count, A/B 상태, Z 횟수를 사용할 수 있습니다.
- 서보 `command_deg`는 실제로 보낸 펄스의 명령 환산각이고, `measured_deg=null`입니다. MT6701을 서보 실측값으로 임의 연결하지 않았습니다.
- US-100은 전역 약 65ms 간격으로 하나씩 발사하므로 센서당 약 3.8Hz입니다. 30ms 응답 제한, 2cm–4m 범위 필터를 적용합니다. 실패 시 `null`, 1초 이상 지난 값도 `null`입니다. 음속 환산은 설정값 343m/s이며 실제 기준 거리에서 오차를 확인하세요. Trig/Echo 모드에서 별도 온도 데이터를 얻지는 않습니다.

Python GPIO 콜백에서 엔코더를 처리하므로 높은 회전수와 CPU 부하에서 누락이 생길 수 있습니다. 한 바퀴 카운트 반복성, CPU 부하, RPM을 실측 확인하세요. `invalid` 카운터만으로 누락을 모두 검출할 수는 없습니다.

## 9. 기록과 코드 구조

기록은 기본 2Hz이며 각 줄이 독립 JSON 객체입니다. UTC 시각, 단조 시계 시각, 모터 RPM·카운트·거리·PWM, 초음파 거리·펄스 폭·상태, MT 상대각·카운트·Z, 서보 명령, LiDAR 점군을 포함합니다. LiDAR 원본 5kHz 전체 로그가 아니라 웹에 제공하는 최신 스캔을 일정 간격으로 저장합니다.

`logs/telemetry.jsonl`은 약 10MB마다 회전하고 `.1`~`.3`을 유지하므로 총 약 40MB로 제한합니다. 오래된 로그는 순환 삭제됩니다. 계속 보관하려면 주행 후 노트북으로 복사하세요. `recording.include_lidar=false`면 용량을 줄일 수 있습니다. 디스크 쓰기 실패는 UI에 표시하며 기록만 중지합니다.

`controller.py` 내부 역할:

| 구성 | 역할 |
|---|---|
| `validate_config` | 핀 중복, 숫자/범위, 서보 펄스 제한 등 실행 전 검증 |
| `Quadrature`, `PI` | 엔코더 계수와 적분 포화 방지 제어 |
| `Hardware` | pigpio 연결, PWM·서보 출력, Python과 독립된 daemon 안전 스크립트 |
| `Ultrasound`, `Lidar` | 센서 수집 스레드와 SDK 프로세스/파싱 |
| `Controller` | 조종 권한, 타임아웃, 방향 전환, 구동·안전 상태 |
| `Runtime` | 스레드 시작/종료, 통합 스냅샷, 파일 기록 |
| `Server`, `Handler` | 정적 화면, 인증된 HTTP 명령/SSE 스트림 |

명령은 `POST /api/command`, 데이터는 `GET /api/events`(SSE) 또는 `GET /api/state`입니다. API는 `X-RC-Token` 헤더가 필요합니다. 조종 명령에는 최근 서버 시각(`seen`), 탭 ID, 활성화 시 발급된 lease, 증가하는 seq가 포함됩니다. 페이지는 외부 CDN/폰트/라이브러리를 요청하지 않습니다.

## 10. 안전 기능과 한계

구현된 기능:

- 시작 시 출력 0, 수동 구동 활성화, 설정 확인 전 출력 잠금.
- 조종 명령이 0.4초 이상 끊기거나 제어 루프가 0.15초 넘게 지연되면 구동 해제.
- 0.4초보다 오래된 서버 데이터에 기반한 명령, NaN/무한대·범위 초과 값, 역순 seq, 이전 lease를 거부.
- Shift deadman, 창 이탈·탭 숨김·페이지 종료 시 정지 요청, 서버 자체 타임아웃.
- 비상정지 잠금, 속도·PWM·서보 범위 제한, PI 적분 제한.
- 방향 반전 시 출력 0으로 최소 0.3초 기다리고 측정 RPM이 낮아진 뒤 반전.
- 출력이 걸려도 일정 시간 RPM이 낮으면 정지, 구동 방향과 엔코더 방향이 반대이면 정지, 과속 감지.
- pigpiod 내 스크립트가 Python heartbeat 중단 약 0.5초 후 모터 PWM과 서보 펄스를 0으로 반복 설정. 제어 루프가 안전 스크립트의 실행 상태도 확인.
- 느린 SSE 연결을 제어 루프와 분리하고 모니터링 최대 4개·전체 HTTP 작업 최대 12개로 제한.

이는 소프트웨어 정지 기능입니다. **OS 전체 정지, pigpiod 자체 정지, 전기적 고장 시 출력을 확실히 끊는 하드웨어 회로를 대체하지 않습니다.** 서보 펄스를 끄면 힘을 유지하지 않을 수 있고 차가 관성으로 움직일 수 있습니다. MDD3A Brake도 기계식 주차 브레이크가 아닙니다. 전원 비상 차단과 실제 정지 거리 시험이 필요합니다.

`SIGKILL` 후에는 정리를 못한 daemon 스크립트가 남습니다. 이 경우 모터 전원을 끄고 pigpiod를 재시작한 뒤 서버를 실행하세요. 같은 폴더의 서버 중복 실행은 파일 잠금으로 거부하지만 다른 폴더의 복사본/다른 GPIO 프로그램까지 차단하지는 않습니다. GPIO 제어 프로그램은 하나만 실행하세요.

다음 단계 개선 아이디어: 실측 PI 튜닝, 하드웨어 전원 차단 watchdog, 모터 전류/배터리 전압 측정, 엔코더 수집 부하 검증 및 필요시 Pi 측 C 구현, 조향 실제각 보정, 차동 속도 보정, 실제 센서 위치에 기반한 선택적 충돌 정지, 이후 위치 추정/SLAM. 현재 요청 범위에는 자동 제동과 시뮬레이션을 추가하지 않았습니다.

## 11. 검사 및 문제 해결

하드웨어에 접근하지 않는 검사:

```bash
python3 controller.py --check-config
python3 -m unittest -v test_controller.py
```

33개 테스트는 설정 검증, x4 카운트·거리, PI 포화, deadman, 통신 시간 초과, 재활성화·권한, 역방향 전환, 모터 정지 감지, LiDAR 필터/스캔 경계/만료, 초음파 타이머 래핑, HTTP 인증·SSE, 출력 정지 오류 처리와 watchdog 상태 검사를 통과했습니다. 실제 Chrome에서도 데스크톱/모바일 화면의 가로 넘침 없음, 데이터 표시, Shift 해제·Space 명령과 JavaScript 오류 없음을 확인했습니다. 브라우저 검사는 고정 테스트 입력을 사용한 UI 검사이며 실차 검증이 아닙니다. 실제 엔코더 주파수, pigpio DMA/안전 스크립트, 서보 기구 범위, LiDAR 장치, 무선 지연/정지 거리는 Pi에서 별도 시험해야 합니다.

| 증상 | 확인 |
|---|---|
| 서버 시작 실패 | Pi 4/Linux인지, pigpiod 실행 여부, `import pigpio`, 설정 오류 |
| 구동 활성화 버튼 비활성 | 화면의 설정 확인 항목, 비상정지 잠금, 다른 탭의 조종 상태 |
| 모터가 반대로 돌거나 정지 잠금 | `motor_sign`과 `encoder_sign`을 각각 확인. 전진 시 두 RPM 모두 양수 |
| RPM/거리가 4배 또는 30배 다름 | PPR와 x4 카운트, 모터축과 감속 출력축 구분 |
| 초음파 응답 없음 | US-100 점퍼 제거, Trig/Echo 뒤바뀜, GND·3.3V, Echo 전압, 반사체 거리 |
| LiDAR 대기/오류 | 실행 경로·ARM 파일·실행 권한, 460800, USB 포트, dialout, 다른 SDK 프로세스 점유 |
| LiDAR만 멈춤 | SDK 진단 문구/USB 전원 확인 후 서버 재시작. 구동은 별개이며 자동 장애물 제동 없음 |
| 화면 접속 불가 | 실제 Pi IP·8080, 방화벽, 같은 Wi-Fi, 게스트/AP isolation 여부 |
| 자주 시간 초과 | Wi-Fi 품질, Pi 전원/부하, 로그/USB 부하 확인. 제한 시간을 무작정 늘리지 않기 |

추가 확인에 사용한 원문:

- [SLAMTEC ultra_simple 소스](https://github.com/Slamtec/rplidar_sdk/blob/master/app/ultra_simple/main.cpp): 실행 인자, 각도 정렬, theta/Dist/Q 단위와 종료 방식.
- [pigpio Python 인터페이스 소스](https://github.com/joan2937/pigpio/blob/master/pigpio.py), [스크립트 명령 원문](https://github.com/joan2937/pigpio/blob/master/pigs.1): GPIO 콜백·PWM·서보·daemon 스크립트.
- [MagnTek MT6701 Rev.1.8](https://www.magntek.com.cn/upload/pdf/202407/MT6701_Rev.1.8.pdf): ABZ PPR, x4 환산, Z 출력.
- [Adafruit US-100 제품 설명](https://www.adafruit.com/product/4019), [센서 핀 설명](https://learn.adafruit.com/ultrasonic-sonar-distance-sensors/pinouts): 점퍼 제거 Trig/Echo 모드, 3.3V 지원 및 10µs 트리거.
- [Raspberry Pi GPIO 문서](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html), [인터페이스 설정](https://www.raspberrypi.com/documentation/computers/configuration.html): GPIO 신호 레벨 및 핀 다중 기능.
