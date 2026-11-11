# Raspberry Pi 4 RC Controller

Pi 4가 MDD3A 후륜 모터 2개, 서보, MT6701, US-100 4개를 처리하며 Windows/macOS 브라우저로 조종합니다. LiDAR SDK 실행 파일은 사용자가 별도 터미널에서 실행하고, 서버는 출력만 수집합니다. 시뮬레이션 프로그램은 없습니다.

## 1. 이번 버전의 동작

- 접속 키 제거: 브라우저에서 Pi 주소를 열면 바로 연결됩니다. 같은 네트워크에서 주소에 접근할 수 있는 사람은 조종도 가능하므로 신뢰하는 사설 Wi-Fi에서 사용합니다.
- 기본 `control.mode="pwm"`: 엔코더 없이 직접 PWM으로 주행합니다. 엔코더 무응답 때문에 멈추지 않습니다. 실제 속도는 부하에 따라 달라집니다.
- 화면에서 구동 해제 후 `RPM PI 제어`로 바꿀 수 있습니다. 이 모드는 엔코더 보정과 피드백이 필요하며 무응답 정지를 유지합니다. 화면에서 바꾼 모드는 현재 실행에만 적용됩니다.
- 목표 RPM 설정 최대 **320 RPM**: 첨부 모터의 무부하 사양입니다. 정격 속도는 280 RPM이며 부하 상태에서 320 RPM 도달을 보장하지 않습니다.
- PWM 설정 최대 **100%**, 서보 명령 최대 **중심 기준 ±90°**(서보축 0–180°), 펄스 500–2500µs. 실제 바퀴 조향각은 링크비에 따라 달라집니다. 링크가 끝까지 움직일 수 있는지 확인하고 필요하면 범위를 줄이세요.
- 시작 슬라이더 값은 PWM 35%, RPM 20, 조향 10°입니다. 최대 범위는 슬라이더를 움직여 선택합니다. 구동은 버튼과 Shift+방향키를 눌러야 시작합니다.
- 통신 끊김 정지, 비상정지, 출력 증가 제한, 방향 전환 대기, pigpio 안전 타이머를 유지합니다.

실제 Pi 장비는 이 개발 환경에 연결되어 있지 않습니다. 코드 검사는 수행했으며 실제 하드웨어 동작은 Pi에서 확인해야 합니다.

## 2. 첨부 자료 분석과 확정/미확정 값

| 자료 | 확인 내용과 반영 |
|---|---|
| [GPIO_Pin.txt](GPIO_Pin.txt) | 모든 번호를 BCM GPIO로 해석. 아래 배선표와 `config.json`에 반영 |
| [모터 PNG](스크린샷%202026-11-08%20081911.png) | JGB37-520R30-12, 12V, 감속 1:30, 정격 280 RPM, 무부하 320 RPM, 11라인 엔코더, 스톨 3.2A |
| [서보 PNG](스크린샷%202026-11-08%20081923.png) | LD-1501MG, 6–8.4V, 500–2500µs, 0–180°, 스톨 약 2.4–3A. 요청에 따라 500–2500µs 전체 범위 사용 |
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

## 4. 설치 및 실행

필수 파일은 `controller.py`, `index.html`, `config.json`입니다. 설명은 이 README 하나에 모았습니다. `test_controller.py`는 회귀 검사 파일입니다.

기존 Pi 설정을 보관한 뒤 새 파일을 복사하세요. 기존에 확인한 `hardware_confirmed`, `servo.calibrated`, 모터 부호·핀·보정값은 새 설정에도 반영해야 합니다. 새 기본 설정은 확인 플래그가 false입니다. 새 프로그램에 이전 config를 계속 쓰면 기존 RPM/조향 최대값도 유지되므로 새 config의 범위를 반영하세요.

Pi에서 한 번 설치:

```bash
sudo apt update
sudo apt install -y pigpio python3-pigpio coreutils util-linux
sudo usermod -aG dialout "$USER"
```

그룹 변경 후 로그아웃/로그인합니다. OS 저장소에 pigpio가 없으면 [공식 설치 안내](https://github.com/joan2937/pigpio)를 확인하세요. Pi 4 Linux 전용이며 Pi 5는 지원하지 않습니다. Python 외부 의존성은 pigpio 하나입니다.

**첫 번째 Pi 터미널: 서버 실행**

```bash
cd ~/RC_Controller3
python3 controller.py --check-config
sudo pigpiod -l -s 5
python3 controller.py
```

pigpiod가 이미 실행 중이면 중복 실행하지 않습니다. `-l`은 pigpio를 Pi 내부 접속 전용으로 제한합니다. 제어 프로그램은 sudo 없이 실행하세요.

`hostname -I`로 Pi IP를 확인합니다. 같은 Wi-Fi의 Windows/macOS 브라우저에서 `http://Pi주소:8080`을 엽니다. **접속 키 입력은 없습니다.** 화면이 예전처럼 보이면 강력 새로고침(Ctrl+F5 또는 Cmd+Shift+R)하세요. 인터넷 포트 포워딩은 설정하지 않습니다.

## 5. LiDAR를 직접 실행해서 연결하기

서버는 SDK를 설치하거나 실행하지 않습니다. `config.json`의 LiDAR 설정은 다음과 같습니다. `executable`, `port`, `baud`는 더 이상 서버에서 사용하지 않습니다.

```json
"lidar": {
  "enabled": true,
  "fifo": "/tmp/rc_controller_lidar.fifo",
  "min_m": 0.05,
  "max_m": 12,
  "min_quality": 1,
  "angle_offset_deg": 0,
  "stale_s": 1.0
}
```

서버를 먼저 실행하면 위 경로에 출력 전달용 FIFO(이름 있는 파이프)를 만듭니다. **두 번째 Pi 터미널에서**, 기존에 단독 실행에 성공한 명령에 출력 전달을 붙이세요. 아래 실행 파일 경로와 USB 포트는 실제 값으로 바꿉니다.

```bash
test -p /tmp/rc_controller_lidar.fifo && \
stdbuf -oL -eL /home/사용자명/rplidar_sdk/output/Linux/Release/ultra_simple --channel --serial /dev/ttyUSB0 460800 > /tmp/rc_controller_lidar.fifo 2>&1
```

터미널에 숫자가 보이지 않는 것은 출력을 웹 서버로 전달하기 때문입니다. 화면의 LiDAR 영역에서 수신 줄 수, 측정 점 수, SDK 진단 문구를 확인할 수 있습니다. 화면 수신이 늦고 단독 터미널 실행에서는 정상이라면, 출력 버퍼를 피하는 다음 방식을 대신 사용하세요(동시에 두 SDK를 실행하지 않습니다).

```bash
test -p /tmp/rc_controller_lidar.fifo && \
script -q -c '/home/사용자명/rplidar_sdk/output/Linux/Release/ultra_simple --channel --serial /dev/ttyUSB0 460800' /dev/null > /tmp/rc_controller_lidar.fifo 2>&1
```

경로 검색: `find "$HOME" -type f -name ultra_simple 2>/dev/null`. 포트 검색: `ls -l /dev/serial/by-id/`. USB 포트가 바뀌면 명령의 포트를 수정합니다. C1의 baud는 460800입니다. SDK 파일은 Pi에서 실행 가능한 Linux ARM 빌드여야 합니다.

서버와 SDK를 같은 사용자 계정으로 실행하세요. FIFO 위치에 일반 파일이나 다른 사용자 소유 파일이 있으면 서버가 덮어쓰지 않고 오류를 표시합니다. 이 경우 config의 fifo를 다른 `/tmp/...` 경로로 바꾸고 출력 명령도 같은 경로로 바꾸세요. 서버가 먼저 준비되지 않으면 `test -p`가 실패하여 SDK가 시작되지 않습니다.

종료할 때는 **SDK 터미널에서 Ctrl+C → 서버 터미널에서 Ctrl+C** 순서입니다. 서버의 비상정지는 차량 구동만 멈춥니다. 외부 SDK와 LiDAR 회전은 사용자가 중지합니다. 서버를 재시작했다면 SDK 명령도 다시 실행하세요.

파서는 `theta / Dist / Q` 줄을 읽고 mm를 m로 변환합니다. 각도 정렬의 360→0 경계로 스캔을 나누며 0.5°당 최근 한 점(최대 720개)을 표시합니다. 거리/품질이 유효하지 않은 점을 제외하고 1초 이상 갱신이 없으면 점군을 비웁니다. FIFO 출력이 끊기면 기다렸다가 다시 연결된 출력을 받습니다. 현재 차량 중심의 2D 스캔이며 SLAM 누적 지도가 아닙니다. `angle_offset_deg`는 설치 방향 보정입니다. 전방 0°, 오른쪽 90°로 표시합니다.

## 6. 구동 활성화와 조종

배선·3.3V 신호·전원 확인 후 `hardware_confirmed=true`, 서보 기구 중립과 허용 범위를 확인한 후 `servo.calibrated=true`로 설정합니다. **PWM 모드에서는 모터의 encoder_confirmed가 false여도 활성화됩니다.** 첫 동작은 바퀴를 띄워 확인하세요. 확인 플래그가 자동 보정을 수행하는 것은 아닙니다.

- 구동 활성화 → Shift+W/S 또는 ↑/↓로 전후진, Shift+A/D 또는 ←/→로 조향합니다.
- 키 해제는 구동 출력 0, Space는 비상정지 잠금입니다. 잠금 해제 후 다시 활성화해야 출발합니다.
- PWM 모드: 출력 슬라이더가 직접 PWM을 정하며 RPM 슬라이더는 비활성입니다.
- RPM 모드: 속도 슬라이더가 목표 RPM, 출력 슬라이더가 PI 출력 상한입니다. 모터별 `encoder_confirmed=true`가 필요합니다.
- 창 이탈·탭 전환·통신 끊김 시 구동을 해제합니다. 재연결만으로 재출발하지 않습니다.
- 모드는 구동 해제 상태에서만 변경합니다. 한 탭만 조종 권한을 가집니다. 모니터링은 최대 4개 연결입니다.
- 서보 양수 방향이 반대면 `servo.sign`을 -1로 바꿉니다. 슬라이더는 서보축 명령각이며 실제 바퀴각은 아닙니다.

## 7. 설정·센서 보정·기록

설정 파일 변경 후 서버를 재시작합니다.

| 설정 | 의미 |
|---|---|
| control.mode | 기본 pwm 또는 rpm |
| max_rpm / max_duty | 목표 RPM 상한(320) / PWM 상한(1.0) |
| ramp_pwm_s / ramp_rpm_s | 초당 PWM 증가량(0.5) / 목표 RPM 증가량(80) |
| kp / ki | PI 비례·적분 계수. PWM/RPM 및 PWM/(RPM·초) |
| rpm_filter_s | RPM 저역 필터 시정수 0.12초 |
| command_timeout_s | 조종 신호 0.4초 미수신 시 해제 |
| stall_timeout_s | RPM 모드에서 저속/무응답 정지 판정 시간 0.7초 |
| reversal_pause_s | 방향 반전 전 출력 0 대기 0.3초 |
| motor_sign / encoder_sign | 모터 출력 방향과 엔코더 계수 방향을 각각 ±1로 보정 |
| encoder_counts_per_rev | 감속 출력축 1회전의 x4 카운트. 1320은 검증 전 추정 |
| wheel_ratio | 바퀴 1회전당 모터 출력축 회전 수, 직결이면 1 |
| servo min/center/max_us | 500/1500/2500µs. 기구 범위에 맞춰 제한 가능 |
| servo us_per_degree / rate_deg_s | 펄스-서보축 명령각 환산 / 명령각 변화 속도 |
| mt6701.ppr | 미확인이므로 null. 확인 후 1..1024 정수 |
| ultrasonic slot_s / timeout_s | 전역 65ms 순차 발사 / Echo 응답 30ms 제한 |
| recording | 기록 활성화, Hz, LiDAR 포함 여부, 파일 크기와 백업 수 |

출력축을 정확히 1회전시켜 엔코더 x4 카운트를 확인합니다. 전진에 해당하는 바퀴 회전에서 양수가 되도록 encoder_sign을 조정하고 모터 구동 방향은 motor_sign으로 별도 조정합니다. PWM 출력은 1kHz/1000단계입니다. PI 모드에서 출력 포화 시 적분을 제한하며 정지 시 적분을 초기화합니다.

```text
출력축 RPM = Δcount / counts_per_rev / Δ초 × 60
바퀴 RPM = 출력축 RPM / wheel_ratio
거리(m) = count / counts_per_rev / wheel_ratio × π × 바퀴 지름(mm) / 1000
MT 상대각 = (count - 영점 count) / (4 × PPR) × 360°
초음파 거리(m) = Echo 폭(µs) × 343 / 2,000,000
```

엔코더 신호가 검출되지 않으면 RPM은 null(`—`)입니다. PWM 주행 중 신호가 오래 없으면 누적 거리도 불완전한 것으로 표시하고 세션의 전체 거리 값을 숨깁니다. `encoder_distance_m`, `signed_distance_m`에는 실제 수신한 카운트에 기반한 부분 거리만 기록합니다. 거리 누락 표시는 서버 재시작 시 초기화됩니다. 엔코더만으로 정지와 단선을 완벽히 구분할 수는 없습니다.

누적 거리는 전후진 회전을 더한 두 바퀴 경로의 평균이며 슬립이나 타이어 변형을 보정하지 않습니다. 세션마다 0부터 계수합니다. Python 콜백은 고속/고부하에서 펄스를 놓칠 수 있으므로 반복 회전 카운트를 확인하세요.

MT6701 PPR이 null이면 카운트·A/B 상태·Z 검출만 표시합니다. 수동 한 회전 카운트÷4로 PPR을 확인할 수 있습니다. 256 PPR은 1024 counts/rev입니다. 수동 영점 버튼은 구동 해제 상태에서 현재 위치를 0°로 설정합니다. Z는 상승 횟수와 당시 카운트만 기록하며 영점을 덮어쓰지 않습니다. ABZ만으로 전원 투입 직후 절대각은 알 수 없습니다. EEPROM을 변경하지 않습니다.

서보 `command_deg`는 명령각이고 `measured_deg=null`입니다. MT6701을 실제 조향각으로 임의 매핑하지 않습니다. US-100은 센서당 약 3.8Hz, 기본 2cm–4m를 수집합니다. 실패·1초 이상 지난 값은 null로 표시하고 자동 장애물 제동은 하지 않습니다. 음속은 343m/s 설정이며 별도 온도 측정은 없습니다.

기록은 `logs/telemetry.jsonl`에 2Hz로 저장합니다. UTC 시각, 모드, 모터·엔코더·서보·MT·초음파·LiDAR 최신 스캔을 담습니다. 10MB마다 회전하고 백업 3개로 총 약 40MB를 유지합니다. 오래된 기록은 순환 삭제됩니다. `include_lidar=false`로 용량을 줄일 수 있습니다. 기록 오류는 화면에 표시합니다.

## 8. 코드 구조, 안전 동작, 문제 해결

`controller.py`의 Quadrature/PI는 계수·제어, Hardware는 GPIO와 pigpio 안전 타이머, Ultrasound/Lidar는 센서 수집, Controller는 모드·조종·정지, Runtime은 스레드·기록, Server/Handler는 HTTP/SSE를 담당합니다. `index.html`은 CSS/JS를 포함한 화면입니다. 외부 CDN이나 빌드 도구는 없습니다.

API는 GET `/api/state`, GET `/api/events`, POST `/api/command`입니다. 접속 키는 없습니다. 오래된 명령/순서 역전/이전 구동 세션을 거부하는 내부 lease와 seq는 그대로 사용합니다. 이 값은 사용자 로그인 키가 아닙니다. 웹사이트 간 요청은 Origin을 확인하지만 네트워크 접근 인증을 대신하지 않습니다.

0.4초 조종 신호 단절, 0.15초 제어 루프 지연, 비상정지는 두 모드 모두 정지시킵니다. Python heartbeat가 끊기면 pigpiod 스크립트가 약 0.5초 후 모터 PWM과 서보 펄스를 0으로 설정합니다. RPM 모드에는 무응답·방향 불일치·과속 감지가 있습니다. PWM 모드는 엔코더와 무관하게 동작하므로 이러한 엔코더 기반 정지는 적용하지 않습니다. 방향 반전 시 PWM 모드는 시간 대기, RPM 모드는 추가로 낮은 측정 RPM을 확인합니다.

OS/pigpiod 자체 정지나 전기 고장은 소프트웨어로 확실하게 차단할 수 없습니다. 모터 전원 비상 스위치를 준비하세요. 서보 ±90°는 센서 사양 범위이며 차량 기구의 허용 조향 범위를 보장하지 않습니다. 서보 펄스를 끄면 힘이 유지되지 않을 수 있습니다. SIGKILL 후 남은 daemon 스크립트는 모터 전원을 끄고 pigpiod를 재시작해 정리합니다. GPIO 프로그램은 하나만 실행하세요.

| 증상 | 확인 |
|---|---|
| LiDAR 수신 0줄 | 서버 먼저 실행, FIFO와 출력 명령 경로 일치, 같은 사용자, enabled=true |
| 수신은 있지만 측정 0개 | 화면 SDK 오류 문구, baud 460800, USB 포트, theta/Dist/Q 출력인지 확인 |
| LiDAR 단독 실행만 정상 | 위 script 방식으로 출력 버퍼 문제 확인. SDK 중복 실행 금지 |
| PWM 모드 활성화 불가 | hardware_confirmed와 servo.calibrated 확인. encoder_confirmed는 PWM에서 불필요 |
| 모터 방향 반대 | motor_sign/encoder_sign 각각 확인 |
| 초음파 응답 없음 | US-100 점퍼 제거, Trig/Echo 배선, Echo 전압, 공통 GND |
| 웹 연결 불가 | Pi IP/8080, 같은 네트워크, 방화벽/AP isolation |

검사 명령은 `python3 controller.py --check-config`, `python3 -m unittest -v test_controller.py`입니다. 실제 GPIO 타이밍·서보 범위·무선 정지 거리·Linux FIFO와 LiDAR 실물 연결은 Pi에서 확인해야 합니다. 다음 개선은 전류/전압 측정, 실제 조향각 보정, 차동 속도 보정, 엔코더 고속 수집 검증, 이후 SLAM입니다.

추가 참고: [SLAMTEC SDK 출력 소스](https://github.com/Slamtec/rplidar_sdk/blob/master/app/ultra_simple/main.cpp), [pigpio](https://github.com/joan2937/pigpio), [MT6701 데이터시트](https://www.magntek.com.cn/upload/pdf/202407/MT6701_Rev.1.8.pdf), [US-100](https://www.adafruit.com/product/4019), [Pi GPIO](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html).
