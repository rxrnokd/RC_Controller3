# Raspberry Pi 4 RC Controller — 최종 사용 안내

Raspberry Pi 4 한 대로 후륜 DC 모터 2개, 서보, MT6701, US-100 4개를 처리하고 Windows/macOS 노트북 브라우저에서 조종합니다. LiDAR는 사용자가 제조사 `ultra_simple`을 직접 실행하며, 이 프로그램은 그 출력만 수집합니다.

**실행 순서: Pi 터미널 1에서 제어 서버 → Pi 터미널 2에서 LiDAR → 노트북 브라우저에서 접속·조종.**

접속 키는 없습니다. 기본 모드는 엔코더 없이 주행할 수 있는 PWM 직접 제어입니다. 시뮬레이션 프로그램은 포함하지 않습니다.

## 1. 파일과 준비 사항

| 파일 | 역할 |
|---|---|
| `controller.py` | Pi GPIO, 모터 제어, 센서 수집, HTTP/SSE, 기록 |
| `index.html` | 웹 화면과 키보드 조종 |
| `config.json` | 핀 번호, 모드, 출력 범위, 센서 설정 |
| `README.md` | 설치·실행·설정·보정·문제 해결 안내 |
| `test_controller.py` | 선택 사항: 코드 회귀 검사 |

실행에 필요한 앞의 세 파일을 Pi의 같은 폴더에 놓습니다. 이 문서는 `~/RC_Controller3` 폴더를 기준으로 설명합니다. 첨부 TXT/PDF/PNG는 참고 자료이며 실행할 때 반드시 복사할 필요는 없습니다.

- 서버 실행 대상: Raspberry Pi 4 + Linux + Python 3 + pigpiod.
- 노트북: Pi와 같은 Wi-Fi에 연결된 Windows 또는 macOS, 일반 웹 브라우저.
- LiDAR: Pi에서 단독 실행이 되는 `ultra_simple`을 사용자가 준비합니다.
- Python 외부 의존성은 `pigpio` 하나입니다. 노트북에는 Python을 설치하지 않아도 됩니다.
- 기존 Pi 설정이 있다면 먼저 보관하고, 확인한 핀·모터 방향·보정값을 유지하세요.

## 2. 최초 한 번 설치

아래 명령은 **노트북이 아니라 Pi 터미널**에서 실행합니다.

```bash
sudo apt update
sudo apt install -y pigpio python3-pigpio coreutils util-linux
sudo usermod -aG dialout "$USER"
```

`dialout` 그룹 변경 후 로그아웃했다가 다시 로그인합니다. 이 그룹은 LiDAR USB 직렬 포트 접근에 사용됩니다. Pi OS 저장소에 pigpio가 없다면 오류를 무시하지 말고 [pigpio 설치 안내](https://github.com/joan2937/pigpio)를 확인하세요. 현재 코드는 Pi 5용이 아닙니다.

배선은 [GPIO 배선과 전원](#10-gpio-배선과-전원)을 확인합니다. GPIO 14/15, 9/10, 4를 사용하므로 Serial login shell, Serial hardware, SPI, 1-Wire가 해당 핀을 점유하지 않게 설정하세요.

## 3. config.json 설정

Pi에서 파일을 엽니다.

```bash
cd ~/RC_Controller3
nano config.json
```

**아래 표에 나온 기존 항목의 값만 수정합니다. 파일 전체를 일부 예시로 덮어쓰지 마세요.** JSON의 쉼표와 중괄호를 유지하고 주석을 넣지 않습니다.

| 위치 | 사용할 값 | 의미 |
|---|---|---|
| 최상위 `hardware_confirmed` | 배선·전원 확인 후 `true` | GPIO 신호 전압과 전원 연결을 확인했다는 표시 |
| `control` 안의 `mode` | `"pwm"` | 엔코더 없이 PWM 직접 제어 |
| `servo` 안의 `calibrated` | 서보 중립·기구 범위 확인 후 `true` | 서보가 허용 범위에서 움직임을 확인했다는 표시 |
| 모터 각각의 `encoder_confirmed` | PWM에서는 `false` 가능 | RPM PI 모드를 사용할 때만 보정 후 `true` 필요 |
| `lidar` 안의 `enabled` | `true` | 외부 LiDAR 출력 수집 |
| `lidar` 안의 `fifo` | `"/tmp/rc_controller_lidar.fifo"` | LiDAR 출력을 전달받는 파이프 경로 |
| `mt6701` 안의 `ppr` | 미확인이면 `null` | 확인 전에는 각도 대신 카운트·Z 정보 표시 |

`hardware_confirmed`와 `servo.calibrated`의 기본값은 `false`입니다. 확인 전에도 센서 수집과 화면 접속은 가능하지만 구동 활성화는 막힙니다. 이 항목을 true로 바꾼다고 자동으로 보정되는 것은 아닙니다.

**LiDAR 실행 파일 경로는 config.json에 넣지 않습니다.** `fifo`는 `ultra_simple`의 위치가 아니라 출력 전달 통로입니다. `/tmp/rc_controller_lidar.fifo` 그대로 사용하세요. 실행 파일 경로와 USB 포트·baud는 두 번째 터미널의 명령에 지정합니다.

저장: **Ctrl+O → Enter → Ctrl+X**. 설정 변경은 서버를 재시작해야 적용됩니다.

현재 기본 범위:

| 항목 | 설정 상한 | 화면 연결 직후 값 |
|---|---|---|
| PWM 출력 | 100% (`max_duty: 1.0`) | 35% |
| 목표 RPM | 320 (`max_rpm: 320`) | 20 RPM |
| 서보 명령각 | 중심 기준 ±90° (`max_angle_deg: 90`) | 키보드 조향 10° |
| 서보 펄스 | 500–2500µs | 구동 비활성 상태에서는 펄스 중지 |

320 RPM은 첨부 모터의 무부하 속도이며 정격은 280 RPM입니다. 부하 상태에서 320 RPM에 도달한다는 뜻은 아닙니다. ±90°는 서보축 명령 범위로, 실제 차량 링크가 허용하는 바퀴 조향 범위를 보장하지 않습니다. 기구에 맞게 제한하세요.

## 4. 매번 실행하는 순서

### 4-1. Pi 터미널 1 — 제어 서버

```bash
cd ~/RC_Controller3
python3 controller.py --check-config
sudo pigpiod -l -s 5
python3 controller.py
```

pigpiod가 이미 실행 중이면 `sudo pigpiod -l -s 5`는 생략합니다. `-l`은 pigpio 접속을 Pi 내부로 제한하고 `-s 5`는 GPIO 샘플링 간격 5µs를 지정합니다. `controller.py`는 sudo 없이 실행합니다.

이 터미널은 계속 켜둡니다. 서버가 정상적으로 LiDAR 수집을 준비하면 `/tmp/rc_controller_lidar.fifo`를 만듭니다. 웹 화면에서 LiDAR 준비 오류가 없는지 확인하세요.

### 4-2. Pi 터미널 2 — 평소 LiDAR 명령에 출력 전달 붙이기

**평소 `ultra_simple`을 실행하던 폴더로 이동합니다.** 기존 명령이 다음과 같았다면:

```bash
./ultra_simple --channel --serial /dev/ttyUSB0 460800
```

앞에 `stdbuf -oL -eL`, 뒤에 `> /tmp/rc_controller_lidar.fifo 2>&1`을 붙여 실행합니다.

```bash
stdbuf -oL -eL ./ultra_simple --channel --serial /dev/ttyUSB0 460800 > /tmp/rc_controller_lidar.fifo 2>&1
```

- `./ultra_simple` 부분은 평소 사용하던 상대 경로나 절대 경로를 그대로 사용합니다.
- USB 포트가 `/dev/ttyUSB0`이 아니라면 실제 포트를 사용합니다. C1의 baud는 `460800`입니다.
- **서버를 먼저 실행해야 합니다.** 처음 사용하는 경로에서 서버가 준비되기 전에 위 명령을 실행하면 FIFO 대신 일반 파일이 만들어질 수 있습니다.
- 준비 여부를 확인하려면 실행 전에 `test -p /tmp/rc_controller_lidar.fifo && echo FIFO_OK`를 입력합니다. `FIFO_OK`가 보인 뒤 실행하세요. 이전 실행의 FIFO가 남아 있을 수도 있으므로 이 확인만으로 서버가 켜져 있다고 판단하지 마세요.
- 서버와 SDK는 같은 사용자 계정으로 실행합니다. 단독 실행 중인 SDK가 있다면 Ctrl+C로 종료하고 위 명령 하나만 실행합니다.
- **터미널에 거리 숫자가 안 나오는 것은 정상입니다.** 출력이 제어 프로그램으로 전달되어 웹 화면에 표시됩니다.

웹의 LiDAR 영역에 수신 줄 수, 측정 점 수, SDK 진단 문구가 표시됩니다. 실행 파일을 별도로 설치하거나 자동 실행하는 기능은 없습니다.

### 4-3. 노트북 — 웹 접속

Pi IP를 모르면 Pi 터미널에서 확인합니다.

```bash
hostname -I
```

같은 Wi-Fi에 연결된 노트북 브라우저에서 다음 주소를 엽니다.

```text
http://Pi의IP주소:8080
```

예: Pi IP가 `192.168.0.25`이면 `http://192.168.0.25:8080`입니다. 접속 키 없이 자동 연결됩니다. 예전 화면이 남으면 Windows에서는 Ctrl+F5, macOS에서는 브라우저의 강력 새로고침을 사용하세요.

접속 인증과 HTTPS는 없습니다. 주소에 접근 가능한 다른 장치도 조종 요청을 보낼 수 있으므로 신뢰하는 사설 Wi-Fi에서 사용하고 외부 포트 포워딩은 하지 마세요.

## 5. 조종과 종료

첫 동작은 바퀴를 바닥에서 띄워 낮은 출력으로 확인합니다.

1. 화면에서 **PWM 직접 제어**를 선택합니다.
2. **구동 활성화**를 누릅니다.
3. Shift를 누른 상태에서 W/S로 전후진, A/D로 조향합니다.

| 입력/버튼 | 동작 |
|---|---|
| Shift + W 또는 ↑ | 전진 |
| Shift + S 또는 ↓ | 후진 |
| Shift + A 또는 ← | 왼쪽 조향 명령 |
| Shift + D 또는 → | 오른쪽 조향 명령 |
| 주행 키 또는 Shift 해제 | 구동 목표 및 출력 0 |
| Space / 비상정지 | 정지 상태 잠금 |
| 정지 잠금 해제 | 잠금만 해제; 다시 구동 활성화 필요 |
| 구동 해제 | 모터 정지, 서보 펄스 중지 |
| MT 상대 영점 | 구동 해제 상태에서 현재 MT 위치를 상대각 0°로 지정 |

PWM 모드에서 출력 슬라이더는 직접 구동할 PWM을 정합니다. RPM 슬라이더는 비활성입니다. 슬라이더만 움직여서는 주행하지 않습니다. 조향 슬라이더는 A/D를 누를 때의 명령각 크기를 정합니다.

입력칸·슬라이더·선택 상자에 포커스가 있으면 방향키가 입력 조작에 쓰일 수 있습니다. 이때는 화면의 빈 곳을 클릭한 뒤 조종하세요. 창 이탈, 탭 숨김, 통신 끊김 후에는 다시 구동 활성화해야 합니다.

**종료 순서:**

1. 웹 화면에서 **구동 해제**.
2. **LiDAR 터미널(터미널 2)에서 Ctrl+C**.
3. **제어 서버 터미널(터미널 1)에서 Ctrl+C**.

차량 비상정지 버튼이나 서버 종료는 별도 터미널의 LiDAR SDK를 종료하지 않습니다. LiDAR 회전은 SDK 터미널에서 직접 중지합니다. 서버를 재시작한 경우 SDK 출력 명령도 다시 실행하세요.

## 6. RPM 모드, 보정 및 센서 값

### PWM과 RPM PI의 차이

| 항목 | PWM 직접 제어 | RPM PI 제어 |
|---|---|---|
| 기본 사용 | 엔코더 없이 우선 주행 | 엔코더 피드백으로 속도 유지 |
| 출력 슬라이더 | 직접 PWM 값 | PI가 사용할 PWM 상한 |
| 속도 슬라이더 | 사용하지 않음 | 목표 출력축 RPM |
| 엔코더 확인 플래그 | false여도 가능 | 두 모터 모두 true 필요 |
| 엔코더 무응답 정지 | 적용하지 않음 | 적용 |
| 실제 속도 유지 | 보장하지 않음 | 보정·전원·부하·제어 범위 내에서 조절 |

구동 해제 후 화면에서 모드를 바꿀 수 있습니다. 화면에서 바꾼 모드는 현재 실행에만 적용됩니다. 기본 모드를 바꾸려면 `control.mode`를 `"pwm"` 또는 `"rpm"`으로 저장하고 재시작합니다. 두 후륜에는 동일한 목표값을 주며, 차동 속도 보정은 구현하지 않았습니다.

### 모터 엔코더와 거리

모터 출력축을 정확히 한 바퀴 돌려 x4 카운트를 확인하고 `encoder_counts_per_rev`를 설정합니다. 기본 1320은 첨부 11라인·감속비 30을 `11 × 4 × 30`으로 해석한 추정값입니다.

전진에 해당하는 바퀴 회전에서 양의 카운트가 되도록 `encoder_sign`을 ±1로 조정합니다. 실제 모터 구동 방향은 별도의 `motor_sign`으로 맞춥니다. `wheel_ratio`는 바퀴 1회전당 모터 출력축 회전 수이며 직결이면 1입니다. 바퀴 지름은 사용자 확인값 75mm입니다.

```text
출력축 RPM = Δ카운트 / 출력축 counts_per_rev / Δ초 × 60
바퀴 RPM = 출력축 RPM / wheel_ratio
거리(m) = 카운트 / counts_per_rev / wheel_ratio × π × 바퀴 지름(mm) / 1000
```

누적 경로에는 전후진 회전을 모두 더하며 화면의 차량 거리는 두 바퀴 평균입니다. 실제 차체 위치나 미끄러짐을 보정한 거리는 아닙니다. 서버 재시작 시 카운트는 0부터 시작합니다.

신호가 검출되지 않은 RPM은 `—`로 표시합니다. PWM 주행 중 신호가 오래 없으면 거리 누락을 표시하고 전체 누적 거리 값도 숨깁니다. 기록의 `encoder_distance_m`와 `signed_distance_m`에는 수신한 카운트에 기반한 부분 거리만 남습니다. 단선과 실제 정지를 엔코더만으로 완벽하게 구분할 수는 없습니다.

### 서보, MT6701, 초음파

서보 중심값은 기본 1500µs, 각도 환산은 약 11.111111µs/도입니다. 기구 중립이 다르면 `center_us`를 보정하고 `min_us`, `max_us`, `max_angle_deg`를 함께 맞춥니다. 방향은 `servo.sign`으로 바꿉니다. `command_deg`는 명령각이며 실제각인 `measured_deg`는 null입니다.

MT6701은 PPR 미확인 상태이므로 `ppr: null`을 유지합니다. A/B 카운트와 Z 검출 횟수는 수집하지만 각도는 표시하지 않습니다. PPR을 확인하면 1~1024 정수로 입력합니다. 예를 들어 256 PPR은 x4 기준 1024 counts/rev입니다.

```text
MT 상대각 = (현재 카운트 - 영점 카운트) / (4 × PPR) × 360°
```

ABZ만으로 전원 투입 직후 절대각을 알 수 없습니다. 수동 영점은 현재 위치 기준이며, Z 검출은 영점을 덮어쓰지 않고 당시 카운트만 기록합니다. EEPROM 설정 변경이나 MT 각도를 실제 조향각으로 매핑하는 기능은 없습니다.

US-100은 뒤쪽 점퍼를 제거한 Trig/Echo 모드로 사용합니다. 4개를 약 65ms 간격으로 순차 발사하여 센서당 약 3.8Hz로 측정합니다. 응답 제한 30ms, 기본 유효 거리 2cm~4m이며 실패하거나 1초 이상 지난 값은 `—`입니다.

```text
초음파 거리(m) = Echo 폭(µs) × 343 / 2,000,000
```

음속 환산값은 설정에서 바꿀 수 있습니다. Trig/Echo 모드에서 별도 온도 데이터는 수집하지 않습니다. 초음파는 거리 수집만 하며 자동 장애물 제동에는 사용하지 않습니다.

### 주요 설정 요약

| 설정 | 기본값 / 의미 |
|---|---|
| `control.hz` | 제어 루프 목표 50Hz |
| `control.ramp_pwm_s` | 0.5: 초당 PWM 0.5 증가, 즉 50%p/초 |
| `control.ramp_rpm_s` | 목표속도 증가 80 RPM/초 |
| `control.kp`, `ki` | PI 비례 0.003, 적분 0.006 |
| `control.rpm_filter_s` | RPM 필터 시정수 0.12초 |
| `control.command_timeout_s` | 조종 신호 제한 0.4초 |
| `control.stall_timeout_s` | RPM 모드 정지/무응답 판정 0.7초 |
| `control.reversal_pause_s` | 반전 전 출력 0 대기 0.3초 |
| `servo.rate_deg_s` | 서보 명령각 변화 60도/초 |
| `lidar.angle_offset_deg` | 설치 방향 보정, 기본 0° |
| `lidar.stale_s` | 1초 이상 새 스캔이 없으면 점군 숨김 |

PI는 PWM을 0~1 비율로 계산합니다. kp 단위는 PWM/RPM, ki 단위는 PWM/(RPM·초)입니다. 출력 포화 시 적분을 제한하고 정지 시 초기화합니다. 실제 장비에 맞춘 튜닝은 필요합니다. PWM은 1kHz, 1000단계입니다.

## 7. LiDAR 상세와 문제 해결

### 출력 전달이 늦거나 숫자가 수집되지 않을 때

평소 단독 실행은 정상인데 `stdbuf` 방식에서 수신이 늦다면 현재 SDK 실행을 Ctrl+C로 중지하고 다음 방식을 대신 사용합니다. 평소 실행하던 폴더에서 입력합니다.

```bash
script -q -c './ultra_simple --channel --serial /dev/ttyUSB0 460800' /dev/null > /tmp/rc_controller_lidar.fifo 2>&1
```

이 방식은 SDK에 터미널을 제공하여 콘솔 출력 버퍼 동작 차이를 줄입니다. 서버를 먼저 실행하고, 같은 직렬 포트를 사용하는 SDK는 하나만 실행하세요.

### FIFO 경로 오류

`fifo`는 이름 있는 파이프여야 합니다. 서버는 같은 사용자 소유의 FIFO만 읽으며 일반 파일이나 링크를 덮어쓰지 않습니다. 웹에 경로 오류가 표시되면 `config.json`의 fifo를 사용하지 않는 새 경로(예: `/tmp/rc_controller_lidar2.fifo`)로 바꾼 뒤 서버를 재시작하세요. LiDAR 명령 끝의 출력 경로도 동일하게 바꿉니다.

### 점군 처리 방식

`theta / Dist / Q` 텍스트에서 거리 mm를 m로 바꿉니다. 정렬된 각도의 360→0 경계로 회전을 나누고 0.5° 구간당 최근 한 점, 최대 720점을 보냅니다. 기본적으로 0.05~12m와 품질 1 이상만 표시합니다. 출력 연결이 끊기면 기존 점군을 지우고 다시 들어오는 출력을 기다립니다.

지도는 현재 차량 중심의 2D 스캔입니다. 전방 0°, 오른쪽 90°이며 위치 추정이나 SLAM 누적 지도는 구현하지 않았습니다.

| 증상 | 확인할 내용 |
|---|---|
| LiDAR 수신 0줄 | 서버가 켜졌는지, enabled=true인지, 출력 경로가 fifo와 일치하는지 |
| 수신 줄은 증가하지만 측정 0개 | 화면 SDK 진단 문구, theta/Dist/Q 형식, 실제 USB 포트와 460800 baud |
| SDK가 멈춘 것처럼 보임 | 서버 수신 준비 여부; 출력이 웹으로 전달되어 터미널은 조용할 수 있음 |
| USB 포트 권한 오류 | 같은 사용자로 실행, dialout 가입 후 재로그인 |
| 구동 활성화 불가 | 화면의 차단 이유, hardware_confirmed와 servo.calibrated |
| PWM에서 엔코더 확인 요구 | 실제 화면 모드가 PWM인지 확인 |
| RPM 모드가 무응답으로 정지 | 엔코더 배선·계수·부호·보정 확인 또는 PWM 모드 선택 |
| 모터 방향 반대 | motor_sign, encoder_sign을 각각 확인 |
| 초음파 응답 없음 | 점퍼 제거, Trig/Echo, 전원·신호 전압과 공통 GND |
| 웹 접속 불가 | Pi IP/8080, 같은 Wi-Fi, 방화벽과 공유기 AP isolation |

## 8. 데이터 기록, 코드 구조 및 안전 기능

### 기록

기본적으로 Pi의 `logs/telemetry.jsonl`에 2Hz로 저장합니다. 각 줄은 UTC 시각, 제어 모드, 모터·엔코더·거리·서보·MT·초음파·LiDAR 스냅샷이 담긴 JSON입니다. LiDAR는 최신 스캔을 일정 간격으로 기록하며 모든 원시 샘플을 보존하는 것은 아닙니다.

약 10MB마다 파일을 회전하고 `.1`~`.3`을 보관하여 총 약 40MB로 제한합니다. 오래된 기록은 순환 삭제됩니다. `recording.include_lidar=false`로 점군 기록을 생략하고 `recording.enabled=false`로 기록을 끌 수 있습니다. 쓰기 실패는 화면에 표시합니다.

### 구조와 통신

| 코드 구성 | 역할 |
|---|---|
| `Quadrature`, `PI` | A/B x4 계수와 PI 제어 |
| `Hardware` | pigpio 연결, PWM/서보, daemon 안전 타이머 |
| `Ultrasound`, `Lidar` | 순차 Echo 측정과 외부 SDK FIFO 수집 |
| `Controller` | 모드, 조종 세션, 출력 계산, 정지 조건 |
| `Runtime` | 수집 스레드, 통합 상태, 기록, 종료 |
| `Server`, `Handler` | 웹 화면, HTTP 명령, SSE 데이터 전송 |

Wi-Fi에서 TCP 기반 HTTP/SSE를 사용합니다. 명령은 약 10Hz, 상태 전송은 약 10Hz입니다. 별도 노트북 앱이나 외부 CDN이 필요 없습니다. Linux/Python 부하에 따라 실제 주기는 달라질 수 있습니다.

API는 GET `/api/state`, GET `/api/events`, POST `/api/command`입니다. 접속 키는 없지만 이전 구동 세션이나 오래된 명령을 거부하기 위한 내부 lease와 seq는 사용합니다. 한 탭만 조종할 수 있고 모니터링 스트림은 최대 4개입니다. Origin 검사는 네트워크 접속 인증을 대신하지 않습니다.

### 정지 기능과 실제 사용 한계

두 모드 모두 통신 신호 0.4초 단절, 제어 루프 0.15초 이상 지연, 비상정지에 반응합니다. Python heartbeat 중단 시 pigpiod 스크립트가 약 0.5초 뒤 모터 PWM과 서보 펄스를 0으로 설정합니다.

RPM 모드는 엔코더 무응답·방향 불일치·과속을 감지하여 정지합니다. PWM 모드는 요청대로 엔코더 피드백 없이 주행하며 이 엔코더 기반 정지는 적용하지 않습니다. 방향 반전 시 PWM은 시간 대기, RPM은 추가로 낮은 측정 RPM을 확인합니다.

MDD3A의 A/B 모두 LOW는 Brake입니다. 차량이 즉시 물리적으로 정지한다는 뜻은 아니며 서보 펄스 중지 시 유지력이 사라질 수 있습니다. OS 전체나 pigpiod 자체 정지, 전기적 고장은 소프트웨어만으로 확실히 차단할 수 없으므로 실제 모터 전원 차단 스위치가 필요합니다.

SIGKILL처럼 정리 없이 종료하면 daemon 스크립트가 남을 수 있습니다. 이 경우 모터 전원을 끄고 pigpiod를 재시작합니다. 동일 폴더의 중복 서버는 파일 잠금으로 거부하지만 다른 폴더의 복사본까지 막지는 않으므로 GPIO 제어 프로그램은 하나만 실행하세요.

현재 코드에 대한 회귀 검사 36개와 웹 코드 검사를 수행했습니다. 실제 Pi GPIO, Linux FIFO와 LiDAR 장치 연결, 모터 부하와 정지 거리는 이 개발 환경에서 검증하지 않았습니다. 검사를 직접 실행하려면:

```bash
python3 controller.py --check-config
python3 -m unittest -v test_controller.py
```

이 명령은 실제 배선이나 기구 보정을 대신하지 않습니다. 향후 개선 사항은 전류·배터리 전압 측정, 실제 조향각 보정, 차동 속도 보정, 고속 엔코더 수집 검증, 위치 추정·SLAM입니다.

## 9. 첨부 자료와 확인된 사양

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

## 10. GPIO 배선과 전원

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


## 11. 추가 참고 자료

- [SLAMTEC ultra_simple 소스](https://github.com/Slamtec/rplidar_sdk/blob/master/app/ultra_simple/main.cpp)
- [pigpio 공식 저장소](https://github.com/joan2937/pigpio)
- [MT6701 데이터시트](https://www.magntek.com.cn/upload/pdf/202407/MT6701_Rev.1.8.pdf)
- [US-100 설명](https://www.adafruit.com/product/4019)
- [Raspberry Pi GPIO 문서](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html)
