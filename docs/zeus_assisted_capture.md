# ZEUS 자동 유효 포즈 탐색 및 촬영

## 오늘의 cam1 · cam3 자동 수집

사람이 두 카메라의 공통 시야에 그리퍼 보드를 대략 배치한다. 이후 `auto 30`이
카메라 월드 위치 없이 작은 이동의 관측 결과로 탐색 방향을 선택한다.
cam2는 손목 카메라로 제외한다. 다른 날에는 `(cam0, cam1)`, `(cam0, cam3)`도
같은 방식으로 각각 실행한다. 카메라 또는 시작 배치가 바뀌면 새 세션을 만든다.

이번 추가 기능은 PC의 `assisted_search.py`와 `assisted_capture.py`에 있다.
로봇 서버 파일을 변경하지 않았으므로 SCP 재전송은 필요 없다.
로봇 live 서버와 SSH 터널을 유지하고, 기존 촬영 PC 프로그램만 `quit` 후 다시 실행한다.
RealSense Viewer에서 같은 카메라의 스트림을 동시에 열지 않는다.

```bash
cd '/home/sprout/Desktop/**jieun-bomin/multicam-calibration-simulation'
python capture/assisted_capture.py --cameras cam1 cam3 \
  --width 1280 --height 720 --fps 6 --startup-stagger-s 5 \
  --min-corners 12 --detection-scale 3
```

작은 마커의 검출 누락을 줄이기 위해 assisted 프로그램은 기본 3배 확대 영상에서
ChArUco 코너를 검출한다. 확대는 새 영상 정보를 만드는 것이 아니라 기존 검출기의
픽셀 크기 조건을 바꾸는 전처리다. `--detection-scale 1`로 기존 방식과 비교할 수 있다.
코너는 OpenCV resize 픽셀 중심 규약에 맞춰 원본 좌표로 환산한 후 기존 K/D로
포즈를 계산한다. 재투영 오차와 가장자리 여유도 원본 픽셀 단위이며, 저장 PNG는
원본 1280×720이다. 배율과 좌표 기준은 capture_config 및 각 검출 결과에 기록한다.
기존 JSON/영상 및 다른 스크립트의 BoardDetector 기본값(1배)은 변경하지 않는다.

이미 설정한 anchor와 현재 위치가 유효한 오늘의 세션에서는:

```text
state
arm
auto 30
```

`state`의 `anchor`가 null이거나 사람이 시작 배치를 새로 했다면, 정지 및 disarm
상태에서 `anchor`를 먼저 지정한다. 탐색 중 좋은 위치를 찾았다고 anchor를 재설정하지 않는다.
`state`에 FAULT가 있으면 먼저 원인과 컨트롤러를 확인한다. 자동 수집은 이를 해제하지 않는다.

### 자동 처리

1. 현재 위치에서 새 이미지 5쌍을 저장하고 판정한다. 카메라마다 적어도 일부 코너가
   검출되어야 탐색을 시작한다. 기본 통과 조건은 12코너이며 `--min-corners 10`으로
   명시적으로 바꿀 수 있다. 실행 도중 기준을 자동으로 낮추지는 않는다.
2. 5쌍 중 4쌍 이상 두 카메라가 동시에 모든 품질/로봇 정지/수신 시각 차이 조건을
   통과하고, 통과 프레임 간 카메라 보드 포즈 변화가 5mm / 2deg 이하여야 유효 후보다.
   동일 frame sequence를 중복 계산하지 않는다. 선택한 1쌍과 나머지 검증 영상 모두 보존한다.
3. 위치 ±1mm, 회전 ±0.75deg와 복합 회전 후보를 생성한다. 두 카메라 중 낮은 품질,
   동시 통과율, 코너 수, 기존 수집 자세와의 차이를 이용해 다음 후보를 선택한다.
   위치가 좋아지면 그 주변을 확장하되 원래 anchor와 관절 제한은 유지한다.
4. 후보 사이의 이동은 실제로 방문한 경로 트리를 따라 되짚는다. 이동 명령은 더 작게
   나누고 매번 기존 서버의 IK/관절 검사를 받는다. 자동 탐색 자체의 기본 범위는
   원래 anchor에서 합성 거리 10mm / 회전 6deg다. 이 범위는 충돌 안전 보장이 아니다.
5. 기존 채택 자세와 위치 차이 1.5mm 미만 AND 회전 차이 1deg 미만이면 중복으로 제외한다.
   목표 개수뿐 아니라 최대 자세 간 회전 3deg 이상 및 회전벡터 분포의 두 번째 특이값
   1deg 이상을 함께 요구한다. 개수에 도달해도 이 검사를 통과할 때까지 계속한다.
   이는 작은 회전/단일 축 데이터의 선별 진단이며 Shah 정확도를 보장하는 수치는 아니다.
6. 기본 120후보 또는 25분 이내에 탐색한다. 목표를 달성하지 못하면 `incomplete`로
   기록하고 모은 유효 데이터를 보존한다. 종료/취소/오류 시 disarm을 요청한다.
   제조사 한계, 충돌 검사, 전역 최적 위치 탐색은 이 기능에서 보장하지 않는다.

`pause`는 진행 중인 짧은 이동 완료 후 탐색을 중단하고 disarm한다. 즉시 정지 요청은
`stop` 또는 미리보기 X/Esc이며 필요할 때 사용한다. 두 카메라가 모두 코너를 잃거나
카메라 프레임이 유실되면 자동 복귀 없이 중단한다. SDK 이동 거절도 임의 재시도하지 않는다.
다시 `auto`를 실행하면 새 탐색 실행으로 기록한다. 같은 세션이라도 이전 실행 수량을
자동 합산하지 않는다. 물리적으로 로봇이 정지했는지는 현장에서 확인한다.

### 결과 파일

시작 시 출력한 세션 폴더에 다음 파일을 저장한다.

- `meta.json`: 실패·검증용 프레임을 포함한 모든 영상과 실제 로봇 자세.
- `valid_meta.json`: 수동 통과 촬영과 자동 다중 프레임 검증에서 채택한 촬영.
- `auto_valid_meta_<실행ID>.json`: **해당 자동 실행에서 채택한 다양한 자세만** 담은 입력.
  Shah 처리에는 이 선택 파일을 사용한다. 원래 이미지 상대 경로와 기존 스키마를 유지한다.
- `auto_report_<실행ID>.json`: complete/incomplete/paused/aborted, 카메라 쌍, 원래 anchor,
  채택 개수, 검증 기준, 회전 특이값과 범위. `complete`도 실제 보정 오차 검증을 대체하지 않는다.

자동 탐색의 평가용 단일 프레임은 그 순간 통과해도 다중 프레임 검증/중복 검사 전에는
유효 목록에 넣지 않는다. 가상 카메라 실행은 모든 유효 목록에서 제외한다.

아래 절은 기존 수동 조그 및 고정 12후보 순회 기능의 설명이다.

현재 사용 중인 `zeus_jog_onboard.py`는 변경하지 않는다. 새 실행기는 ZEUS
ZRA-0515P의 현재 플랜지 자세를 기준으로 작은 이동과 촬영을 수행한다.
실제 로봇에서 검증한 코드라는 뜻은 아니다. Python 2.7 호환성과 로컬 시뮬레이션을
검사했으며, 실제 SDK 호환성·정지 전달·기계의 이동은 현장에서 확인해야 한다.

## 구성

| 위치 | 파일 | 역할 |
|---|---|---|
| PC | `capture/assisted_capture.py` | 숫자 조종, 순회, 촬영 기록, 명령/통신 관리 |
| PC | `capture/assisted_camera.py` | 두 카메라 연속 취득, 검출, 미리보기 |
| PC·로봇 | `capture/robot/zeus_capture_core.py` | 회전 계산, 제한 검사, 후보 생성 |
| 로봇 | `capture/robot/zeus_capture_server.py` | SDK 제어권 하나, 이동과 상태 조회 |
| 로봇 | `capture/robot/zeus_stop_helper.py` | 별도 프로세스에서 컨트롤러 정지 요청 |

PC는 기존 `board_config.py`, `shah_capture_client.py`의 보드 검출·카메라 클래스·
변환 규약을 재사용한다. 로봇에는 위 세 개의 `zeus_*.py`만 전송하면 된다.
로봇 실행기는 `pose_query_server.py`의 상태 조회 역할도 포함한다.

## 확인된 설정

- 로봇: ZEUS ZRA-0515P.
- 로봇 보드: 가로 7칸 × 세로 5칸, 체커 17 mm, 마커 12 mm, 시작 ID 0.
- 기존 사전 `DICT_4X4_250`을 재사용. 코너 최대 24개.
- 로봇 제어/기록: tool 1, 오프셋 0, 플랜지 기준.
- 6개 숫자의 순서: `[x, y, z, rz, ry, rx]`, 위치 mm, 회전 deg.
- 저장 행렬: `T_destination_source`, 병진 metre, 회전 `Rz @ Ry @ Rx`.
- 회전 입력: 로봇 BASE 축을 기준으로 플랜지 위치를 유지하며 회전.
  보드 중심 고정 회전은 이번 버전에 포함하지 않는다.

## 1. 먼저 PC에서 장비 없는 시험

현재 프로젝트 폴더:

```bash
cd '/home/sprout/Desktop/**jieun-bomin/multicam-calibration-simulation'
```

PC Python은 현재 `robot_multicam` 환경의 Python을 사용하면 된다. 필요한 모듈은
NumPy, OpenCV의 aruco 기능, 실제 카메라 사용 시 pyrealsense2다.
이 PC에서 해당 모듈들이 로드되는 것을 확인했다. 새 환경에는 기존 의존성을 설치해야 한다.

터미널 A — 가상 로봇:

```bash
python2 capture/robot/zeus_capture_server.py --simulate
```

터미널 B — 가상 카메라와 PC 프로그램:

```bash
python capture/assisted_capture.py --simulate-cameras
```

미리보기 없이 전체 예제를 시험하려면 터미널 B에서:

```bash
python capture/assisted_capture.py --simulate-cameras --no-preview \
  --commands capture/assisted_simulation_commands.txt
```

명령 파일과 가상 카메라는 실제 로봇 서버에 연결했을 때 거절된다.
가상 데이터는 `simulation: true`로 표시하고 `valid_meta.json`에서 제외한다.
가상 영상은 소프트웨어 검증용이며 로봇 운동학/카메라 기하의 정답 데이터가 아니다.

## 2. 로봇 PC에 전송

아래는 **로컬 PC 터미널**에서 실행한다. SSH 비밀번호가 필요할 수 있다.
새 전용 폴더를 사용하며 기존 조그 파일은 덮어쓰지 않는다.

```bash
ssh i611usr@192.168.0.23 'mkdir -p ~/zeus_capture_v1'

scp -O capture/robot/zeus_capture_server.py \
  capture/robot/zeus_capture_core.py \
  capture/robot/zeus_stop_helper.py \
  i611usr@192.168.0.23:~/zeus_capture_v1/
```

로봇에 접속한 뒤, **이동 없이 API와 상태 조회부터 확인**한다:

```bash
python ~/zeus_capture_v1/zeus_capture_server.py \
  --sdk-dir /home/i611usr --check-sdk
```

출력은 실제 로드한 SDK 경로, 필수 API 누락 여부, 제어 관리자 조회 결과,
현재 자세/관절/오류 상태다. 이 모드에서는 `rb.open()`과 이동을 호출하지 않는다.
`--sdk-dir`은 실제 `i611_MCS.py` 폴더다. 기존 조그와 같은 Python을 사용한다.
설치 위치가 다르면 그 경로로 바꾼다. SDK가 시스템에 설치돼 있다면 이 옵션을 생략해도 된다.
출력 오류를 해결한 후 진행한다. 체크 성공은 실제 정지 성능을 검증했다는 뜻이 아니다.

## 3. 처음에는 상태 조회 모드로 연결

로봇 SSH 터미널:

```bash
python ~/zeus_capture_v1/zeus_capture_server.py --sdk-dir /home/i611usr
```

기본 모드는 `readonly`다. `rb.open()` 없이 공유 메모리만 읽으며 이동 명령은 거절한다.
이 모드는 기존 조그와 함께 진단할 수 있다. 현재 tool을 확정하지 않으므로
촬영해도 `tool_verified: false`이고 정식 분석용 선택에서는 제외한다.

별도의 **로컬 PC 터미널**에서 SSH 터널을 열고 유지한다:

```bash
ssh -N -o ExitOnForwardFailure=yes \
  -o ServerAliveInterval=5 -o ServerAliveCountMax=2 \
  -L 12351:127.0.0.1:12351 i611usr@192.168.0.23
```

서버는 로봇 PC의 `127.0.0.1`만 수신한다. PC도 `127.0.0.1:12351`로 접속한다.
SSH가 인증과 전송 보호를 담당한다. 로봇 IP를 PC 프로그램의 `--robot-host`로
넣는 직접 연결 방식이 아니다.

다른 **로컬 PC 터미널**에서 실제 두 카메라를 명시한다:

```bash
python capture/assisted_capture.py --cameras cam0 cam1
```

`cam0 cam1`은 예제다. 실제 사용하는 두 대가 `cam0 cam3`이면 그 이름으로 바꾼다.
각 이름은 `intrinsics/<이름>.npz`의 시리얼 번호에 연결된다.
기본 해상도는 현재 intrinsics와 같은 1280×720, 15 fps다.
다른 보드에 동일 마커 ID가 있어 영상에 함께 들어오면 구별이 어려우므로 가린다.

## 4. 실제 이동 모드

기존 조그로 최초 촬영 위치를 잡는다. 로봇이 멈춘 상태에서 기존 조그를 `q`로 종료하고,
다른 제어 프로그램도 종료한다. 상태 조회 모드 서버도 Ctrl+C로 종료한 뒤 새로 실행한다.

```bash
python ~/zeus_capture_v1/zeus_capture_server.py \
  --sdk-dir /home/i611usr --enable-motion
```

실행 시 정지용 보조 프로세스가 제어 관리자에 접속 가능한지 조회한다.
성공한 뒤에만 SDK를 열고 tool 1/오프셋 0과 낮은 동작 파라미터를 설정한다.
시작 자체에는 이동 명령이 없다. **기존 조그와 새 제어 실행기를 동시에 실행하지 않는다.**
이미 연결했던 PC 프로그램은 `quit` 후 다시 실행한다. SSH 터널은 유지해도 된다.

PC 프로그램의 `zeus>` 프롬프트에서 한 줄씩 입력한다:

```text
state
anchor
arm
jog x 1
capture x_plus_1
jog x -1
jog ry 0.5
capture tilted
jog ry -0.5
```

`anchor`는 현재 자세를 기준점으로 저장한다. `arm`은 이 서버의 이동 허용 상태를
설정하는 명령이며 서보 전원을 켜는 명령이 아니다. 처음에는 이 작은 이동과
정지 요청이 실제 장비에서 예상대로 동작하는지 확인한다. GUI의 X/Esc와 터미널의
`stop`은 소프트웨어 정지 요청이며 물리적 비상정지를 대체하지 않는다.

## 5. 제한된 주변 촬영

기준 자세에 돌아와 다음을 실행한다:

```text
plan 1 0.5
next
next
```

첫 `next`는 후보로 이동해 촬영한다. 두 번째는 기준 자세로 돌아온다.
계획은 X/Y/Z ±1 mm, Rz/Ry/Rx ±0.5 deg의 12개 후보와 각 후보 뒤의 기준 복귀,
총 24개 이동 항목이다. 매 목표는 기준 자세에서 계산한다.

현장에서 구간을 점검한 뒤 남은 계획을 이어 실행한다:

```text
run
suggest
disarm
quit
```

- `run` 시작 전 두 카메라에서 각각 코너 4개 이상이 보여야 한다.
- 코너 부족이나 재투영 오차가 큰 촬영도 저장한다.
- 두 카메라 모두 코너가 0개이거나 프레임을 얻지 못하면 순회를 중단한다.
  시야를 잃은 상태에서 복귀 경로가 괜찮다고 가정해 자동 이동하지 않는다.
- `pause`는 진행 중 이동이 끝난 뒤 나머지 순회를 중단한다.
- `stop`은 독립 정지 요청을 보내며 오류 상태를 유지한다. 원인 확인 후 서버를
  재시작하고 새 PC 세션을 연결한다. 컨트롤러 reset/서보 조작은 자동으로 하지 않는다.
- `suggest`는 두 카메라 동시 통과 및 평가 전용 제외 조건을 만족한 기록 중 최소 품질 점수가 가장 큰 기록을
  표시한다. 추천 자세로 자동 이동하지 않는다.

`plan/next/run`은 규칙적인 주변 탐색과 관측 결과 비교 기능이다. `auto`는 위의 피드백 탐색을 사용한다. 카메라 extrinsic으로
미래 화면을 예측하거나 모든 장애물을 피하는 계획기는 아니다. 기본 작은 탐색 범위는
초기 동작 확인용이며, 교정에 충분한 회전 다양성을 보장하지 않는다.

## 이동 검사와 한계

다음은 소프트웨어의 제한값이며 제조사 정격 또는 검증된 안전값이 아니다.

| 항목 | 첫 버전 제한 |
|---|---:|
| 한 명령의 위치 변화 | 합성 거리 2 mm 이내 |
| 한 명령의 방향 변화 | 회전각 1 deg 이내 |
| 기준 자세에서의 위치/방향 변화 | 20 mm / 10 deg 이내 |
| 기준 관절값에서의 변화 | 각 관절 10 deg 이내 |
| 한 이동의 관절 변화 | 각 관절 2 deg 이내 |
| 인접 IK 검사점 사이 관절 변화 | 각 관절 1 deg 이내 |
| 경로 분할 | 위치 0.5 mm / Euler 성분 0.25 deg 이하 |
| SDK 속도 파라미터 | lin_speed 5 mm/s, pose_speed 5%, jnt_speed 5%, override 10% |
| 가감속 시간 / blending | 1 s / 0 mm |
| heartbeat 끊김 / 작업 시간 제한 | 2.5 s / 15 s |

현재 posture·multiturn을 유지한 IK를 경로의 각 검사점에서 계산한다.
전체 검사에 통과한 다음 각 작은 구간을 동기 실행하고 실제 자세·관절과 비교한다.
회전 표현의 불연속, 큰 관절 응답, 상태 오류는 거절/중단한다.
이산 검사만으로 구간 사이의 모든 특이점을 증명하거나 충돌을 검증하지는 않는다.

제조사/설치 상태에서 확인한 절대 관절 한계를 제공하면 추가로 검사할 수 있다.
JSON에는 `verified: true`, 6개씩의 `min_deg`·`max_deg`, `margin_deg`를 기록하고
`--joint-limits /경로/joint_limits.json`으로 로봇 서버에 전달한다.
확인되지 않은 관절 한계값은 만들어 넣지 않는다. 옵션을 주지 않으면 상태에
`absolute_joint_limits_checked: false`, 항상 `collision_check: false`를 표시한다.

정지 보조 프로세스는 SDK의 `RobSys.cmd_stop()`을 사용한다. 메인 SDK 객체와
별도 프로세스여서 `rb.line()` 대기 중에도 요청을 전송할 수 있다. 요청 결과와
실제 정지 확인은 구분한다. 제어 관리자나 PC/로봇 OS가 고장 난 상황까지 보장하는
하드웨어 안전 장치는 아니다. 정지 실패 원문도 로봇 로그에 남긴다.

## 저장과 오류 조사

PC 기본 저장 폴더는 `datasets/assisted/session_<날짜>_<고유번호>/`이다.
매 시작마다 새 세션을 만들며 카메라가 움직인 세션을 이어 붙이지 않는다.

```text
meta.json          전체 촬영, 실패 이유, 목표/실제 자세, 전후 로봇 상태
valid_meta.json    품질·로봇 상태 조건에 통과한 실제 촬영만 선택
cam0/rgb_00000.png
cam1/rgb_00000.png
```

실제 프레임은 검출 결과와 관계없이 PNG로 저장한다. 각 카메라에 코너 좌표/ID,
재투영 오차, 검출점의 가장자리 여유, 선명도, 수신 시간, 검출 결과를 기록한다.
기본 품질 조건은 코너 12개 이상, 재투영 오차 1.5 px 이하, 검출점 여유 15 px 이상,
비공선 검출점과 유효한 양의 깊이다. 선명도는 수치만 기록하며 `--min-sharpness`를
검증한 후 지정하면 차단 조건으로 사용할 수 있다.

로봇 자세는 최소 0.6초 동안 여러 번 읽어 정지 상태를 확인하고, 이후 새로 수신한
카메라 프레임을 저장한다. 촬영 전후 실제 자세/관절 차이를 검사한다.
이는 정지 촬영용 소프트웨어 결합이며 하드웨어 동기화가 아니다. 수신 시간 차이와
장치 시간은 따로 기록한다. 명령한 목표값으로 실제 측정값을 대체하지 않는다.

로봇 로그는 `~/zeus_capture_v1/logs/executor_*.jsonl`에 저장된다.
최초 오류, 시스템 오류 번호, 자세·관절, 목표, IK 결과, 속도 파라미터,
명령 거절, 정지 요청 결과를 기록한다. 문제 발생 시 이 파일을 PC로 복사하면 된다.

## 검증 명령

```bash
python2 -m py_compile capture/robot/zeus_capture_core.py \
  capture/robot/zeus_capture_server.py capture/robot/zeus_stop_helper.py

python -m unittest discover -s tests -p test_zeus_assisted.py -v
python -m unittest discover -s tests -p test_zeus_assisted_integration.py -v
```

통합 테스트는 로컬 TCP 소켓을 사용하며 Python 2 가상 서버만 실행한다.
실제 로봇 이동·실제 카메라·제어 관리자 정지 동작은 이 테스트에 포함되지 않는다.
