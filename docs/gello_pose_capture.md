# GELLO 조종과 데이터셋 기록 동시 실행

`zeus_gello.py`와 기존 `pose_query_server.py`는 둘 다 12350을 바인딩했다.
먼저 실행한 서버가 포트를 차지하면 다른 서버는 시작할 수 없고, PC가 잘못된
서버에 붙으면 `op/ok`와 `command/status` 프로토콜 차이로 오류가 난다.

수정 후 연결은 다음과 같다.

| 역할 | 로봇 SSH에서 실행 | PC 클라이언트 | 포트 |
| --- | --- | --- | --- |
| GELLO 조종 | `python zeus_gello.py` | `gello/gello_zeus_real_teleop.py` | 12350 |
| 자세 조회·기록 | `python pose_server.py` | `capture/record_dataset.py` | 12352 |

`pose_server.py`는 `i611shm.shm_read()`만 사용한다. `rb.open()`, tool 변경,
이동, 정지, 그리퍼 명령을 호출하지 않는다. 레코더가 종료되어도 GELLO 연결은
유지된다. `zeus_gello.py`를 사용할 때는 `zeus_jog_onboard.py` 등 다른
`rb.open()` 조작 프로그램을 함께 실행하지 않는다.

## 배포

로컬 저장소 루트에서 실행한다. 원격 대상 경로는 실제 파일 위치에 맞춘다.
기존 파일은 교체 전에 백업한다.

```bash
scp capture/robot/pose_server.py capture/robot/pose_query_server.py \
    i611usr@192.168.0.23:~/unknown/
```

`pose_server.py`는 단독 배포할 수 있다. 기존 `python pose_query_server.py`
명령을 유지하려면 두 파일을 같은 디렉터리에 복사한다. 수정 전 자세 서버는
파일을 교체해도 실행 중인 코드가 바뀌지 않으므로 해당 자세 서버를 종료한 뒤
새 파일로 다시 시작한다.

## 실행 순서

1. 로봇 SSH 터미널 A에서 기존 `zeus_gello.py`를 실행한다 (12350).
2. 로봇 SSH 터미널 B에서 업로드한 파일을 실행한다.

   ```bash
   cd ~/unknown
   python pose_server.py
   ```

3. 로컬 `multicam-calibration-simulation/capture`에서 연결만 확인한다.
   레코더를 이미 실행 중이라면 먼저 종료한다. 자세 서버는 한 번에 레코더 한
   연결을 처리하므로 진단도 레코더를 시작하기 전에 실행한다.

   ```bash
   python record_dataset.py --check-robot
   ```

4. 로컬 `rb-calibration-marker-experiment`에서 기존 GELLO 명령을 실행한다.

   ```bash
   /home/sprout/anaconda3/envs/robot_multicam/bin/python gello/gello_zeus_real_teleop.py
   ```

5. 별도 로컬 `multicam-calibration-simulation/capture` 터미널에서 기록한다.

   ```bash
   python record_dataset.py
   ```

GELLO 터미널의 Enter는 조종 연결/해제이고, **레코더 터미널의 Enter는 촬영과
저장**이다. 로봇을 멈춘 뒤 기록한다. `s`는 자세 확인, `q`는 레코더 종료다.
현재 레코더는 일정 주기로 자동 촬영하는 방식이 아니라 Enter 한 번마다 자세와
이미지를 자동 저장하는 방식이다.

공유 메모리 자세는 현재 tool 기준이다. 확인한 로컬 `zeus_gello.py`는
`settool(1, 0, 0, 0, 0, 0, 0)` 후 `changetool(1)`을 호출한다. 실제 로봇에서도
같은 버전과 tool 설정을 사용하는지 확인해야 한다. 관찰 서버가 tool을 바꾸지는 않는다.

## 로컬 회귀 검사

```bash
env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q \
    tests/test_pose_server.py tests/test_capture_storage.py
```

Python 2.7 실제 자세 서버와 실제 Python 3 레코더를 TCP로 연결한다. 하드웨어
대신 가짜 공유 메모리와 GELLO 응답기를 사용해 포트 공존, 저장/재접속, 잘못된
제어 요청 거부를 확인한다. 실제 로봇의 동시 조종 검증과는 구분한다.
