# 실제 로봇 Shah 촬영 — 리그 구성, 준비 상태, 남은 작업

이 문서는 시뮬레이션 benchmark를 **실제 ZEUS 로봇 데이터**로 옮기기 위한 촬영 준비
내역이다. 작성일 2026-09-03. 아직 실촬영은 하지 않았고, 촬영 코드와 사전 분석까지
완료된 상태다.

다음 사람이 이어받을 수 있도록 **확인된 사실 / 측정한 수치 / 아직 모르는 것**을
구분해 적었다. 근거 없이 적은 값은 없으며, 각 수치의 출처는 §9에 있다.

---

## 1. 리그 구성 — 시뮬레이션과 다르다

실물 리그가 저장소 README의 전제와 다르므로 먼저 확인할 것.

| | README/시뮬레이션 | **실제 리그** |
| --- | --- | --- |
| 손목 카메라 | 1대 (eye-in-hand) | **없음** |
| 고정 카메라 | 3대 (eye-to-hand) | 3대, 테이블 위 |
| 보드 위치 | gripper에 고정 + base에 고정 | **로봇 팔 끝에 볼트 고정** |
| 총 카메라 | 4대 | 3대 |

즉 **전부 eye-to-hand**다. 로봇 팔 끝에 ChArUco 보드가 강체로 붙어 있고, 테이블에
고정된 카메라 3대가 그것을 본다. 이것이 Shah의 `AX = YB`가 상정하는 표준 구성이다.

```
T_base_gripper(k) @ X  ==  Y_i @ T_cam_i_board(k)

  X   = T_gripper_board  보드가 플랜지에 붙은 위치 — 세 카메라 공통, 하나뿐
  Y_i = T_base_cam_i     카메라 i의 base 기준 위치 — 주 출력
```

보드를 **쥐지 않고 볼트로 고정**한 이유가 여기 있다. 그리퍼로 물체를 쥐면 재파지마다
`X`가 바뀌어 `AX = YB`가 성립하지 않는다. 볼트 고정이라야 `X`가 상수로 보장된다.

> **`X`는 자로 재서 넣는 값이 아니다.** 보드가 플랜지 어디에 어떤 각도로 붙었는지는
> Shah가 추정한다. 필요한 조건은 촬영 내내 흔들리지 않는 것 하나뿐이다.

### 카메라 식별

`intrinsics/*.npz`의 `is_gripper` 플래그로 확인했다.

| 파일 | serial | `is_gripper` | 현재 역할 |
| --- | --- | --- | --- |
| cam0 | 314522062542 | `False` | 테이블 고정 |
| cam1 | 319522062138 | `False` | 테이블 고정 |
| cam3 | 912322060991 | `False` | 테이블 고정 |
| cam2 | 752112070297 | `True` | **구 손목 카메라, 현재 미장착** |

전부 1280×720, 2026-08-06 ChArUco intrinsic 캘리브레이션, reprojection 0.22~0.36px.

`config.example.json`이 이미 cam0/cam1/cam3만 fixed로 정의하고 cam2를 wrist로 따로
읽으므로, **시뮬레이션의 고정 3대가 실물 3대와 1:1로 대응**한다. 구조를 새로 짤 필요는
없고 wrist 항만 제거하면 된다.

---

## 2. 보드 두 장

리그에 ChArUco 보드가 **두 장** 있다. 어느 보드를 말하는지 항상 명시해야 한다.
규격은 `capture/board_config.py`에 단일 정의로 두었다.

| | **로봇 보드** | **테이블 보드** |
| --- | --- | --- |
| 위치 | 로봇 팔 끝(플랜지) | 광학 테이블 위 |
| 움직임 | 로봇과 함께 이동 | 고정 |
| 역할 | **Shah의 관측 대상** | 카메라 상호등록 기준(검증용) |
| squares (x×y) | 7 × 5 | 11 × 7 |
| checker | 17 mm | 25 mm |
| marker | 12 mm | 18 mm |
| dictionary | DICT_4X4 | DICT_4X4_250 |
| 물리 크기 | 119 × 85 mm | 275 × 175 mm |
| 체커 코너 | **24개** | 60개 |
| 마커 | 17개 | 38개 |
| 마커 ID | **미확인** ⚠ | **5 ~ 42** |

> 실험실 표기 `5*7`, `7*11`은 **세로×가로**다. OpenCV는 `(squares_x, squares_y)` =
> 가로, 세로 순이므로 뒤집어 넣는다. 테이블 보드가 `7*11` → `squares_x=11,
> squares_y=7`로 기록돼 있는 것과 일치하고, 사진상 두 보드 모두 가로가 길다.

테이블 보드 규격은 세 곳에서 교차 확인했다.
1. `jiwoo/rb-calibration-marker-experiment/config.py` `CharucoBoardConfig`
2. 같은 저장소 `intrinsics/charuco_intrinsics_report.json`
3. cam0 라이브 프레임에서 마커 ID **6~42** 실측 (계산값 5~42와 일치, 5번만 각도로 미검출)

---

## 3. ⚠ 마커 ID 충돌 — 촬영 전 반드시 해결

**두 보드가 같은 DICT_4X4를 쓴다.** 테이블 보드가 ID 5~42를 점유하므로, 로봇 보드가
OpenCV 기본값인 ID 0부터로 인쇄됐다면 **5~16번 12개가 겹친다.**

```
테이블 보드 : ID  5 ~ 42   (38 마커)
로봇 보드   : ID  0 ~ 16   (17 마커, 기본값 가정)  ← 12개 충돌
```

고정 카메라는 테이블을 향하므로 두 보드가 거의 항상 한 화면에 들어온다. ID가 겹치면
검출기가 두 보드의 마커를 섞어 ChArUco 보간을 하고, **예외도 경고도 없이 틀린 pose**를
낸다. 이대로 촬영하면 데이터를 통째로 버리게 된다.

참고로 `jiwoo/config.py`의 주석은 *"큐브(DICT_APRILTAG_36h11)와 다른 딕셔너리라 ID
겹쳐도 무방"* 이라고 되어 있다. 큐브가 AprilTag였기에 성립하던 전제이고, 로봇 보드가
DICT_4X4인 지금은 깨졌다.

### 로봇 보드 ID를 모르는 이유

인쇄에 쓴 PDF에 ID가 적혀 있지 않고, **PC 전체를 검색해도 이 보드의 생성 기록이
없다.** 아직 어느 파이프라인에도 등록된 적 없는 새 보드다. 따라서 **실물 보드가 유일한
진실의 출처**다.

```bash
# 로봇을 켤 필요 없음. 보드를 카메라 앞에 들거나 휴대폰 사진 한 장이면 된다.
python capture/check_board_ids.py --live
python capture/check_board_ids.py --image 보드사진.jpg
```

이 스크립트는 딕셔너리 변종(50/100/250/1000)과 ID 시작번호를 알려주고, 가로세로가
뒤바뀌지 않았는지까지 코너 수로 검증한다. 확인한 값을 `capture/board_config.py`의
`ROBOT_BOARD.marker_id_start`에 적어 넣으면 그 뒤로는 그 파일이 진실의 출처가 된다.

### 대응 — 권장안은 (c)

| | 방법 | 비용 |
| --- | --- | --- |
| (a) | 로봇 보드를 ID 50부터로 재인쇄 (50~66) | 재인쇄 + 재부착 |
| (b) | 로봇 보드만 다른 딕셔너리(DICT_5X5_50 등)로 재인쇄 | 재인쇄 + 재부착 |
| **(c)** | **촬영을 2단계로 나눈다** | **없음** ✔ |

**(c) 2단계 촬영:**

```
1단계  테이블 보드만 놓고 3대로 몇 장   →  T_cam_i_cam_j 기준값 확보
        (로봇 불필요. 카메라 상호등록 검증에 쓴다)

2단계  테이블 보드를 치우고 Shah 촬영   →  화면에 로봇 보드만 존재
        (ID가 겹쳐도 섞일 상대가 없다)
```

**두 단계 사이에 카메라를 절대 움직이지 않는 것**이 유일한 조건이다. 이러면 충돌이
성립하지 않고, 재인쇄도 필요 없고, §7의 registration 교차검증까지 공짜로 얻는다.

단 검출기에 넣을 `marker_id_start` 값은 여전히 필요하므로 위 스크립트 확인은 해야 한다.

촬영 중 섞임 감시를 위해, 각 촬영 레코드에 **`foreign_marker_ids`**(로봇 보드에 속하지
않는데 화면에 보인 마커 ID)를 기록한다. 비어 있지 않으면 테이블 보드가 아직 화면에
있다는 뜻이다.

---

## 4. 촬영 지침 — 회전 다양성이 전부다

시뮬레이션으로 측정한 결과, **로봇을 얼마나 크게 돌리는가가 정확도를 지배한다.**
조건: 고정 3대 + 로봇 보드(7×5/17mm) + corner noise 1mm + 30 trials.

### 회전 다양성 (자세 14개 고정)

| 자세 간 평균 상대회전 | Camera pose 오차 | Registration | X 흩어짐 |
| ---: | ---: | ---: | ---: |
| 5° | 95.09 mm | 138.82 mm | 73.24 mm |
| 10° | 28.32 mm | 42.60 mm | 18.05 mm |
| 20° | 11.21 mm | 17.03 mm | 4.57 mm |
| 30° | 7.17 mm | 10.77 mm | 2.10 mm |
| **40°** | **5.42 mm** | **8.03 mm** | **1.26 mm** |
| 58° | 3.84 mm | 5.62 mm | 0.65 mm |
| 76° | 3.10 mm | 4.50 mm | 0.43 mm |

### 자세 개수 (회전 40° 고정)

| 자세 수 | Camera pose | Registration |
| ---: | ---: | ---: |
| 6 | 9.73 mm | 13.41 mm |
| 10 | 7.08 mm | 9.86 mm |
| 14 | 5.42 mm | 8.03 mm |
| 20 | 4.45 mm | 6.47 mm |
| 30 | 3.67 mm | 5.47 mm |

**회전을 5°→40°로 늘리면 오차가 18배 줄지만, 자세를 6→30개로 늘려도 2.6배뿐이다.**

> 평행이동만 하며 30장 찍는 것보다, 크게 기울여 15장 찍는 편이 훨씬 낫다.
> 실험실 시간이 부족하면 장수를 줄이고 각도를 키워라.

**목표: 자세 간 평균 상대회전 40° 이상, 자세 20개 내외.** 각 축(rz/ry/rx)에 대해
±20~30° 범위를 고루 쓰면 도달한다.

로봇측 스크립트가 자세를 기록할 때마다 현재 평균 상대회전을 실시간으로 표시하므로,
목표에 도달했는지 촬영 중에 알 수 있다.

---

## 5. 예상 정확도 — 보드가 작아 불리하다

로봇 보드(119×85mm, 코너 24개)는 테이블 보드(275×175mm, 코너 60개)보다 작다. 코너가
적어 평균화 효과가 줄고, 보드가 작아 회전 추정의 lever arm이 짧다. 두 효과가 겹쳐
**정확도가 약 4배 나빠진다.**

고정 3대, Shah, 30 trials 기준:

| corner noise | 지표 | 테이블 보드 크기 | **로봇 보드 크기** |
| ---: | --- | ---: | ---: |
| 0 mm | 전 지표 | 0 (복원) | **0 (복원)** |
| 1 mm | Held-out | 0.61 mm | **2.30 mm** |
| | Camera pose | 1.76 mm | **7.08 mm** |
| | Registration | 2.67 mm | **9.65 mm** |
| | Reprojection | 1.91 px | **2.64 px** |
| 3 mm | Camera pose | 5.30 mm | **21.22 mm** |
| 5 mm | Camera pose | 8.83 mm | **35.38 mm** |

무잡음에서는 양쪽 다 정확히 0을 복원하므로 **알고리즘과 transform convention에는
문제가 없다.** 순수하게 보드 기하에서 오는 conditioning 문제다.

> **README §9의 수치는 실제 보드에 적용되지 않는다.** 논문에 예상치를 쓴다면 실제 보드
> 기하로 sweep을 다시 돌려야 한다.

또 하나 주목할 점: reprojection은 1.4배만 나빠지는데 camera pose는 4배 나빠진다.
README §8.4의 *"reprojection이 작다고 extrinsic이 정확한 건 아니다"* 라는 경고가 이
보드에서 특히 두드러진다. **reprojection만 보고 판단하면 안 된다.**

---

## 6. 현장 품질 지표 — ground truth 없이 쓸 수 있다

실데이터에는 GT가 없다. 그런데 카메라 3대가 **같은 로봇 보드**를 보므로, Shah는
`X = T_gripper_board`를 카메라마다 하나씩, 총 3개 내놓는다. 보드는 물리적으로
하나뿐이니 **세 추정치는 같아야 한다.** 그 흩어짐이 품질 신호다.

시뮬레이션에서 흩어짐과 실제 오차의 비를 재보니 노이즈 수준과 무관하게 일정했다:

| corner noise | X 흩어짐 (측정 가능) | 실제 Camera pose 오차 | 비 |
| ---: | ---: | ---: | ---: |
| 1 mm | 1.31 mm | 7.08 mm | 5.4× |
| 3 mm | 3.94 mm | 21.22 mm | 5.4× |
| 5 mm | 6.52 mm | 35.38 mm | 5.4× |

**현장에서 세 X 추정치의 평균 편차를 재고 약 5배 하면 실제 extrinsic 오차를 가늠할 수
있다.**

주의: 이 계수는 기하에 의존한다. 테이블 보드 크기에서는 4.7배, 자세 수를 바꾸면
4.3~5.4배 사이에서 움직였다. **실제 카메라 배치를 config에 넣고 시뮬레이션으로 계수를
다시 뽑아 두는 것을 권한다.** 코드에는 잠정값 5.4가 들어 있다.

PC측 클라이언트가 세션 종료 시 이 값을 자동으로 계산해 출력하고 `meta.json`의
`shah_field_check`에도 남긴다. **실험실을 떠나기 전에 세션이 쓸 만한지 알 수 있다.**

---

## 7. 실데이터에서는 지표 4개 중 2개만 계산된다

README §8의 통합 지표를 실데이터에 옮기면 이렇게 된다.

| README §8 지표 | 실데이터 | 근거 |
| --- | --- | --- |
| §8.1 Held-out chain | **계산 가능** | 로봇 FK를 기준으로 삼는다. 시뮬레이션의 `estimated_mounts` 방식 그대로 |
| §8.2 Camera pose accuracy | **불가** | GT 카메라 위치가 없다 |
| §8.3 Registration | **대체 가능** | 테이블 보드로 `T_cam_i_cam_j`를 로봇과 무관하게 직접 측정 → 대리 GT |
| §8.4 Held-out reprojection | **계산 가능** | 그대로 |

§8.3 대체가 §3의 2단계 촬영에서 1단계가 필요한 이유다. 로봇 FK 오차가 전혀 섞이지 않은
독립 기준이 생기므로, 결과가 어긋날 때 **원인이 카메라 쪽인지 로봇 FK 쪽인지 가를 수
있다.**

여기에 §6의 X 흩어짐이 Shah 전용 지표로 추가된다. (다른 5개 방법은 `AX=XB`라 `X`를
출력하지 않으므로 이 지표를 쓸 수 없다.)

---

## 8. 준비된 코드

```
capture/
├── board_config.py             두 보드의 단일 정의. 여기만 고치면 전체에 반영된다
├── check_board_ids.py          로봇 보드 마커 ID를 실물에서 읽는다
├── shah_capture_client.py      PC측 촬영 세션 레코더 (Python 3)
└── robot/
    └── shah_capture_server.py  로봇측 조작·촬영 트리거 (Python 2, ZEUS에 scp)
```

### 8.1 왜 새로 만들었나

기존 스크립트 중 이 리그에 맞는 것이 없었다.

- **`i611usr/c1.py` 계열** (= `robot_calb/c2.py` = `jiwoo/server/c1.py`, md5 동일):
  ArUco 큐브를 집었다 놨다 하는 시나리오 전용. 지금은 그리퍼가 없으므로
  `gripper_close()`가 IO 48번 응답을 5초 기다린 뒤 **경고만 찍고 조용히 진행**한다.
  자동 촬영 경로도 큐브를 집고 옮기고 놓는 동작이 필수 단계로 박혀 있다.
- **`i611usr/handeye_server.py`**: 그리퍼·큐브 코드가 없어 구조는 가장 가깝지만,
  자세를 `position_list`에 모으기만 하고 종료 시 `print`만 한다. **PC로 보내지도, 파일로
  쓰지도 않아 데이터가 남지 않는다.**

또 `i611usr/robot_calb/c1.py`에는 tool 전환 누수가 있다. 시작 시 `changetool(1)`인데
`get_cube_center()`가 `changetool(4)` 후 **3으로** 복귀하므로, 첫 촬영만 tool 1 기준이고
두 번째부터는 tool 3(+150mm) 기준으로 자세가 기록된다. 같은 데이터셋 안에서 좌표계가
한 번 바뀌는데 아무 경고가 없다. 새 스크립트에는 `changetool` 호출이 하나뿐이다.

### 8.2 tool 규약 — 중요

보드를 볼트로 고정했으므로 **tool 오프셋을 건드리지 않는다.** `changetool(1)`
(0,0,0,0,0,0)로 두고 `T_base_gripper`를 **순수 플랜지 자세**로 기록한다.

보드가 플랜지에서 얼마나 떨어져 있는지는 Shah의 `X`가 추정하므로 직접 재서 넣을 필요가
없고, 넣으면 **이중 보정**이 되어 결과가 어긋난다.

### 8.3 회전 규약

로봇측(Python 2, numpy 없음)과 PC측(Python 3)이 같은 규약을 써야 한다.

```
ZEUS 자세 : [x, y, z (mm), rz, ry, rx (deg)]
회전 행렬 : R = Rz @ Ry @ Rx   (scipy 기준 intrinsic "ZYX")
```

`jiwoo/robot_comm.py:euler_deg_to_matrix`와 동일하다. 두 구현이 일치하는지 검증했다
(오차 5.7e-14, §9 참조).

---

## 9. 촬영 절차

### 준비

1. **로봇 보드 ID 확인** (§3) — 아직 안 했다면 여기서 막힌다
   ```bash
   python capture/check_board_ids.py --live
   # 확인한 값을 capture/board_config.py 의 ROBOT_BOARD.marker_id_start 에 기입
   ```

2. **보드 강체 고정 확인** — 보드 모서리를 손으로 눌러 조금이라도 밀리면 안 된다.
   `X`가 상수라는 가정이 깨지면 무잡음이어도 수렴하지 않는다. 사진상 흰 보드 좌우의
   검은 띠가 벨크로인지 백플레이트인지 구분되지 않으므로 **직접 확인 필요.**

3. **cam2를 USB에서 분리** (§10 대역폭 문제)

### 1단계 — 테이블 보드 기준값 (로봇 불필요)

테이블 보드를 3대가 모두 보는 위치에 두고 몇 장 촬영. `T_cam_i_cam_j` 기준값을 얻는다.
**이후 카메라를 절대 움직이지 않는다.**

### 2단계 — Shah 촬영

테이블 보드를 치운다. 터미널 2개가 필요하다.

```
터미널 A: SSH → ZEUS                      터미널 B: PC (로컬)
  python shah_capture_server.py             python capture/shah_capture_client.py \
  "Waiting..."          ←── 접속 ──────────    --session-dir ~/shah_data/session01 \
                                               --robot-host 192.168.0.23 --show
  로봇 조작 / rec 로 자세 기록
  (평균 상대회전 40° 이상까지)
  start                  ─── capture ────→   3대 촬영·검출·저장
                         ←── saved ───────    응답
```

> **로봇 스크립트는 PC가 접속해야 프롬프트가 나온다.** `s.accept()`에서 블로킹하므로
> SSH만 열고 실행하면 "Waiting..."에서 멈춘다. 그리고 **이미지·자세·검출 결과는 전부
> PC에만** 쌓인다. ZEUS 컨트롤러에는 카메라가 없고, 로봇측 스크립트는 파일을 쓰지
> 않는다.

로봇측 명령:

| 명령 | 동작 |
| --- | --- |
| `p x,10` / `j d1,5` | TCP / 관절 상대이동 |
| `gotop` / `gotoj` | 절대이동 |
| `show` | 현재 자세 |
| `rec` | 현재 자세를 촬영 자세로 기록 (+ 평균 상대회전 표시) |
| `list` / `undo` | 기록 확인 / 취소 |
| `c` | 지금 자세로 즉시 촬영 |
| `start [speed]` | 기록된 자세를 순회하며 자동 촬영 |
| `q` | 종료 |

---

## 10. meta.json 스키마

`jiwoo/rb-calibration-marker-experiment`의 형식을 최대한 따랐다. 그 저장소의
Step3~Step5와 CP_* 분석 코드가 **전부 `meta.json`만 읽고 파일시스템을 뒤지지 않으므로**,
형식을 맞춰 두면 하류를 재사용할 수 있다.

```
세션 (파일당 1회)
  schema_version, root_folder, gripper_cam_idx(null), n_fixed_cams(3),
  n_gripper_cams(0), cam_indices, cam_serials,
  board_config_source, board_config, table_board_config,
  capture_config { intrinsics_dir, width, height, fps, save_depth,
                   settle_time_s, min_charuco_corners, max_charuco_reproj_px,
                   robot_pose_convention, transform_convention,
                   color_photometry }
  captures[]

촬영 (매 회)
  event_id, capture_index, capture_span_ms, capture_block("B_eyetohand"),
  robot_pose_6dof, robot_pose_matrix_4x4,
  capture_gripper_pose_6dof, capture_gripper_pose_matrix_4x4,   ← Step3 호환 별칭
  capture_robot_joints_6dof, tool, capture_gate, cams{}

카메라별 (매 회 × 3)
  saved, is_gripper(false), rgb_path, depth_path,
  ts_ms, host_monotonic_ts_ms, device_ts_ms,
  n_markers_detected, marker_ids, foreign_marker_ids,
  charuco_detect_n,
  charuco { ok, n_corners, reproj_error_px, rvec, tvec, T_cam_board_4x4 },
  gate_reason (null이면 통과), skip_reason (취득 실패 시)
```

### 기존 스키마와 다른 점

| 변경 | 이유 |
| --- | --- |
| `cube_config` → `board_config` | 큐브 대신 보드가 대상 |
| `table_board_config` 추가 | 보드가 두 장이므로 어느 쪽인지 기록해야 한다 |
| `foreign_marker_ids` 추가 | ID 충돌(§3) 감시 |
| `gripper_cam_idx: null` | 손목 카메라 없음 |
| `cube_pnp`, `set_index`, `grasp_id`, `capture_cube_center_6dof` 제거 | 이 리그에 대응물 없음 |
| `capture_block`은 항상 `"B_eyetohand"` | 보드가 gripper에 고정 = eye-to-hand. A/B 구분이 없어졌지만 Step3가 이 필드를 읽으므로 유지 |

### 설계 원칙 세 가지

1. **실패도 기록한다.** 코너가 모자라 거부된 촬영을 지우지 않고 `gate_reason`과 함께
   남긴다. 이미지도 검출 성공 여부와 무관하게 저장한다. README §11-7의
   *"실패 trial을 삭제하지 않고 failure로 기록한다"* 원칙이고, 나중에 검출기를 바꿔
   다시 뽑을 수 있어야 하기 때문이다.
2. **파일시스템을 스캔하지 않는다.** 하류는 항상 `meta["captures"]`를 읽는다. 폴더를
   glob하면 거부된 이미지가 섞여 들어온다.
3. **매 촬영마다 파일에 기록한다.** 중단되어도 그때까지의 데이터가 남는다.

### 저장 위치

세션 데이터는 이미지가 많아(기존 session01은 928장) 저장소에 커밋하지 않는 것이 좋다.
`--session-dir`을 저장소 밖으로 두거나, 저장소 안에 둔다면 `.gitignore`에 추가할 것.
**아직 `.gitignore`에 추가하지 않았다.**

---

## 11. 남은 작업

우선순위 순.

- [ ] **로봇 보드 마커 ID 확인** (§3) — 이것 없이는 촬영 불가. 최우선.
- [ ] **보드 강체 고정 확인** (§9 준비 2) — 벨크로면 Shah가 성립하지 않는다.
- [ ] **USB 대역폭 해결** (§12) — 3대 동시 스트리밍에서 cam3가 실패했다.
- [ ] **실제 카메라 배치를 config에 반영** — §5, §6의 수치는 `config.example.json`의
      가상 배치(워크스페이스 주위 반원 ±0.48m) 기준이다. 실제 배치로 다시 뽑아야
      예상치와 X-흩어짐 계수가 맞는다.
- [ ] **분석 단계 작성** — 촬영된 `meta.json`을 읽어 Shah를 포함한 6개 방법을 돌리고
      §7의 지표를 계산하는 코드. `jiwoo/Step3_calibration.py`가 참고 대상이지만 큐브
      전제가 섞여 있다. **아직 없다.**
- [ ] **`AX=XB` 5개 방법의 실데이터 연결** — Tsai/Park/Horaud/Andreff/Daniilidis는
      eye-to-hand에서 로봇 자세를 **반전**해 넣어야 하고 Shah는 반전하지 않는다. 차이는
      `SOTA_Simulation/shah_solver.py:123` 주석에 정리돼 있다. 촬영 데이터는 6개 방법이
      **공유**하므로 촬영은 한 번이면 된다.
- [ ] **README 갱신** — §2 좌표계의 eye-in-hand 항, §8.2/§8.3/§8.4의 wrist·6쌍·cam2
      언급이 실물과 어긋난다. §9 예시 결과도 테이블 보드 크기 기준이다.
- [ ] **`.gitignore`에 세션 데이터 경로 추가**

---

## 12. 알려진 문제

**cam3 프레임 타임아웃 (USB 대역폭).** 3대를 1280×720@15로 동시에 열면 cam3가
`Frame didn't arrive within 5000`으로 실패했다. 단독으로는 1280×720@15에서 10/10 정상,
640×480에서도 10/10 정상이므로 **장치 고장이 아니라 대역폭 경합**이다.

완화 수단:
- cam2(구 손목 카메라)를 USB에서 분리 — 스트리밍하지 않아도 대역폭을 나눠 쓴다
- 카메라를 서로 다른 USB 컨트롤러/루트 허브에 분산
- 파이프라인 순차 기동 (`--startup-stagger-s`, 기본 0.8초)
- 해상도를 640×480으로 낮춤 — 단 **intrinsics가 1280×720이므로 K가 맞지 않는다.**
  클라이언트가 해상도 불일치를 검사해 거부한다. 낮추려면 해당 해상도로 intrinsic을
  다시 잡아야 한다.

`jiwoo/camera.py`에 같은 증상에 대한 hardware_reset 처리가 있으니 참고할 것.

---

## 13. 근거와 재현

이 문서의 수치는 전부 이 저장소 코드로 측정한 것이다. 추정치나 인용값은 없다.

| 내용 | 방법 |
| --- | --- |
| §2 테이블 보드 ID 5~42 | cam0 라이브 프레임에서 `detectMarkers` — ID 6~42 검출 |
| §4 회전 다양성 표 | `generate_case` 궤적의 회전 진폭을 배율로 조정, 고정 3대 + 로봇 보드 기하, Shah, 30 trials |
| §5 보드 크기 비교 | `opencv_multicam_evaluation.py --methods shah --noise-mm 0 1 3 5 --trials 30`, board 블록만 교체 |
| §5, §6 고정 3대 수치 | `run_evaluation`의 지표 정의를 그대로 따르되 wrist 항 제거, 노이즈 seed 정렬 |
| §8.3 회전 규약 일치 | 로봇측 순수 python 구현과 scipy `ZYX` 비교 — 오차 5.7e-14 |
| §12 cam3 | 3대 동시 vs 단독 프레임 취득 비교 |

**코드 검증 (2026-09-03):**

- `BoardDetector`: 렌더한 로봇 보드에서 코너 24/24, reprojection 0.0000px
- `foreign_marker_ids`: 로봇 보드 검출기로 테이블 보드를 보면 외부 마커 38개 감지
- `meta.json` 스키마 → `solve_and_report`: 무잡음 합성 세션에서 `X`, `Y` 모두
  **1e-12 mm 수준 복원**
- 로봇 스크립트 Python 2 문법 컴파일 통과
- ID 미확인 시 클라이언트가 실행을 거부하는지 확인

**아직 검증하지 못한 것:**

- 실제 로봇과의 소켓 통신 (로봇이 없어 확인 불가)
- RealSense 3대 동시 촬영 경로 (§12의 대역폭 문제로 미완)
- 실물 보드에서의 검출 (로봇 보드가 카메라 앞에 없었음)

---

## 14. 참고

- 시뮬레이션 benchmark: [`README.md`](../README.md)
- Shah 알고리즘 검증: [`docs/shah_calibration_evaluation.md`](shah_calibration_evaluation.md)
- Shah solver와 transform convention: [`SOTA_Simulation/shah_solver.py`](../SOTA_Simulation/shah_solver.py)
- 참고용 기존 파이프라인 (**읽기 전용, 수정 금지**):
  `~/Desktop/**jiwoo/rb-calibration-marker-experiment/`
  — `Step2_capture.py`(저장 로직), `charuco_utils.py`, `config.py`, `robot_comm.py`
- ZEUS 로봇 스크립트 백업 (SSH 사본, 참고용): [`i611usr/`](../i611usr/)
