# Shah (2013) 실제 데이터 적용 준비

> 이 문서는 `README.md`의 좌표계 표기(`T_destination_source`)와 6.1절 1단계(Shah) 작업 순서를
> 기준으로, 실제 로봇 데이터에 Shah robot-world/hand-eye calibration을 적용하기 위한
> 사전 검토 자료임. `docs/shah_calibration_evaluation.md` 초안으로 사용 가능.

## 0. Shah 방법과 OpenCV 파라미터 매핑

OpenCV `calibrateRobotWorldHandEye(..., method=CALIB_ROBOT_WORLD_HAND_EYE_SHAH)`의
내부 관계식을 README 표기(`T_dest_src`)로 옮기면 다음과 같다.

```text
T_cam_gripper (X, 고정)  @  T_gripper_base_i (A_i, 매 pose 변화)
      =
T_cam_world_i (B_i, 매 pose 변화)  @  T_world_base (Z, 고정)
```

OpenCV 파라미터 ↔ README 표기 대응:

| OpenCV 파라미터 | 의미 (OpenCV 문서) | README 표기 |
| --- | --- | --- |
| `R/t_world2cam` (input) | world→cam 변환 | `T_cam_world` |
| `R/t_base2gripper` (input) | base→gripper 변환 | `T_gripper_base` |
| `R/t_base2world` (output) | base→world 변환 | `T_world_base` |
| `R/t_gripper2cam` (output) | gripper→cam 변환 | `T_cam_gripper` |

여기서 `world`는 물리적으로 **calibration board 좌표계**를 의미한다. 다만 아래
1절에서 보듯, board가 "고정"인지 "gripper에 부착"인지에 따라 `world` 역할에
넣어야 할 실제 값이 달라진다.

## 1. 실제 Shah 적용에 필요한 입력 데이터 목록

| 데이터 | 카메라 | 형태 | 비고 |
| --- | --- | --- | --- |
| `T_base_gripper(k)` | 공통 | 4x4 pose, per event | 로봇 컨트롤러/FK 로그 |
| `T_camera_board(k)` | wrist, cam0/1/3 | 4x4 pose, per event | board detection + PnP (또는 solvePnP) 결과 |
| Camera intrinsic/distortion | wrist, cam0/1/3 | `intrinsics/camN.npz` 형식과 동일 | 이미 보유 (README 3절) |
| Board geometry | 공통 | 정사각형 크기, 마커 배치, 원점 정의 | corner numbering 포함 |
| Event timestamp | 공통 | robot pose ↔ image capture 매칭 키 | sync 오차 원인이 되므로 필수 |
| Calibration/held-out split | 공통 | event index 목록 | 기존 시뮬레이션과 동일 split 유지 (14개: calib 10 / held-out 4) |
| Camera role 태그 | 공통 | `wrist` vs `fixed_i` | 아래 3절의 입력 방향이 이 태그로 갈라짐 |

시뮬레이션과 달리 실제 데이터에는 **detection 실패/outlier**, **board 미검출
frame**이 섞여 있을 수 있으므로, event별로 "이 카메라에서 이 pose가 유효한가"를
나타내는 valid-mask도 함께 준비해야 함 (README 11절의 "실패 trial을 삭제하지
않고 기록" 원칙과 연결).

## 2. Robot pose 데이터의 좌표계 / 방향 정의

- 로봇 컨트롤러가 보통 보고하는 값은 `T_base_gripper` (gripper pose를 base
  frame 기준으로 표현) — README의 `T_base_gripper`와 동일 방향.
- **Shah 어댑터 입력은 그 역변환이 필요**: `R/t_base2gripper` 파라미터는 실제로는
  `T_gripper_base = inv(T_base_gripper)`. 이는 wrist/fixed 구분과 무관하게 항상
  동일하게 적용된다.
- 확인해야 할 항목:
  - **Gripper frame 정의점**: flange 원점인지 TCP(tool center point)인지. Wrist
    카메라 마운트가 TCP 재정의에 포함되어 있으면 `T_gripper_wrist`가 이미
    TCP 기준으로 바뀌어 있을 수 있음 — 로봇 controller 설정과 반드시 대조.
  - **회전 표현**: 컨트롤러가 axis-angle, RPY(고정축/오일러축), quaternion 중
    무엇을 주는지, 각도 단위(rad/deg), RPY라면 intrinsic/extrinsic 순서(XYZ,
    ZYX 등) 확인. 잘못된 오일러 순서 변환은 zero-noise 검증에서도 "그럴듯하게
    틀린" 결과를 낼 수 있어 가장 놓치기 쉬움.
  - **단위**: translation이 mm인지 m인지 — OpenCV 함수 자체는 단위에 중립적이지만,
    board geometry(보통 m)와 반드시 통일해야 함.
  - **좌표계 방향(handedness)**: 로봇 base frame이 right-handed이고 base
    Z축이 실제로 어느 방향(up/down)인지 — 특히 UR류/타 브랜드 간 base frame
    정의가 다를 수 있음.

## 3. Camera-board observation 데이터의 좌표계 / 방향 정의

Board detection → `solvePnP`류 알고리즘의 표준 출력은 항상 `T_camera_board`
(board가 source, camera가 destination) 방향이다. 그런데 **Shah의 `world2cam`
입력에 이 값을 그대로 넣을 수 있는지는 카메라가 wrist인지 fixed인지에 따라
다르다** — README의 eye-in-hand/eye-to-hand 체인 정의(2절)와 직접 연결되는
부분이라 아래에 표로 정리함.

| 카메라 | README 체인 | 물리적으로 고정된 것 (X, 고정) | 물리적으로 고정된 것 (Z, 고정) | Shah `world2cam` 입력 |
| --- | --- | --- | --- | --- |
| wrist (`T_wrist_board(k)`) | eye-in-hand: board 고정, gripper 이동 | `T_cam_gripper` (카메라-gripper) | `T_world_base` (board-base) | `T_camera_board(k)` **그대로** |
| fixed_i (`T_fixed_i_board(k)`) | eye-to-hand: board가 gripper에 부착, 카메라 고정 | `T_board_gripper` (board-gripper 부착) | `T_cam_base` (카메라-base) | `inv(T_camera_board(k))` **역변환 필요** |

이유: Shah/`calibrateRobotWorldHandEye`가 성립하려면 "X 슬롯에 넣는 두 대상"과
"Z 슬롯에 넣는 두 대상"이 각각 **물리적으로 고정된 강체 관계**여야 함.
wrist에서는 카메라가 gripper에 고정돼 있어 그대로 넣으면 되지만, fixed
카메라는 카메라가 base에 고정돼 있고 board가 gripper에 고정돼 있어 역할이
뒤바뀐다. 이걸 그대로(비역변환) 넣으면 **수치가 나오긴 하지만 물리적으로
틀린 값**이 나오는데, 노이즈가 없는 상태에서도 GT와 어긋나는 형태로 나타나므로
README 6.1절이 강조하는 "zero-noise 검증을 먼저 통과시켜라"가 정확히 이 실수를
잡기 위한 절차임.

추가 확인 사항:

- **Board 좌표계 원점/축 방향**: corner numbering이 OpenCV 표준(좌상단 원점,
  x는 오른쪽, y는 아래쪽 등)과 일치하는지. ChArUco라면 marker ID ↔ 좌표
  매핑이 라이브러리 버전 간 달라질 수 있어 별도 확인 필요.
- **Intrinsic/distortion 모델**: `intrinsics/camN.npz`와 동일한 model(예:
  radial-tangential vs fisheye)인지, 실촬영 데이터도 같은 모델로 추정됐는지.
- **PnP 방법**: `solvePnP` 기본(iterative) vs `IPPE`/`SQPNP` 등 — 코너 개수가
  적거나 평면성이 강한 board에서는 방법에 따라 pose ambiguity(반사 해)가
  생길 수 있음. 두 개의 거의 동일한 해 중 하나가 잘못 선택되면 역시
  zero-noise 검증에서 드러남.

## 4. 실제 데이터 → Shah solver 입력 adapter 구성 방법

`SOTA_Simulation/adapter_template.py` 형식을 따르되, wrist와 fixed 카메라를
분리된 함수로 구성할 것을 권장.

```text
adapters/shah_adapter.py
├── build_shah_input_wrist(events, valid_mask)
│     A_i = inv(T_base_gripper(k))                 # 항상 역변환
│     B_i = T_camera_board(k)                       # 그대로
│     → cv2.calibrateRobotWorldHandEye(B, A, method=SHAH)
│     후처리: T_gripper_wrist = inv(R/t_gripper2cam)   # 저장용 (README 표기)
│            T_base_board    = inv(R/t_base2world)     # GT 비교용
│
└── build_shah_input_fixed(events, camera_id, valid_mask)
      A_i = inv(T_base_gripper(k))                 # 동일
      B_i = inv(T_camera_board(k))                  # 역변환 필요 (3절 참고)
      → cv2.calibrateRobotWorldHandEye(B, A, method=SHAH)
      후처리: T_gripper_board  = inv(R/t_gripper2cam)   # board가 gripper에 붙은 위치(진단용)
             T_base_fixed_i   = inv(R/t_base2world)     # 실제로 저장할 camera extrinsic
```

검증 순서(README 6.1/6.2절과 동일하게 유지):

1. **Zero-noise recovery**: 현재 시뮬레이션의 무잡음 GT 데이터를 이 어댑터에
   통과시켜, wrist/fixed 각각 GT `T_gripper_wrist`, `T_base_fixed_i`가
   tolerance 이내로 복원되는지 확인. 여기서 실패하면 3절의 방향 문제일
   가능성이 가장 높음.
2. **Transform convention 확인**: 어댑터 출력으로 재구성한
   `T_base_board = T_base_gripper(k) @ T_gripper_wrist @ T_wrist_board(k)`
   (wrist)와 `T_base_board(k) = T_base_fixed_i @ T_fixed_i_board(k)`
   (fixed)가 held-out pose에서도 서로 일치하는지 cross-check.
3. `0, 1, 3, 5 mm` noise sweep — 기존 runner(`tsai_noise_sweep.py`류)와
   동일한 trajectory·noise sample·seed 재사용.
4. Held-out evaluation — 8절 4개 통합 지표(held-out chain, camera pose,
   registration consistency, reprojection RMSE) 동일 포맷으로 출력.

주의: 이 어댑터는 board detection이 실패한 event를 자동으로 건너뛰지 말고
`valid_mask=False`로 표시해 report에 남길 것 (README 11절 원칙 7).

## 5. 실제 로봇 실험에서 필요한 calibration trajectory 검토

Shah의 closed-form(Kronecker product) 해는 상대운동 차분(`A_i A_j^{-1} X = X B_i B_j^{-1}`) 없이
절대 pose를 직접 사용하는 simultaneous solver이므로, **회전축이 충분히 다양한
pose 집합**이 있어야 해가 안정적으로 수렴한다. 기존 classical(Tsai 등)
방법보다 최소 요구 pose 수는 비슷하지만(이론상 3개 이상), 실무적으로는
다음을 권장:

- **회전축 다양성**: 최소 2~3개의 서로 평행하지 않은 회전축을 포함하도록
  구성 (한 축으로만 회전하는 trajectory는 특이(degenerate) 해를 유발).
- **Pose 개수**: 현재 시뮬레이션은 14개(calib 10 / held-out 4)로 무잡음
  검증에는 충분하나, 실제 데이터는 detection noise·outlier·FK 오차가
  섞이므로 calibration 20개 이상 + held-out 6~8개 수준으로 늘리는 것을
  권장 (README 12절에 명시된 "미포함 현실 요소" — FK 절대오차,
  backlash, thermal drift, timestamp sync 오차 — 가 실제 noise를
  키우기 때문).
- **Board 가시성**: fixed camera 3대 + wrist 1대가 각 pose에서 board를
  동시에 볼 필요는 없지만(카메라별로 독립 추정), **camera별로는 자신의
  FOV 안에서 board가 충분히 다양한 각도로 보이도록** trajectory를
  구성해야 함. Fixed camera는 board가 gripper를 따라 움직이므로, 해당
  camera FOV를 벗어나는 구간이 있는지 사전에 시뮬레이션/러프 스캔으로
  확인 필요.
- **Workspace 커버리지**: registration consistency(8.3절, pairwise 6쌍)를
  의미 있게 평가하려면 모든 fixed camera의 공통 관측 가능 영역을
  포함하는 pose 분포가 필요.
- **재현성/반복측정**: 실제 로봇은 backlash·thermal drift가 있으므로,
  가능하면 동일 목표 pose를 왕복 방향에서 재방문해 FK 반복 정밀도를
  별도로 측정해두는 것을 권장 (Shah 결과의 오차가 카메라/알고리즘
  기인인지 FK 기인인지 구분하기 위함).
- **Held-out split 원칙 유지**: 새 trajectory를 설계하더라도 "calibration에
  사용한 pose와 held-out pose는 완전히 분리"라는 README 7절 원칙은
  그대로 유지.

## 6. Shah 체크리스트

- [ ] Wrist 입력에서 `world2cam`을 **역변환 없이** 넣었는가
- [ ] Fixed 카메라 입력에서 `world2cam`을 **역변환해서** 넣었는가
- [ ] `base2gripper` 입력이 항상 `inv(T_base_gripper)`인가 (wrist/fixed 공통)
- [ ] 출력 `gripper2cam`, `base2world`를 각각 역변환해 README 표기로
      되돌렸는가
- [ ] Zero-noise 상태에서 wrist/fixed 각각 GT `T_gripper_wrist`,
      `T_base_fixed_i` 복원이 tolerance 이내인가
- [ ] Board corner numbering과 원점 정의가 OpenCV 표준과 일치하는가
- [ ] 로봇 pose의 회전 표현/오일러 순서/각도 단위를 실제 로그 포맷으로
      직접 확인했는가
- [ ] Detection 실패 event를 삭제하지 않고 valid_mask로 기록했는가
- [ ] 기존 trajectory·noise·seed·held-out split을 그대로 재사용했는가