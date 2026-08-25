# Tsai–Lenz Hand–Eye Calibration 시뮬레이션 평가

## 1. 작업 범위

이 문서는 저장소에 이미 연결되어 있는 OpenCV Tsai–Lenz solver를 공통 시뮬레이션 환경에서 실행하고 평가한 결과를 정리한 문서입니다. Tsai–Lenz 알고리즘을 새로 구현한 것은 아닙니다.

이번 실험에서는 transform 방향이 프로젝트의 기준과 맞는지, noise가 없을 때 Ground Truth(GT)를 복원하는지, noise가 커질 때 오차가 어떻게 달라지는지 확인했습니다. 또한 calibration에 사용하지 않은 held-out pose에서도 성능을 평가했습니다.

- 실행 방법: `tsai` (`cv2.CALIB_HAND_EYE_TSAI`)
- OpenCV: 4.13.0
- Python: 3.10.21
- 실행일: 2026-08-25
- 실행 브랜치: `1-implement-tsai-calibration`
- 브랜치 기준 commit: `931cef7`

## 2. Tsai–Lenz 방법 요약

Hand–eye calibration은 로봇의 움직임과 카메라에서 관측한 움직임 사이의 관계를 이용하여 미지의 카메라 위치를 구하는 문제이며, 일반적으로 `A X = X B` 형태로 표현합니다.

Tsai–Lenz는 수식을 반복적으로 최적화하지 않고 직접 해를 구하는 closed-form 방법입니다. 먼저 rotation을 추정하고, 구한 rotation을 고정한 상태에서 translation을 선형 least-squares로 계산합니다. 이처럼 두 값을 나누어 계산하므로 **rotation/translation separable method**로 분류합니다.

OpenCV의 `calibrateHandEyeTsai()`는 여러 pose 쌍에서 gripper와 camera의 상대적인 움직임을 계산합니다. Rotation은 minimal quaternion 벡터를 이용한 선형식을 SVD로 풀고, 올바른 unit rotation이 되도록 정규화합니다. 이후 계산한 rotation을 이용하여 translation 선형식을 다시 SVD로 풉니다.

OpenCV 구현은 상대 rotation이 너무 작거나 180°에 너무 가까운 pose 쌍을 rotation 계산에서 제외합니다. 따라서 실제 데이터를 촬영할 때는 비슷한 자세만 반복하지 않고, 회전축과 회전각이 다양한 움직임을 포함해야 합니다.

참고:

- [Tsai & Lenz, *A New Technique for Fully Autonomous and Efficient 3D Robotics Hand/Eye Calibration*](https://doi.org/10.1109/70.34770)
- [OpenCV `calibrateHandEye()` 문서](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)
- [OpenCV `calibration_handeye.cpp` 소스](https://github.com/opencv/opencv/blob/4.x/modules/calib3d/src/calibration_handeye.cpp)

## 3. 현재 코드의 입력·출력과 transform convention

이 프로젝트에서는 `T_destination_source` 형식으로 transform을 표기합니다. 이는 source frame의 좌표를 destination frame의 좌표로 변환한다는 뜻입니다. 모든 transform은 4×4 homogeneous transform입니다.

### Eye-in-hand wrist camera

- Robot input: `T_base_gripper`
- Visual input: `T_wrist_board`
- OpenCV output: `T_gripper_wrist`

```text
T_base_board
  = T_base_gripper(k)
  @ T_gripper_wrist
  @ T_wrist_board(k)
```

### Eye-to-hand fixed camera

Eye-to-hand에서는 robot pose를 역행렬로 바꾸어 OpenCV hand–eye API에 전달합니다.

- Robot input: `T_gripper_base = inverse(T_base_gripper)`
- Visual input: `T_fixed_i_board`
- Interpreted output: `T_base_fixed_i`

```text
T_base_board(k)
  = T_base_fixed_i
  @ T_fixed_i_board(k)
```

공통 연결 코드는 `SOTA_Simulation/tsai_combined_demo.py`의 `HAND_EYE_METHODS`와 `solve_hand_eye()`에 있습니다. 방법별 script를 따로 복사하지 않고, 공통 runner에 `--methods tsai`를 전달하여 평가했습니다.

## 4. 실험 설정

```text
Camera                 wrist 1 + fixed 3
Pose                   14
Calibration pose       10: 0, 1, 3, 4, 6, 7, 8, 10, 11, 13
Held-out pose           4: 2, 5, 9, 12
3D corner noise [mm]    0, 1, 3, 5
Trials                  30
Seed                    2026…2055
```

전체 pose 14개 중 10개는 calibration에 사용하고, 나머지 4개는 결과를 평가하는 held-out pose로 사용했습니다. 각 noise 조건에서 30회씩 반복하여 총 120개의 결과를 만들었습니다.

각 trial은 `seed + trial`을 random seed로 사용합니다. 같은 trial의 1, 3, 5 mm 조건에서는 동일한 standard-normal corner noise sample에 크기만 다르게 적용했습니다. 기존 trajectory, calibration/held-out split, intrinsic 및 random seed는 변경하지 않았습니다.

재현 명령:

```powershell
conda activate sota-calibration-sim

python SOTA_Simulation/opencv_multicam_evaluation.py `
  --methods tsai `
  --noise-mm 0 1 3 5 `
  --trials 30 `
  --seed 2026 `
  --output examples/tsai_multicam_metrics
```

## 5. 결과

아래 값은 30회 실행 결과의 `평균 ± 모집단 표준편차`입니다. Translation 관련 값은 mm, rotation 관련 값은 degree, reprojection 값은 pixel 단위입니다.

| Noise | Held-out chain | Camera pose | Registration | Held-out reprojection |
| ---: | ---: | ---: | ---: | ---: |
| 0 mm | 0.000004 ± 0.000000 mm / 0.00000010° | 0.000005 ± 0.000000 mm / 0.00000009° | 0.000003 ± 0.000000 mm / 0.00000009° | 0.000002 ± 0.000000 px |
| 1 mm | 1.335 ± 0.267 mm / 0.300 ± 0.052° | 2.081 ± 0.466 mm / 0.210 ± 0.065° | 2.756 ± 0.627 mm / 0.296 ± 0.087° | 2.478 ± 0.062 px |
| 3 mm | 4.305 ± 0.883 mm / 0.950 ± 0.164° | 6.400 ± 1.596 mm / 0.692 ± 0.198° | 8.865 ± 2.350 mm / 0.988 ± 0.281° | 7.505 ± 0.204 px |
| 5 mm | 8.037 ± 1.882 mm / 1.740 ± 0.297° | 11.460 ± 2.883 mm / 1.329 ± 0.364° | 16.980 ± 4.473 mm / 1.954 ± 0.561° | 12.777 ± 0.505 px |

### 해석

- Noise가 0 mm일 때는 모든 translation, rotation 및 reprojection 오차가 수치 계산에서 발생하는 매우 작은 오차 범위 안에 있습니다. 따라서 Ground Truth 복원에 성공했다고 볼 수 있습니다.
- Corner noise가 커질수록 네 가지 평가 지표가 모두 증가했습니다.
- 30회 실험 결과는 README의 Tsai 예시 결과와 반올림 수준에서 일치합니다.
- 이 결과는 현재 synthetic trajectory와 camera frame에 적용한 3D-corner Gaussian noise model에 대한 결과입니다. 실제 장비에서의 절대 성능을 의미하지는 않습니다.

## 6. 평가 지표

- **Held-out chain error:** Calibration에 사용하지 않은 pose에서 board 관측값과 추정 extrinsic을 연결하여 base frame의 board pose를 구합니다. 이를 GT board pose와 비교한 translation 및 rotation 오차입니다.
- **Camera pose accuracy:** Wrist camera 한 대와 fixed camera 세 대의 추정 extrinsic을 각각 GT와 비교한 뒤, 네 camera의 오차를 같은 비중으로 평균한 값입니다.
- **Multi-camera registration consistency:** 네 camera로 만들 수 있는 여섯 개 camera 쌍의 상대 transform을 계산하고, 이를 GT 상대 transform과 비교한 평균 오차입니다.
- **Held-out reprojection RMSE:** Calibration에 사용하지 않은 board corner를 robot pose와 추정 extrinsic으로 image에 다시 투영한 뒤, 관측한 noisy corner와 비교한 pixel 단위 RMSE입니다.

## 7. 실제 데이터 적용을 위한 입력

### 공통 필수 데이터

- 같은 시점끼리 연결된 event별 robot FK `T_base_gripper`가 필요합니다.
- Camera별 RGB image 또는 이미 검출된 board corner가 필요합니다.
- Calibration board의 3D corner geometry가 필요합니다.
- Camera별 intrinsic matrix `K`와 distortion coefficient `D`가 필요합니다.
- PnP로 계산한 event/camera별 `T_camera_board`가 필요합니다.
- `event_id`, camera ID 및 robot pose–image timestamp의 매칭 정보가 필요합니다.
- Translation 단위와 robot pose convention이 flange, TCP, tool frame 중 무엇인지 명시해야 합니다.
- Calibration event와 held-out event를 미리 나누어 두어야 합니다.

### 촬영 구성

- Eye-in-hand 실험에서는 board를 작업 공간에 고정하고, wrist camera가 여러 robot pose에서 board를 관측하도록 촬영해야 합니다.
- Eye-to-hand 실험에서는 board를 gripper에 고정하고 움직이면서 fixed camera가 board를 관측하도록 촬영해야 합니다.
- Tsai rotation 계산에 충분한 pose 쌍이 남도록 회전축과 회전각이 다양한 trajectory를 사용해야 합니다. 비슷한 움직임만 반복하는 degenerate trajectory는 피해야 합니다.

### 전처리 및 검증

1. Camera intrinsic과 distortion을 보정하고 사용한 버전을 고정해야 합니다.
2. Board corner를 검출한 뒤 최소 corner 수, positive depth 및 PnP reprojection error를 검사해야 합니다.
3. Robot pose와 image를 같은 event로 동기화해야 합니다.
4. Robot pose를 하나의 공통 physical frame 기준으로 정리해야 합니다.
5. PnP 출력이 `T_camera_board` 방향인지 확인하고 translation 단위를 metre로 통일해야 합니다.
6. Eye-to-hand 데이터에만 `T_base_gripper`를 `T_gripper_base`로 반전하여 OpenCV에 전달해야 합니다.
7. Solver 실행 전에 calibration/held-out split을 고정하고, held-out event는 초기화와 fitting에서 제외해야 합니다.

독립적인 물리 Ground Truth가 없는 실제 데이터에서도 held-out reprojection과 camera 사이의 consistency는 평가할 수 있습니다. 그러나 정확한 camera pose와의 절대 오차는 측정할 수 없습니다. 절대 성능을 비교하려면 calibration에 사용하지 않은 정밀 jig, tracker, motion capture 또는 별도의 6-DoF Ground Truth가 필요합니다.

## 8. 결과 파일

- `examples/tsai_multicam_metrics/report.json`: 실험 설정과 noise별 평균 및 표준편차가 들어 있습니다.
- `examples/tsai_multicam_metrics/records.csv`: 4개 noise 조건에서 각각 30회 실행한 총 120개의 원시 결과가 들어 있습니다.
- `examples/tsai_multicam_metrics/figure1_integrated_metrics.png`: 주요 translation 및 reprojection 평가 지표를 보여 주는 그래프입니다.
- `examples/tsai_multicam_metrics/figure2_rotation_metrics.png`: Rotation 평가 지표를 보여 주는 그래프입니다.

