# Andreff Hand–Eye Calibration 시뮬레이션 평가

## 1. 작업 범위

이 문서는 공통 시뮬레이션 및 평가 환경에서 Andreff 방법을 단독으로 실행한 결과를 정리한 문서입니다. 이번 실험에서는 transform 방향이 프로젝트의 기준과 맞는지, noise가 없을 때 Ground Truth(GT)를 복원하는지, noise가 커질 때 오차가 어떻게 달라지는지 확인했습니다. 또한 calibration에 사용하지 않은 held-out pose에서도 성능을 평가했습니다.

- 실행 방법: `andreff` (`cv2.CALIB_HAND_EYE_ANDREFF`)
- OpenCV: 4.13.0
- Python: 3.10.21
- 실행일: 2026-08-25
- 실행 브랜치: `4-implement-andreff-calibration`
- 브랜치 기준 commit: `5a0010d`

## 2. Andreff 방법 요약

Hand–eye calibration은 로봇의 움직임과 카메라에서 관측한 움직임 사이의 관계를 이용하여 미지의 카메라 위치를 구하는 문제이며, 일반적으로 `A X = X B` 형태로 표현합니다.

Andreff·Horaud·Espiau 방법은 이 관계에서 rotation과 translation을 하나의 선형 system으로 묶어서 계산합니다. Rotation을 먼저 구한 뒤 translation을 따로 계산하는 Tsai, Park 및 Horaud와 달리 두 값을 동시에 추정합니다. 따라서 OpenCV는 Andreff 방법을 **simultaneous rotation/translation method**로 분류합니다.

OpenCV의 `calibrateHandEyeAndreff()`는 여러 pose 쌍의 상대적인 움직임에서 Kronecker product를 이용하여 rotation matrix 성분과 translation을 하나의 선형 방정식으로 구성합니다. 선형 해를 구한 뒤 rotation 부분이 올바른 `SO(3)` 회전 행렬이 되도록 정규화하고, 이때 계산된 scale을 translation에도 반영합니다.

원 논문은 새 데이터가 들어올 때마다 결과를 갱신할 수 있는 on-line 방식까지 설명합니다. 이번 실험에서는 데이터를 한 번에 계산하는 OpenCV의 batch `calibrateHandEye()` 구현을 사용했습니다.

참고:

- [Andreff, Horaud & Espiau, *On-line Hand-Eye Calibration*](https://doi.org/10.1109/IM.1999.805374)
- [OpenCV `calibrateHandEye()` 문서](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)
- [OpenCV `calibration_handeye.cpp` 소스](https://github.com/opencv/opencv/blob/4.x/modules/calib3d/src/calibration_handeye.cpp)

## 3. 입력·출력과 transform convention

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

공통 연결 코드는 `SOTA_Simulation/tsai_combined_demo.py`의 `HAND_EYE_METHODS`와 `solve_hand_eye()`에 있습니다. 공통 runner에 `--methods andreff`를 전달하여 평가했습니다.

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
  --methods andreff `
  --noise-mm 0 1 3 5 `
  --trials 30 `
  --seed 2026 `
  --output examples/andreff_multicam_metrics
```

## 5. 결과

아래 값은 30회 실행 결과의 `평균 ± 모집단 표준편차`입니다. Translation 관련 값은 mm, rotation 관련 값은 degree, reprojection 값은 pixel 단위입니다.

| Noise | Held-out chain | Camera pose | Registration | Held-out reprojection |
| ---: | ---: | ---: | ---: | ---: |
| 0 mm | 0.000007 ± 0.000000 mm / 0.00000006° | 0.000007 ± 0.000000 mm / 0.00000006° | 0.000008 ± 0.000000 mm / 0.00000007° | 0.000003 ± 0.000000 px |
| 1 mm | 3.353 ± 0.949 mm / 0.285 ± 0.047° | 3.861 ± 0.916 mm / 0.189 ± 0.060° | 5.556 ± 1.527 mm / 0.270 ± 0.081° | 5.138 ± 1.294 px |
| 3 mm | 20.001 ± 4.243 mm / 0.854 ± 0.142° | 21.268 ± 4.046 mm / 0.568 ± 0.181° | 32.564 ± 6.591 mm / 0.811 ± 0.242° | 36.892 ± 8.140 px |
| 5 mm | 46.774 ± 7.423 mm / 1.424 ± 0.236° | 48.120 ± 7.255 mm / 0.949 ± 0.302° | 71.233 ± 11.190 mm / 1.353 ± 0.401° | 75.709 ± 11.354 px |

### 해석

- Noise가 0 mm일 때는 모든 translation, rotation 및 reprojection 오차가 수치 계산에서 발생하는 매우 작은 오차 범위 안에 있습니다. 따라서 Ground Truth 복원에 성공했다고 볼 수 있습니다.
- Corner noise가 커질수록 네 가지 평가 지표가 모두 증가했습니다.
- 30회 실험 결과는 README의 Andreff 예시 결과와 반올림 수준에서 일치합니다.
- 현재 synthetic trajectory에서는 noise가 3 mm와 5 mm로 증가할 때 translation, registration 및 reprojection 오차가 빠르게 커졌습니다. Rotation 오차의 증가만으로는 전체 성능 악화를 설명할 수 없으므로 translation과 pixel 지표도 반드시 함께 확인해야 합니다.
- 이 결과는 현재 synthetic noise model에 대한 결과입니다. 실제 RGB/robot 데이터에서도 같은 성능 순위가 나온다고 단정할 수는 없습니다.

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

### 촬영 구성 및 전처리

- Eye-in-hand 실험에서는 board를 작업 공간에 고정하고, wrist camera가 여러 robot pose에서 board를 관측하도록 촬영해야 합니다.
- Eye-to-hand 실험에서는 board를 gripper에 고정하고 움직이면서 fixed camera가 board를 관측하도록 촬영해야 합니다.
- Rotation과 translation을 동시에 계산하는 system이 충분히 구속되도록 회전축과 translation이 다양한 trajectory를 사용해야 합니다. 비슷한 움직임만 반복하는 degenerate trajectory는 피해야 합니다.
- Camera intrinsic과 distortion 값을 고정하고, corner 검출 결과, positive depth 및 PnP reprojection error를 확인해야 합니다.
- Robot pose와 image를 같은 event로 동기화하고, robot pose를 하나의 공통 physical frame 기준으로 정리해야 합니다.
- PnP 출력은 `T_camera_board` 방향으로, translation 단위는 metre로 통일해야 합니다.
- Eye-to-hand 데이터에만 `T_base_gripper`를 `T_gripper_base`로 반전하여 OpenCV에 전달해야 합니다.
- Solver 실행 전에 calibration/held-out split을 고정하고, held-out event는 fitting에서 제외해야 합니다.

독립적인 물리 Ground Truth가 없는 실제 데이터에서도 held-out reprojection과 camera 사이의 consistency는 평가할 수 있습니다. 그러나 정확한 camera pose와의 절대 오차는 측정할 수 없습니다. 절대 성능을 비교하려면 calibration에 사용하지 않은 정밀 jig, tracker, motion capture 또는 별도의 6-DoF Ground Truth가 필요합니다.

## 8. 결과 파일

- `examples/andreff_multicam_metrics/report.json`: 실험 설정과 noise별 평균 및 표준편차가 들어 있습니다.
- `examples/andreff_multicam_metrics/records.csv`: 4개 noise 조건에서 각각 30회 실행한 총 120개의 원시 결과가 들어 있습니다.
- `examples/andreff_multicam_metrics/figure1_integrated_metrics.png`: 주요 translation 및 reprojection 평가 지표를 보여 주는 그래프입니다.
- `examples/andreff_multicam_metrics/figure2_rotation_metrics.png`: Rotation 평가 지표를 보여 주는 그래프입니다.

