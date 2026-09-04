# Daniilidis Hand–Eye Calibration 시뮬레이션 평가

## 1. 작업 범위

이 문서는 저장소에 이미 연결되어 있는 OpenCV Daniilidis solver를 공통 시뮬레이션 환경에서 실행하고 평가한 결과를 정리한 문서입니다. Daniilidis 알고리즘을 새로 구현한 것은 아닙니다.

이번 실험에서는 다음 내용을 확인했습니다.

- 입력과 출력의 transform 방향이 현재 프로젝트의 기준과 맞는지 확인했습니다.
- Noise가 없을 때 정답인 Ground Truth(GT)를 복원하는지 확인했습니다.
- Noise가 증가할 때 오차가 어떻게 변하는지 확인했습니다.
- Calibration에 사용하지 않은 held-out pose에서도 결과가 잘 맞는지 평가했습니다.

실행 환경은 다음과 같습니다.

- 실행 방법: `daniilidis` (`cv2.CALIB_HAND_EYE_DANIILIDIS`)
- OpenCV: 4.13.0
- Python: 3.10.21
- 실행일: 2026-08-25
- 실행 브랜치: `5-implement-daniilidis-calibration`
- 브랜치 기준 commit: `dcb48c9`

## 2. Daniilidis 방법 요약

Hand–eye calibration은 로봇의 움직임과 카메라에서 관측한 움직임 사이의 관계를 이용하여 미지의 카메라 위치를 구하는 문제이며, 일반적으로 `A X = X B` 형태로 표현합니다.

Daniilidis 방법은 3차원 회전과 이동을 **dual quaternion**이라는 하나의 표현으로 묶어서 계산합니다. Dual quaternion의 실수부 quaternion은 rotation을 나타내고, dual part는 translation을 나타냅니다. 따라서 rotation을 먼저 계산한 후 translation을 따로 계산하는 방법과 달리, Daniilidis 방법은 rotation과 translation을 동시에 추정합니다. OpenCV에서도 이 방법을 **simultaneous rotation/translation method**로 분류합니다.

OpenCV의 `calibrateHandEyeDaniilidis()`는 다음 순서로 계산합니다.

1. 여러 pose 쌍에서 로봇과 카메라의 상대적인 움직임을 계산합니다.
2. 각 움직임을 dual quaternion으로 변환합니다.
3. 이 관계를 homogeneous linear system으로 구성합니다.
4. SVD로 null-space 후보를 구하고, unit dual-quaternion 제약을 적용하여 해를 결정합니다.
5. 최종 결과를 rotation matrix와 translation으로 변환합니다.

이번 실험에서는 별도의 추가 보정이나 refinement를 적용하지 않고 OpenCV의 `calibrateHandEye()` 구현을 그대로 사용했습니다.

참고 자료는 다음과 같습니다.

- [Daniilidis, *Hand-Eye Calibration Using Dual Quaternions*](https://doi.org/10.1177/02783649922066213)
- [저자 공개 논문 PDF](https://www.cis.upenn.edu/~kostas/mypub.dir/ijrr99.pdf)
- [OpenCV `calibrateHandEye()` 문서](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)
- [OpenCV `calibration_handeye.cpp` 소스](https://github.com/opencv/opencv/blob/4.x/modules/calib3d/src/calibration_handeye.cpp)

## 3. 입력·출력과 transform convention

이 프로젝트에서는 `T_destination_source` 형식으로 transform을 표기합니다. 이는 source frame의 좌표를 destination frame의 좌표로 변환한다는 뜻입니다. 모든 transform은 4×4 homogeneous transform입니다.

### Eye-in-hand wrist camera

로봇 손목에 카메라가 고정된 경우에는 다음 값을 사용합니다.

- Robot input은 `T_base_gripper`입니다.
- Visual input은 `T_wrist_board`입니다.
- OpenCV output은 `T_gripper_wrist`입니다.

각 transform은 다음 관계를 만족해야 합니다.

```text
T_base_board
  = T_base_gripper(k)
  @ T_gripper_wrist
  @ T_wrist_board(k)
```

즉, 로봇 base에서 gripper로 이동하고, gripper에서 wrist camera로 이동한 다음, camera에서 board로 이동하면 base 기준 board pose를 얻을 수 있습니다.

### Eye-to-hand fixed camera

카메라가 외부에 고정된 경우에는 robot pose의 역행렬을 OpenCV hand–eye API에 전달합니다.

- Robot input은 `T_gripper_base = inverse(T_base_gripper)`입니다.
- Visual input은 `T_fixed_i_board`입니다.
- OpenCV output은 `T_base_fixed_i`로 해석합니다.

각 transform은 다음 관계를 만족해야 합니다.

```text
T_base_board(k)
  = T_base_fixed_i
  @ T_fixed_i_board(k)
```

즉, base에서 fixed camera로 이동한 다음 camera에서 board로 이동하면 base 기준 board pose를 얻을 수 있습니다.

공통 연결 코드는 `SOTA_Simulation/tsai_combined_demo.py`의 `HAND_EYE_METHODS`와 `solve_hand_eye()`에 있습니다. 이번 평가에서는 공통 runner에 `--methods daniilidis`를 전달했습니다.

## 4. 실험 설정

실험 조건은 다음과 같습니다.

```text
Camera                 wrist 1 + fixed 3
Pose                   14
Calibration pose       10: 0, 1, 3, 4, 6, 7, 8, 10, 11, 13
Held-out pose           4: 2, 5, 9, 12
3D corner noise [mm]    0, 1, 3, 5
Trials                  30
Seed                    2026…2055
```

전체 pose 14개 중 10개는 calibration에 사용하고, 나머지 4개는 calibration 결과를 평가하는 held-out pose로 사용했습니다. 각 noise 조건에서 30회씩 반복하여 총 120개의 결과를 만들었습니다.

각 trial은 `seed + trial`을 random seed로 사용합니다. 같은 trial의 1, 3, 5 mm 조건에서는 동일한 standard-normal corner noise sample에 크기만 다르게 적용했습니다. 따라서 noise 크기만 달라졌을 때의 결과를 비교할 수 있습니다. 기존 trajectory, calibration/held-out split, intrinsic 및 random seed는 변경하지 않았습니다.

다음 명령으로 실험을 재현할 수 있습니다.

```powershell
conda activate sota-calibration-sim

python SOTA_Simulation/opencv_multicam_evaluation.py `
  --methods daniilidis `
  --noise-mm 0 1 3 5 `
  --trials 30 `
  --seed 2026 `
  --output examples/daniilidis_multicam_metrics
```

## 5. 실행 결과

아래 값은 30회 실행 결과의 `평균 ± 모집단 표준편차`입니다. Translation 관련 값은 mm, rotation 관련 값은 degree, reprojection 값은 pixel 단위입니다.

| Noise | Held-out chain | Camera pose | Registration | Held-out reprojection |
| ---: | ---: | ---: | ---: | ---: |
| 0 mm | 0.000002 ± 0.000000 mm / 0.00000008° | 0.000002 ± 0.000000 mm / 0.00000007° | 0.000001 ± 0.000000 mm / 0.00000008° | 0.000001 ± 0.000000 px |
| 1 mm | 14.195 ± 11.852 mm / 22.062 ± 19.354° | 57.299 ± 49.263 mm / 21.983 ± 19.362° | 113.194 ± 98.547 mm / 43.833 ± 38.722° | 48.757 ± 41.197 px |
| 3 mm | 15.556 ± 11.541 mm / 22.586 ± 19.288° | 60.528 ± 48.752 mm / 22.350 ± 19.311° | 116.825 ± 97.524 mm / 44.300 ± 38.615° | 51.099 ± 38.949 px |
| 5 mm | 17.099 ± 11.413 mm / 23.113 ± 19.231° | 63.799 ± 48.296 mm / 22.719 ± 19.269° | 120.514 ± 96.546 mm / 44.775 ± 38.526° | 53.379 ± 36.657 px |

### 결과 해석

- Noise가 0 mm일 때는 모든 translation, rotation 및 reprojection 오차가 수치 계산에서 발생하는 매우 작은 오차 범위 안에 있습니다. 따라서 Ground Truth 복원에 성공했다고 볼 수 있습니다.
- 30회 실험의 평균 결과는 README에 정리된 Daniilidis 예시 결과와 반올림 수준에서 일치합니다.
- Noise가 있는 조건에서는 평균 오차뿐 아니라 trial 사이의 표준편차도 매우 큽니다. 이는 현재 wrist trajectory와 3D corner noise의 조합에서 일부 trial의 계산 결과가 크게 흔들렸고, 그 영향이 전체 camera registration 결과에도 전달되었다는 뜻입니다.
- 총 120개의 record에서 NaN 또는 Inf는 발견되지 않았습니다. 다만 모든 결과가 유한한 숫자로 나왔다는 사실만으로 안정적인 계산이었다고 판단할 수는 없습니다. 평균, 표준편차 및 각 trial의 원시 결과를 함께 확인해야 합니다.
- 이 결과는 현재 프로젝트의 synthetic trajectory와 noise model에 대한 결과입니다. 따라서 Daniilidis 방법이 일반적으로 실패한다고 결론 내리거나, 실제 데이터에서도 같은 성능 순위가 나온다고 단정할 수는 없습니다.

## 6. 평가 지표 설명

- **Held-out chain error:** Calibration에 사용하지 않은 pose에서, 관측한 board pose와 추정한 camera extrinsic을 연결하여 base frame의 board pose를 계산합니다. 이를 GT board pose와 비교한 translation 및 rotation 오차입니다.
- **Camera pose accuracy:** Wrist camera 한 대와 fixed camera 세 대의 추정 extrinsic을 각각 GT와 비교한 뒤, 네 camera의 오차를 동일한 비중으로 평균한 값입니다.
- **Multi-camera registration consistency:** 네 camera로 만들 수 있는 여섯 개의 camera 쌍에 대해 상대 transform을 계산하고, 이를 GT 상대 transform과 비교한 평균 오차입니다.
- **Held-out reprojection RMSE:** Calibration에 사용하지 않은 board corner를 robot pose와 추정 extrinsic으로 image에 다시 투영한 뒤, 관측한 noisy corner와 비교한 pixel 단위 RMSE입니다.

## 7. 실제 데이터 적용에 필요한 입력

### 공통 필수 데이터

실제 데이터에 Daniilidis 방법을 적용하려면 다음 데이터가 필요합니다.

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
- Dual-quaternion system이 충분히 계산될 수 있도록 rotation axis와 translation이 다양한 trajectory를 사용해야 합니다. 비슷한 방향의 움직임만 반복하는 degenerate trajectory는 피해야 합니다.
- Camera intrinsic과 distortion 값을 고정하고, corner 검출 결과, positive depth 및 PnP reprojection error를 확인해야 합니다.
- Robot pose와 image를 같은 event로 동기화하고, robot pose를 하나의 공통 physical frame 기준으로 정리해야 합니다.
- PnP 출력은 `T_camera_board` 방향으로 통일하고, translation 단위는 metre로 통일해야 합니다.
- Eye-to-hand 데이터에만 `T_base_gripper`를 `T_gripper_base`로 반전하여 OpenCV에 전달해야 합니다.
- Solver를 실행하기 전에 calibration/held-out split을 고정하고, held-out event는 calibration 계산에서 제외해야 합니다.

독립적인 물리 Ground Truth가 없는 실제 데이터에서도 held-out reprojection과 camera 사이의 consistency는 평가할 수 있습니다. 그러나 정확한 camera pose와의 절대 오차는 측정할 수 없습니다. 절대적인 성능을 비교하려면 calibration에 사용하지 않은 정밀 jig, tracker, motion capture 또는 별도의 6-DoF Ground Truth가 필요합니다.

## 8. 결과 파일

실행 결과는 다음 파일에 저장되어 있습니다.

- `examples/daniilidis_multicam_metrics/report.json`: 실험 설정과 noise별 평균 및 표준편차가 들어 있습니다.
- `examples/daniilidis_multicam_metrics/records.csv`: 4개 noise 조건에서 각각 30회 실행한 총 120개의 원시 결과가 들어 있습니다.
- `examples/daniilidis_multicam_metrics/figure1_integrated_metrics.png`: 주요 translation 및 reprojection 평가 지표를 보여 주는 그래프입니다.
- `examples/daniilidis_multicam_metrics/figure2_rotation_metrics.png`: Rotation 평가 지표를 보여 주는 그래프입니다.
