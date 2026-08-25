# Tsai–Lenz Hand–Eye Calibration 시뮬레이션 평가

## 1. 작업 범위

이 문서는 저장소에 이미 연결된 OpenCV Tsai–Lenz solver를 새로 재구현한 결과가 아니다. 공통 simulation/evaluation protocol에서 Tsai 방법을 단독 실행하고, transform 방향·무잡음 복원·noise 민감도·held-out 성능을 확인한 결과다.

- 실행 방법: `tsai` (`cv2.CALIB_HAND_EYE_TSAI`)
- OpenCV: 4.13.0
- Python: 3.10.21
- 실행일: 2026-08-25
- 실행 브랜치: `1-implement-tsai-calibration`
- 브랜치 기준 commit: `931cef7`

## 2. Tsai–Lenz 방법 요약

Tsai–Lenz 방법은 hand–eye calibration을 상대 robot/camera motion의 `A X = X B` 문제로 표현한 closed-form 방법이다. Rotation을 먼저 추정하고 그 결과를 고정한 뒤 translation을 선형 least-squares로 계산하는 **rotation/translation separable method**다.

OpenCV의 `calibrateHandEyeTsai()`는 pose pair로 상대 gripper/camera motion을 만든다. Rotation은 minimal quaternion 벡터를 이용한 선형식을 SVD로 풀고 unit rotation으로 정규화한다. 이후 추정된 rotation을 사용해 translation 선형식을 SVD로 푼다. OpenCV 구현은 상대 rotation이 너무 작거나 π에 너무 가까운 pose pair를 rotation 계산에서 제외하므로, 실제 촬영에서 다양한 회전 motion이 중요하다.

참고:

- [Tsai & Lenz, *A New Technique for Fully Autonomous and Efficient 3D Robotics Hand/Eye Calibration*](https://doi.org/10.1109/70.34770)
- [OpenCV `calibrateHandEye()` 문서](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)
- [OpenCV `calibration_handeye.cpp` 소스](https://github.com/opencv/opencv/blob/4.x/modules/calib3d/src/calibration_handeye.cpp)

## 3. 현재 코드의 입력·출력과 transform convention

저장소의 변환은 `T_destination_source`, 즉 source frame 좌표를 destination frame으로 옮기는 4×4 homogeneous transform으로 표기한다.

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

Eye-to-hand에서는 robot pose를 역행렬로 바꾸어 OpenCV hand–eye API에 전달한다.

- Robot input: `T_gripper_base = inverse(T_base_gripper)`
- Visual input: `T_fixed_i_board`
- Interpreted output: `T_base_fixed_i`

```text
T_base_board(k)
  = T_base_fixed_i
  @ T_fixed_i_board(k)
```

공통 연결 코드는 `SOTA_Simulation/tsai_combined_demo.py`의 `HAND_EYE_METHODS` 및 `solve_hand_eye()`에 있다. 평가는 방법별 script를 복사하지 않고 공통 runner에 `--methods tsai`를 전달했다.

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

각 trial은 `seed + trial` RNG를 사용한다. 같은 trial의 1/3/5 mm 조건은 같은 standard-normal corner noise sample을 scaling한다. 기존 trajectory, calibration/held-out split, intrinsic, random seed를 변경하지 않았다.

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

값은 30 trials의 `mean ± population standard deviation`이다.

| Noise | Held-out chain | Camera pose | Registration | Held-out reprojection |
| ---: | ---: | ---: | ---: | ---: |
| 0 mm | 0.000004 ± 0.000000 mm / 0.00000010° | 0.000005 ± 0.000000 mm / 0.00000009° | 0.000003 ± 0.000000 mm / 0.00000009° | 0.000002 ± 0.000000 px |
| 1 mm | 1.335 ± 0.267 mm / 0.300 ± 0.052° | 2.081 ± 0.466 mm / 0.210 ± 0.065° | 2.756 ± 0.627 mm / 0.296 ± 0.087° | 2.478 ± 0.062 px |
| 3 mm | 4.305 ± 0.883 mm / 0.950 ± 0.164° | 6.400 ± 1.596 mm / 0.692 ± 0.198° | 8.865 ± 2.350 mm / 0.988 ± 0.281° | 7.505 ± 0.204 px |
| 5 mm | 8.037 ± 1.882 mm / 1.740 ± 0.297° | 11.460 ± 2.883 mm / 1.329 ± 0.364° | 16.980 ± 4.473 mm / 1.954 ± 0.561° | 12.777 ± 0.505 px |

### 해석

- Noise 0에서 모든 translation/rotation/reprojection 오차가 numerical tolerance 내이므로 GT 복원을 통과했다.
- Corner noise가 커질수록 네 가지 평가 지표가 모두 증가했다.
- 30 trials 결과는 README의 Tsai 예시 결과와 반올림 수준에서 일치한다.
- 이 결과는 synthetic trajectory와 camera-frame 3D-corner Gaussian noise model에 대한 결과이며, 실제 장비에서의 절대 성능을 의미하지 않는다.

## 6. 평가 지표

- **Held-out chain error:** held-out board observation을 추정 extrinsic으로 base frame에 옮겨 GT board pose와 비교한 translation/rotation error.
- **Camera pose accuracy:** wrist와 fixed camera 3대의 extrinsic을 GT와 비교한 equal-camera macro error.
- **Multi-camera registration consistency:** 네 camera의 6개 pairwise relative transform을 GT relative transform와 비교한 평균 error.
- **Held-out reprojection RMSE:** held-out board corner를 robot pose와 추정 extrinsic으로 예측해 noisy held-out observation과 비교한 pixel RMSE.

## 7. 실제 데이터 적용을 위한 입력

### 공통 필수 데이터

- 시간이 맞는 event별 robot FK `T_base_gripper`
- camera별 RGB image 또는 이미 검출된 board corner
- board의 3D corner geometry
- camera별 intrinsic matrix `K` 및 distortion coefficient `D`
- PnP로 계산한 event/camera별 `T_camera_board`
- `event_id`, camera ID, robot pose–image timestamp 매칭
- translation 단위와 robot pose convention(flange/TCP/tool frame) 명세
- calibration event와 held-out event의 고정 split

### 촬영 구성

- Eye-in-hand: workspace에 정지한 board를 wrist camera가 다양한 robot pose에서 관측.
- Eye-to-hand: gripper와 관계가 고정된 board를 움직이며 fixed camera가 관측.
- Tsai rotation 계산에 충분한 유효 pose pair가 남도록 rotation axis와 각도가 다양한 non-degenerate trajectory를 사용.

### 전처리 및 검증

1. Camera intrinsic/distortion 보정 및 버전 고정.
2. Board corner 검출 후 최소 corner 수, positive depth, PnP reprojection error 검사.
3. Robot pose와 image를 같은 event로 synchronization.
4. Robot pose를 하나의 canonical physical frame으로 정규화.
5. PnP 출력이 `T_camera_board` 방향인지 확인하고 translation을 metre로 통일.
6. Eye-to-hand에서만 `T_base_gripper`를 `T_gripper_base`로 반전해 OpenCV에 전달.
7. Calibration/held-out split을 solver 실행 전에 고정하고 held-out event를 초기화와 fitting에서 제외.

독립적인 물리 GT가 없는 실제 데이터에서는 held-out reprojection과 camera 간 consistency는 평가할 수 있지만 절대 camera pose accuracy는 주장할 수 없다. 절대 성능을 비교하려면 calibration에 사용하지 않은 정밀 jig, tracker, motion capture 또는 독립 6-DoF GT가 필요하다.

## 8. 결과 파일

- `examples/tsai_multicam_metrics/report.json`: 실험 설정과 noise별 mean/std
- `examples/tsai_multicam_metrics/records.csv`: 4 noise × 30 trials = 120개 원시 record
- `examples/tsai_multicam_metrics/figure1_integrated_metrics.png`: primary metric 그래프
- `examples/tsai_multicam_metrics/figure2_rotation_metrics.png`: rotation metric 그래프

