# Park–Martin Hand–Eye Calibration 시뮬레이션 평가

## 1. 작업 범위

이 문서는 이 저장소에 이미 연결된 OpenCV Park–Martin hand–eye solver를 새로 재구현한 결과가 아니다. 현재 simulation과 공통 evaluation protocol에서 Park 방법을 단독 실행하고, 변환 방향·무잡음 복원·noise 민감도·held-out 성능을 확인한 결과다.

- 실행 방법: `park` (`cv2.CALIB_HAND_EYE_PARK`)
- OpenCV: 4.13.0
- Python: 3.10.21
- 실행일: 2026-08-25
- 실행 브랜치: `2-implement-park-calibration`
- 브랜치 기준 commit: `931cef7`

## 2. Park–Martin 방법 요약

Park–Martin 방법은 hand–eye calibration을 Euclidean group의 `A X = X B` 문제로 표현한 closed-form 방법이다. 상대 motion의 rotation을 `SO(3)`에서 logarithm map으로 `so(3)` Lie algebra에 옮긴 뒤 rotation을 계산하고, 그 rotation을 고정한 선형 방정식으로 translation을 계산한다. 따라서 rotation과 translation을 한번에 풀지 않는 **separable method**다.

OpenCV 공식 문서는 Park를 separable hand–eye method로 분류한다. OpenCV 소스의 `calibrateHandEyePark()`도 pose pair에서 상대 motion을 만들고, rotation 로그 벡터로 rotation을 먼저 구한 후 translation 선형식을 SVD로 푼다.

참고:

- [Park & Martin, *Robot Sensor Calibration: Solving AX=XB on the Euclidean Group*](https://doi.org/10.1109/70.326576)
- [OpenCV `calibrateHandEye()` 문서](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)
- [OpenCV `calibration_handeye.cpp` 소스](https://github.com/opencv/opencv/blob/4.x/modules/calib3d/src/calibration_handeye.cpp)

## 3. 현재 코드의 입력·출력과 transform convention

저장소의 변환은 모두 `T_destination_source`, 즉 source frame의 좌표를 destination frame으로 옮기는 4×4 homogeneous transform으로 표기한다.

### Eye-in-hand wrist camera

OpenCV에 다음을 전달한다.

- Robot input: `T_base_gripper`
- Visual input: `T_wrist_board`
- OpenCV output: `T_gripper_wrist`

체인은 다음과 같다.

```text
T_base_board
  = T_base_gripper(k)
  @ T_gripper_wrist
  @ T_wrist_board(k)
```

### Eye-to-hand fixed camera

OpenCV hand–eye API를 eye-to-hand에 사용하기 위해 robot pose를 역행렬로 바꾸어 전달한다.

- Robot input: `T_gripper_base = inverse(T_base_gripper)`
- Visual input: `T_fixed_i_board`
- Interpreted output: `T_base_fixed_i`

체인은 다음과 같다.

```text
T_base_board(k)
  = T_base_fixed_i
  @ T_fixed_i_board(k)
```

실제 연결 코드는 `SOTA_Simulation/tsai_combined_demo.py` 내 `HAND_EYE_METHODS` 및 `solve_hand_eye()`에 있다. 방법별 script를 복사하지 않고, 공통 runner에 `--methods park`를 전달했다.

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

각 trial은 `seed + trial` RNG를 사용한다. 같은 trial의 1/3/5 mm 조건은 같은 standard-normal corner noise sample을 scaling한다. trajectory, calibration/held-out split, intrinsic, random seed를 변경하지 않았다.

재현 명령:

```powershell
conda activate sota-calibration-sim

python SOTA_Simulation/opencv_multicam_evaluation.py `
  --methods park `
  --noise-mm 0 1 3 5 `
  --trials 30 `
  --seed 2026 `
  --output examples/park_multicam_metrics
```

## 5. 결과

값은 30 trials의 `mean ± population standard deviation`이다.

| Noise | Held-out chain | Camera pose | Registration | Held-out reprojection |
| ---: | ---: | ---: | ---: | ---: |
| 0 mm | 0.000004 ± 0.000000 mm / 0.00000005° | 0.000004 ± 0.000000 mm / 0.00000003° | 0.000003 ± 0.000000 mm / 0.00000002° | 0.000002 ± 0.000000 px |
| 1 mm | 1.237 ± 0.318 mm / 0.285 ± 0.048° | 2.009 ± 0.441 mm / 0.190 ± 0.061° | 2.633 ± 0.587 mm / 0.272 ± 0.080° | 2.462 ± 0.067 px |
| 3 mm | 3.710 ± 0.953 mm / 0.856 ± 0.143° | 6.027 ± 1.319 mm / 0.570 ± 0.184° | 7.907 ± 1.754 mm / 0.816 ± 0.242° | 7.388 ± 0.203 px |
| 5 mm | 6.180 ± 1.586 mm / 1.426 ± 0.239° | 10.046 ± 2.192 mm / 0.951 ± 0.306° | 13.188 ± 2.911 mm / 1.361 ± 0.403° | 12.306 ± 0.334 px |

### 해석

- Noise 0에서 모든 translation/rotation/reprojection 오차가 numerical tolerance 내이므로 GT 복원을 통과했다.
- Corner noise가 커질수록 네 가지 평가 지표가 모두 증가했다.
- 30 trials 결과는 README의 Park 예시 결과와 반올림 수준에서 일치한다.
- 이 결과는 synthetic trajectory와 camera-frame 3D-corner Gaussian noise model에 대한 결과이다. 실제 RGB detector, robot FK, time synchronization, board geometry 오차를 포함한 절대 성능을 의미하지 않는다.

## 6. 평가 지표

- **Held-out chain error:** calibration에 쓰지 않은 board observation을 추정 extrinsic으로 base frame에 옮겨 GT board pose와 비교한 translation/rotation error.
- **Camera pose accuracy:** wrist와 fixed camera 3대의 extrinsic을 GT와 비교한 equal-camera macro error.
- **Multi-camera registration consistency:** 네 camera의 6개 pairwise relative transform을 GT relative transform와 비교한 평균 error.
- **Held-out reprojection RMSE:** held-out pose의 board corner를 robot pose와 추정 extrinsic으로 예측하여 noisy held-out observation과 비교한 pixel RMSE.

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

- Eye-in-hand: workspace에 정지한 board를 wrist camera가 서로 다른 robot rotation/translation에서 관측.
- Eye-to-hand: gripper와 관계가 고정된 board를 움직이며 각 fixed camera가 관측.
- 단순히 pose 개수만 늘리지 말고, rotation axis와 translation이 다양하게 변하는 non-degenerate trajectory가 필요하다.

### 전처리 및 검증

1. Camera intrinsic/distortion 보정 및 버전 고정.
2. Board corner 검출 후 최소 corner 수, positive depth, PnP reprojection error 검사.
3. Robot pose와 image를 같은 event로 synchronization.
4. Robot pose를 하나의 canonical physical frame으로 정규화.
5. PnP 출력이 `T_camera_board` 방향인지 확인하고 translation을 metre로 통일.
6. Eye-to-hand에서만 `T_base_gripper`를 `T_gripper_base`로 반전해 OpenCV에 전달.
7. Calibration/held-out split을 solver 실행 전에 고정하고 held-out event를 초기화와 fitting에서 제외.

실제 카메라 데이터만 있고 독립적인 물리 GT가 없다면 held-out reprojection과 camera 간 consistency는 평가할 수 있지만, 절대 camera pose accuracy는 주장할 수 없다. 절대 성능을 비교하려면 calibration에 사용하지 않은 정밀 jig, tracker, motion capture 또는 독립 6-DoF GT가 필요하다.

## 8. 결과 파일

- `examples/park_multicam_metrics/report.json`: 실험 설정과 noise별 mean/std
- `examples/park_multicam_metrics/records.csv`: 4 noise × 30 trials = 120개 원시 record
- `examples/park_multicam_metrics/figure1_integrated_metrics.png`: primary metric 그래프
- `examples/park_multicam_metrics/figure2_rotation_metrics.png`: rotation metric 그래프

