# Daniilidis Hand–Eye Calibration 시뮬레이션 평가

## 1. 작업 범위

이 문서는 저장소에 이미 연결된 OpenCV Daniilidis solver를 새로 구현한 결과가 아니다. 공통 simulation/evaluation protocol에서 Daniilidis 방법을 단독 실행하고 transform 방향, 무잡음 복원, noise 민감도와 held-out 성능을 확인한 결과다.

- 실행 방법: `daniilidis` (`cv2.CALIB_HAND_EYE_DANIILIDIS`)
- OpenCV: 4.13.0
- Python: 3.10.21
- 실행일: 2026-08-25
- 실행 브랜치: `5-implement-daniilidis-calibration`
- 브랜치 기준 commit: `dcb48c9`

## 2. Daniilidis 방법 요약

Daniilidis 방법은 hand–eye calibration의 `A X = X B` 관계에서 rigid motion을 dual quaternion으로 표현한다. 실수부 quaternion은 rotation을, dual part는 translation을 함께 나타내므로 rotation을 먼저 구하고 translation을 별도로 푸는 separable 방법과 달리 두 성분을 동시에 추정한다. OpenCV도 이 방법을 **simultaneous rotation/translation method**로 분류한다.

OpenCV의 `calibrateHandEyeDaniilidis()`는 pose pair에서 상대 motion을 만들고, 각 motion을 dual quaternion으로 변환해 homogeneous linear system을 구성한다. SVD로 얻은 null-space 후보와 unit dual-quaternion 제약으로 해를 정한 뒤 rotation matrix와 translation으로 변환한다. 본 benchmark는 별도의 보정이나 refinement 없이 OpenCV `calibrateHandEye()` 구현을 그대로 사용한다.

참고:

- [Daniilidis, *Hand-Eye Calibration Using Dual Quaternions*](https://doi.org/10.1177/02783649922066213)
- [저자 공개 논문 PDF](https://www.cis.upenn.edu/~kostas/mypub.dir/ijrr99.pdf)
- [OpenCV `calibrateHandEye()` 문서](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)
- [OpenCV `calibration_handeye.cpp` 소스](https://github.com/opencv/opencv/blob/4.x/modules/calib3d/src/calibration_handeye.cpp)

## 3. 입력·출력과 transform convention

저장소의 변환은 source frame 좌표를 destination frame으로 옮기는 `T_destination_source` 4×4 homogeneous transform으로 표기한다.

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

공통 연결 코드는 `SOTA_Simulation/tsai_combined_demo.py`의 `HAND_EYE_METHODS` 및 `solve_hand_eye()`에 있다. 평가는 공통 runner에 `--methods daniilidis`를 전달했다.

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
  --methods daniilidis `
  --noise-mm 0 1 3 5 `
  --trials 30 `
  --seed 2026 `
  --output examples/daniilidis_multicam_metrics
```

## 5. 결과

값은 30 trials의 `mean ± population standard deviation`이다.

| Noise | Held-out chain | Camera pose | Registration | Held-out reprojection |
| ---: | ---: | ---: | ---: | ---: |
| 0 mm | 0.000002 ± 0.000000 mm / 0.00000008° | 0.000002 ± 0.000000 mm / 0.00000007° | 0.000001 ± 0.000000 mm / 0.00000008° | 0.000001 ± 0.000000 px |
| 1 mm | 14.195 ± 11.852 mm / 22.062 ± 19.354° | 57.299 ± 49.263 mm / 21.983 ± 19.362° | 113.194 ± 98.547 mm / 43.833 ± 38.722° | 48.757 ± 41.197 px |
| 3 mm | 15.556 ± 11.541 mm / 22.586 ± 19.288° | 60.528 ± 48.752 mm / 22.350 ± 19.311° | 116.825 ± 97.524 mm / 44.300 ± 38.615° | 51.099 ± 38.949 px |
| 5 mm | 17.099 ± 11.413 mm / 23.113 ± 19.231° | 63.799 ± 48.296 mm / 22.719 ± 19.269° | 120.514 ± 96.546 mm / 44.775 ± 38.526° | 53.379 ± 36.657 px |

### 해석

- Noise 0에서 모든 translation/rotation/reprojection 오차가 numerical tolerance 내이므로 GT 복원을 통과했다.
- 30 trials 결과는 README의 Daniilidis 예시 결과와 반올림 수준에서 일치한다.
- Noise가 있는 조건에서는 mean뿐 아니라 trial 간 표준편차도 매우 크다. 현 wrist trajectory와 3D-corner noise 조합에서 일부 trial의 해가 크게 흔들리는 불안정성이 전체 camera registration에 전파된 결과다.
- 120개 record에는 NaN 또는 Inf가 없지만, 유한값이라는 사실이 안정적인 추정을 뜻하지는 않는다. 평균과 표준편차, 원시 trial 값을 함께 확인해야 한다.
- 이 결과는 현 synthetic trajectory와 noise model에 한정되며 Daniilidis 방법의 일반적인 실패나 실제 데이터의 절대 순위로 일반화할 수 없다.

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

### 촬영 구성 및 전처리

- Eye-in-hand: workspace에 정지한 board를 wrist camera가 다양한 robot pose에서 관측.
- Eye-to-hand: gripper와 관계가 고정된 board를 움직이며 fixed camera가 관측.
- Dual-quaternion system이 잘 구속되도록 rotation axis와 translation이 다양한 non-degenerate trajectory를 사용.
- Intrinsic/distortion을 고정하고 corner 검출, positive depth, PnP reprojection error를 검사.
- Robot pose와 image를 같은 event로 synchronization하고 robot pose를 하나의 canonical physical frame으로 정규화.
- PnP 출력을 `T_camera_board`, translation을 metre로 통일.
- Eye-to-hand에서만 `T_base_gripper`를 `T_gripper_base`로 반전해 OpenCV에 전달.
- Calibration/held-out split을 solver 실행 전에 고정하고 held-out event를 fitting에서 제외.

독립적인 물리 GT가 없는 실제 데이터에서는 held-out reprojection과 camera 간 consistency는 평가할 수 있지만 절대 camera pose accuracy는 주장할 수 없다. 절대 성능을 비교하려면 calibration에 사용하지 않은 정밀 jig, tracker, motion capture 또는 독립 6-DoF GT가 필요하다.

## 8. 결과 파일

- `examples/daniilidis_multicam_metrics/report.json`: 실험 설정과 noise별 mean/std
- `examples/daniilidis_multicam_metrics/records.csv`: 4 noise × 30 trials = 120개 원시 record
- `examples/daniilidis_multicam_metrics/figure1_integrated_metrics.png`: primary metric 그래프
- `examples/daniilidis_multicam_metrics/figure2_rotation_metrics.png`: rotation metric 그래프
