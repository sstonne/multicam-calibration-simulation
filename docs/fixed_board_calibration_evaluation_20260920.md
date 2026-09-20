# FixedBoard 4-camera 캘리브레이션 및 평가 결과

작성일: 2026-09-20  
대상: cam0·cam1·cam3 고정카메라와 cam2 그리퍼 카메라  
기준 결과: `results/fixed_board_calibration_20260920_v2/`  
상태: `review_pending_not_deployed`

## 1. 결론

9월 9일 `datasets/fixedBoard`의 15개 실제 촬영 자세로 cam2 eye-in-hand 캘리브레이션을 추가 수행했다. cam0·1·3은 같은 고정 보드의 train 관측으로 `T_base_camera`를 구하여, 네 카메라를 9월 9일 촬영 당시의 하나의 로봇 base 좌표계로 연결했다.

Shah, Tsai, Park, Horaud, Andreff, Daniilidis 6개 방법은 모두 train 및 전체 입력 계산에 성공했다. 고정 held-out 4개 자세에서 전체 4-camera 결과는 다음 범위였다.

- Held-out board-chain: **0.789–1.310 mm / 0.163–0.229°**
- Camera-pose consistency: **1.492–1.670 mm / 0.163–0.229°**
- Registration consistency: **2.260–3.244 mm / 0.289–0.414°**
- Held-out reprojection: **1.406–5.312 px**

이번 한 데이터에서는 Andreff가 네 표 지표 모두 가장 낮았지만, 9월 10일 움직이는 보드 데이터에서는 Andreff가 매우 불안정했다. 따라서 Andreff를 일반적인 최종 우승 방법으로 선정하지 않는다. Park·Horaud·Shah·Daniilidis도 서로 가까운 결과이며, 외부 물리 GT와 반복 설치 실험 없이 배포 방법을 확정하지 않는다.

중요하게, 9월 9일 `fixedBoard` 결과와 기존 9월 10일 고정카메라 결과를 직접 결합하면 약 60 mm 이상의 불일치가 나타났다. 두 날짜 사이 카메라 설치가 유지됐는지 metadata로 확인할 수 없으므로 두 결과를 한 rig의 동시 캘리브레이션처럼 혼합하지 않는다. 본 결과의 cam0·1·3과 cam2 한 벌은 **9월 9일 fixedBoard 촬영 상태**를 나타낸다.

## 2. 입력과 카메라 매핑

| 촬영 파일 | 카메라 | 역할 | Intrinsic serial |
| --- | --- | --- | --- |
| `cam_039422061216` | cam0 | 첫 번째 고정카메라 | `039422061216` |
| `cam_fixed2` | cam1 | 두 번째 고정카메라 | `319522062138` |
| `cam_gripper` | cam2 | 그리퍼 카메라 | `752112070297` |
| `cam_fixed3` | cam3 | 세 번째 고정카메라 | `912322060991` |

cam2가 그리퍼 카메라이고 RealSense 고정카메라 순서가 0·1·3이라는 사용자 확인을 분석 manifest에 기록했다. 각 intrinsic의 serial, `is_gripper`, 해상도 1280×720도 검사했다.

보드는 저장소의 `TABLE_BOARD` 설정을 사용했다.

- 11×7 squares, 25 mm square, 18 mm marker
- `DICT_4X4_250`, marker ID 5–42
- 내부 ChArUco corner 60개
- `legacy_pattern=False`

15개 자세 각각에 네 카메라 RGB·depth와 `robot.json`이 있다. RGB에서 ChArUco corner를 재검출하고 고정 intrinsic으로 `T_camera_board`를 PnP 계산했다. Depth는 scale 및 RGB-depth 정렬 metadata가 없어서 계산에는 쓰지 않았지만 파일 해시와 형식은 감사 기록에 남겼다.

| 카메라 | 원본 관측 | PnP 사용 | 계산 불가 제외 |
| --- | ---: | ---: | ---: |
| cam0 | 15 | 15 | 0 |
| cam1 | 15 | 13 | 2 |
| cam2 | 15 | 15 | 0 |
| cam3 | 15 | 15 | 0 |
| 합계 | 60 | 58 | 2 |

cam1의 event 007·008은 ArUco marker가 보이지만 ChArUco corner pose를 만들 수 없어 `fewer_than_four_charuco_corners`로 제외했다. 코너 수나 PnP reprojection 오차에 임의 문턱값을 적용하지 않았고, 계산 가능한 나머지 관측은 모두 사용했다.

관측별 판단은 [audit.json](../results/fixed_board_calibration_20260920_v2/audit.json)에 있다.

## 3. 캘리브레이션 방법

### 3.1 cam2 eye-in-hand

각 event `e`에서 다음 관계를 사용했다.

```text
T_base_robot(e)
× T_robot_cam2
× T_cam2_fixedBoard(e)
=
T_base_fixedBoard
```

Shah는 `T_robot_cam2`와 `T_base_fixedBoard`를 직접 함께 추정한다. 나머지 5개 OpenCV hand-eye 방법은 `T_robot_cam2`를 계산한 뒤, train event의 `T_base_robot × T_robot_cam2 × T_cam2_fixedBoard`를 평균하여 `T_base_fixedBoard`를 정했다. 여기서 `robot.json`은 i611 `[x,y,z,rz,ry,rx]` mm/degree를 명시하며 `Rz @ Ry @ Rx`로 변환했다. 단, 정확한 active tool 번호는 JSON에 없으므로 결과 변환 이름은 엄밀하게 `T_recordedRobotPose_cam2`로 읽어야 한다.

### 3.2 cam0·1·3 fixed-board 캘리브레이션

cam2 train으로 정한 `T_base_fixedBoard`와 각 고정카메라 PnP를 이용했다.

```text
T_base_fixedCamera_i(e)
=
T_base_fixedBoard
× inverse(T_fixedCamera_i_fixedBoard(e))
```

train event에서 얻은 카메라 pose들의 회전 평균과 이동 평균을 해당 방법의 cam0·1·3 추정으로 사용했다. 따라서 새 결과는 cam2만 추가한 것이 아니라, 9월 9일 fixedBoard 촬영 상태에 대한 cam0·1·2·3 한 벌이다.

### 3.3 분할

기존 공통 simulation protocol에 있던 held-out ID 2, 5, 9, 12를 solver 실행 전에 고정했다.

- Train 11개: 000, 001, 003, 004, 006, 007, 008, 010, 011, 013, 014
- Held-out 4개: 002, 005, 009, 012

1.5 mm / 1° 이내 근접 robot pose group이 train과 held-out에 갈라지지 않는지 확인했으며, 15개는 모두 서로 다른 group이었다. 근접 group 검사는 분할 누출 방지용이고 관측 제거용이 아니다.

평가용 train 추정과 15개 전체 입력 최종 추정을 분리했다. 아래 성능표는 train 추정만으로 held-out을 평가한 값이고, 배포 검토용 전체 행렬은 [full_data_estimates.json](../results/fixed_board_calibration_20260920_v2/full_data_estimates.json)에 별도로 저장했다.

## 4. 요청 지표 결과

| 방법 | Held-out (mm / deg) | Camera pose (mm / deg) | Registration (mm / deg) | Reprojection (px) |
| --- | ---: | ---: | ---: | ---: |
| Shah | 0.943 / 0.163 | 1.555 / 0.163 | 2.397 / 0.289 | 1.991 |
| Tsai | 1.310 / 0.229 | 1.670 / 0.229 | 3.244 / 0.414 | 5.312 |
| Park | 0.882 / 0.163 | 1.492 / 0.163 | 2.292 / 0.289 | 1.893 |
| Horaud | 0.882 / 0.163 | 1.493 / 0.163 | 2.292 / 0.289 | 1.884 |
| Andreff | **0.789 / 0.163** | **1.492 / 0.163** | **2.260 / 0.289** | **1.406** |
| Daniilidis | 0.932 / 0.166 | 1.609 / 0.166 | 2.451 / 0.297 | 1.560 |

표시값은 소수점 이하 3자리로 반올림했다. 미반올림 값은 [summary.csv](../results/fixed_board_calibration_20260920_v2/summary.csv)에 있다.

### 지표 계약

- **Held-out:** `T_calibratedCamera × T_camera_board`와 train에서 고정한 `T_base_fixedBoard`의 차이를 16개 held-out event-camera 관측에서 평균했다.
- **Camera pose:** `T_base_fixedBoard × inverse(T_camera_board)`로 얻은 held-out PnP 기준 pose와 캘리브레이션 pose의 차이다. 카메라별 event 평균 후 네 카메라를 동일 가중치로 평균했다.
- **Registration:** 캘리브레이션 camera pose가 만드는 상대변환과 같은 event의 공통 보드 PnP가 만드는 상대변환을 비교했다. 6개 camera pair별 event 평균 후 pair를 동일 가중치로 평균했다.
- **Reprojection:** train 결과로 예측한 held-out board pose를 영상에 투영해 641개 관측 corner를 직접 pooling한 `sqrt(sum(||duv||²)/N)`이다.

Camera pose와 Registration은 이제 N/A가 아니라 실제 데이터에서 계산됐다. 그러나 둘 다 fixedBoard PnP를 reference로 사용하는 **held-out consistency**다. tracker/CMM 등으로 측정한 독립 물리 GT accuracy는 아니다.

Registration은 held-out 4 event × 6 camera pair = 24개 상대변환을 사용했다. 별도의 양방향 cross-view pixel transfer는 1,923 destination corner에서 계산했다.

| 방법 | Cross-view reprojection RMSE (px) |
| --- | ---: |
| Shah | 2.855 |
| Tsai | 6.685 |
| Park | 2.716 |
| Horaud | 2.705 |
| Andreff | 2.175 |
| Daniilidis | 2.267 |

## 5. cam2 그리퍼 카메라 결과

전체 4-camera 수치는 거의 정지한 고정카메라 3대를 포함하므로 cam2만 별도로 확인해야 한다.

| 방법 | cam2 held-out (mm / deg) | cam2 reprojection (px) |
| --- | ---: | ---: |
| Shah | 2.568 / 0.456 | 3.202 |
| Tsai | 4.038 / 0.721 | 8.693 |
| Park | 2.327 / 0.456 | 3.038 |
| Horaud | 2.326 / 0.456 | 3.022 |
| Andreff | **1.954 / 0.455** | **2.212** |
| Daniilidis | 2.527 / 0.468 | 2.475 |

고정카메라와 보드는 모두 정지하므로 cam0·1·3의 held-out은 새로운 관점 일반화보다 PnP 반복성과 시간 안정성을 주로 측정한다. 알고리즘 차이는 대부분 cam2 eye-in-hand 추정에서 나온다.

전체 15개 입력으로 계산한 `T_robot_cam2` 이동 성분은 다음과 같다. 회전과 전체 4×4 행렬은 JSON을 기준으로 한다.

| 방법 | x (mm) | y (mm) | z (mm) |
| --- | ---: | ---: | ---: |
| Shah | 32.703 | 27.804 | 35.552 |
| Tsai | 36.789 | 32.612 | 36.625 |
| Park | 33.015 | 27.395 | 36.026 |
| Horaud | 33.017 | 27.413 | 36.034 |
| Andreff | 33.144 | 26.912 | 38.462 |
| Daniilidis | 33.378 | 28.704 | 35.666 |

Park와 Horaud의 cam2 결과는 거의 동일하다. Shah·Park·Horaud·Daniilidis 사이 이동 차이는 약 1.4 mm 이내지만, Andreff는 일부 방법과 약 2.5–3.3 mm, Tsai는 다른 방법과 최대 약 7.0 mm 및 2.3° 차이가 난다.

## 6. 9월 10일 결과와의 비교 진단

9월 10일 움직이는 그리퍼 보드로 구한 cam0·1·3 결과를 고정하고, 9월 9일 fixedBoard 및 새 cam2 결과와 함께 평가하면 다음과 같다.

| 방법 | Board-chain (mm / deg) | Camera pose (mm / deg) | Registration (mm / deg) | Reprojection (px) |
| --- | ---: | ---: | ---: | ---: |
| Shah | 59.943 / 8.091 | 105.007 / 8.091 | 164.823 / 12.914 | 92.679 |
| Tsai | 60.592 / 9.269 | 109.364 / 9.269 | 163.170 / 14.273 | 97.077 |
| Park | 60.296 / 8.090 | 106.420 / 8.090 | 166.055 / 12.886 | 98.763 |
| Horaud | 60.158 / 8.093 | 106.319 / 8.093 | 165.791 / 12.912 | 98.474 |
| Andreff | 100.429 / 8.158 | 150.551 / 8.158 | 187.947 / 12.825 | 218.490 |
| Daniilidis | 60.341 / 7.951 | 105.469 / 7.951 | 162.904 / 12.652 | 96.902 |

원시 값은 [prior_calibration_comparison.csv](../results/fixed_board_calibration_20260920_v2/prior_calibration_comparison.csv)에 있다. 이 불일치는 다음 가능성을 포함하지만 현재 데이터만으로 원인을 분리할 수 없다.

- 9월 9일과 10일 사이 고정카메라 설치 위치 변화
- `robot.json`의 active tool/frame과 9월 10일 flange convention 차이
- 촬영 당시와 현재 intrinsic 차이
- 서로 다른 target PnP의 systematic 편향
- 9월 10일 기존 캘리브레이션 자체의 오차

따라서 9월 9일 새 4-camera 결과로 9월 10일 결과를 자동 교체하지 않았고, 두 날짜 결과를 joint optimization에도 함께 넣지 않았다. 동일 설치가 유지됐다는 근거가 확보되기 전에는 이 비교를 알고리즘 성능 차이로 해석하지 않는다.

## 7. 결과 해석

1. **추가 캘리브레이션은 가능했고 계산에 성공했다.** cam2 eye-in-hand와 cam0·1·3 fixed-board pose를 합쳐 4-camera 한 벌을 만들었다.
2. **요청된 네 지표를 모두 숫자로 채웠다.** 다만 Camera pose와 Registration은 independent physical GT가 아니라 held-out fixed-board consistency다.
3. **이번 데이터에서는 Andreff가 가장 낮다.** 하지만 이전 9월 10일 데이터에서의 큰 발산과 4개 held-out pose만 고려하면 최종 방법으로 확정할 근거가 부족하다.
4. **Tsai는 이번 cam2에서 상대적으로 불리하다.** held-out 4.038 mm / 8.693 px로 다른 방법보다 높고, 전체 추정도 다른 방법과 회전 약 2.3° 차이가 난다.
5. **고정카메라 수치가 낮은 것은 정상이나 과대해석하면 안 된다.** 고정카메라와 보드가 동일 자세이므로 새로운 공간 위치에서의 정확도를 시험한 것이 아니다.
6. **현재 행렬은 배포하지 않는다.** `robot.json`의 정확한 active tool 확인, 동일 설치 반복촬영, 물리 GT 또는 실제 task 검증이 필요하다.

## 8. 산출물

기준 폴더: `results/fixed_board_calibration_20260920_v2/`

| 파일 | 내용 |
| --- | --- |
| [analysis.json](../results/fixed_board_calibration_20260920_v2/analysis.json) | 전체 결과, 지표 정의, 가정, provenance |
| [full_data_estimates.json](../results/fixed_board_calibration_20260920_v2/full_data_estimates.json) | 6개 방법의 전체 입력 cam2 및 cam0·1·3 행렬 |
| [prepared.json](../results/fixed_board_calibration_20260920_v2/prepared.json) | 확정 입력, PnP, intrinsic, 파일 해시 |
| [audit.json](../results/fixed_board_calibration_20260920_v2/audit.json) | 60개 camera 관측 포함·제외 감사 |
| [split.json](../results/fixed_board_calibration_20260920_v2/split.json) | 공통 train/held-out 분할 |
| [summary.csv](../results/fixed_board_calibration_20260920_v2/summary.csv) | 요청 네 지표의 미반올림 값 |
| [per_camera.csv](../results/fixed_board_calibration_20260920_v2/per_camera.csv) | 카메라별 held-out 값 |
| [event_metrics.csv](../results/fixed_board_calibration_20260920_v2/event_metrics.csv) | event-camera 원시 지표 |
| [registration_pairs.csv](../results/fixed_board_calibration_20260920_v2/registration_pairs.csv) | event-pair registration 및 cross-view 원시 지표 |
| [prior_calibration_comparison.csv](../results/fixed_board_calibration_20260920_v2/prior_calibration_comparison.csv) | 9월 10일 기존 결과와의 별도 진단 |

초기 진단 결과 폴더 `results/fixed_board_calibration_20260920/`은 보존돼 있으나, 설명문과 provenance를 정리해 다시 실행한 `_v2`를 canonical 결과로 사용한다.

## 9. 재현 방법과 검증

노트북 부하를 줄이기 위해 OpenCV·BLAS를 한 스레드로 제한하고, 준비와 6개 방법을 순차 단계로 실행했다. 각 단계는 입력 SHA-256을 다시 확인하며 기존 방법 결과를 덮어쓰지 않는다.

```powershell
$env:OPENBLAS_NUM_THREADS = '1'
$env:OMP_NUM_THREADS = '1'
$env:MKL_NUM_THREADS = '1'
$output = 'results/fixed_board_calibration_NEW_RUN'

.\.venv-unified\Scripts\python.exe -m SOTA_Simulation.fixed_board_evaluation --output $output --method prepare

foreach ($method in @('shah','tsai','park','horaud','andreff','daniilidis')) {
    .\.venv-unified\Scripts\python.exe -m SOTA_Simulation.fixed_board_evaluation --output $output --method $method
    if ($LASTEXITCODE -ne 0) { throw "failed: $method" }
}

.\.venv-unified\Scripts\python.exe -m SOTA_Simulation.fixed_board_evaluation --output $output --method collect
```

검증 결과:

- 새 unit test와 기존 real/unified 회귀 test: **28 passed**
- Shah eye-in-hand/eye-to-hand 무잡음 convention self-test: 통과
- 6개 방법의 train 및 full-data fit: 모두 성공
- 모든 방법: held-out 16개 관측, 641개 corner, registration 24개/6 pair, cross-view 1,923개 directed corner
- 방법별 JSON 원시 행에서 mean, corner-pooled RMSE, equal-pair macro를 독립 재집계해 summary와 일치함을 확인
- `datasets`와 기존 `results/real_260910_unified_20260917_v2`는 변경하지 않음

실행기는 [fixed_board_evaluation.py](../SOTA_Simulation/fixed_board_evaluation.py), 검증 코드는 [test_fixed_board_evaluation.py](../tests/test_fixed_board_evaluation.py)다.
