# 9월 10일 실촬영 데이터 통합 캘리브레이션 재수행 레포트

작성일: 2026-09-17  
분석 대상: 고정카메라 cam0·cam1·cam3 / 2026-09-10 촬영 3개 세션  
결과 상태: `review_pending_not_deployed` — 검토 대기, 로봇에 적용하지 않음

## 1. 요약

회의에서 정리한 요구사항에 따라 기존 결과를 보존하고, 실촬영 데이터를 품질 문턱값으로 선별하지 않은 채 카메라별로 세션을 통합하여 캘리브레이션을 다시 수행했다. 합성 노이즈는 추가하지 않았다.

117개 촬영 event를 모두 유지했다. 총 234개 event-camera 관측 중 변환 행렬 입력을 사용할 수 없는 18개 관측만 계산 대상에서 제외하고, 216개 관측을 사용했다. Shah, Tsai, Park, Horaud, Andreff, Daniilidis의 6개 방법에 대해 cam0·1·3 결과를 각각 한 벌로 생성했다. 전체 입력을 사용한 18개 카메라별 추정은 모두 계산에 성공했다.

공통 held-out 평가에서는 Shah의 전체 평균 위치 잔차가 **7.471 mm**, reprojection RMSE가 **9.269 px**로 6개 방법 중 가장 낮았다. 그러나 cam3의 오차가 모든 방법에서 크게 나타났으며, 수치 계산 성공을 물리 정확도 검증으로 해석할 수는 없다.

요청된 네 지표 중 Held-out과 Reprojection은 계산했다. 기존 시뮬레이션 문서에서 Camera pose와 Registration은 독립 GT와 비교하는 정확도 지표다. 이번 실제 데이터에는 해당 GT가 없어 이 두 정확도는 **N/A**로 남겼다. 대신 GT 없이 계산 가능한 카메라 간 registration 내부 일관성을 별도 보조 지표로 보고한다. 이를 요청 정확도 지표의 대체값으로 취급하지 않는다.

본 레포트의 기준 결과는 [통합 분석 JSON](../results/real_260910_unified_20260917_v2/analysis.json)이다.

## 2. 재수행 배경과 기존 결과와의 차이

| 항목 | 기존 9월 10일 분석 | 이번 재수행 |
| --- | --- | --- |
| 데이터 선택 | 코너 수·PnP 오차 등의 기준으로 event 선별 | 사용할 수 있는 변환 입력이 있으면 포함, 품질 기준으로 제외하지 않음 |
| 세션 처리 | 카메라 쌍별 세션을 각각 분석 | 동일 카메라의 관측을 세션 전체에서 모아 추정 |
| 평가 | LOO 위치 중앙값·P90, 카메라별 mount 차이 | 공통 고정 held-out 위치·회전 잔차, pixel RMSE, 별도 registration 일관성 |
| 최종 추정 | 선별한 세션별 입력 | 사용 가능한 전체 216개 관측으로 방법별 cam0·1·3 한 벌 |
| 결과 보존 | 기존 파일 | 기존 파일 유지, 새 결과 폴더에 별도 저장 |

기존 보고서는 [260910 calibration 보고서](../reports/260910_calibration/README.md)다. 기존 cam1·3 Shah의 LOO 위치 중앙값 평균 1.32 mm와 이번 통합 held-out 평균 7.471 mm는 입력 모집단, 세션 구성, 분할 방식, 집계 방식이 다르다. 따라서 두 숫자의 차이를 같은 조건에서의 성능 악화로 단정하지 않는다.

[재촬영 분석 문서](real_recapture_evaluation_20260910.md)는 9월 5·6일 촬영을 다루는 별도 분석이다. 이번 9월 10일 통합 결과와 혼합하지 않았다. 제공된 다른 파이프라인 설명문의 A0–A5/B1–B3, cube 평가 및 pixel joint optimization 결과도 이번 실행 결과로 인용하지 않았다.

## 3. 입력 데이터와 포함·제외 정책

### 3.1 원본 세션

| 세션 | 촬영 카메라 | 원본 event 수 | 유지 event 수 |
| --- | --- | ---: | ---: |
| [cam0-1_02](../datasets/260910/cam0-1_02/meta.json) | cam0·1 | 30 | 30 |
| [cam0-3_01](../datasets/260910/cam0-3_01/meta.json) | cam0·3 | 40 | 40 |
| [cam1-3_02](../datasets/260910/cam1-3_02/meta.json) | cam1·3 | 47 | 47 |
| 합계 | cam0·1·3 | 117 | 117 |

event는 한 번의 촬영 기록이며, camera 관측은 그 event 안의 카메라별 기록이다. 한 카메라의 입력이 사용할 수 없더라도 다른 카메라의 입력이 있으면 event 전체를 버리지 않았다.

| 카메라 | 원본 관측 | 사용 관측 | 입력 불가 제외 | Train 관측 | Held-out 관측 |
| --- | ---: | ---: | ---: | ---: | ---: |
| cam0 | 70 | 59 | 11 | 44 | 15 |
| cam1 | 77 | 75 | 2 | 57 | 18 |
| cam3 | 87 | 82 | 5 | 62 | 20 |
| 합계 | 234 | 216 | 18 | 163 | 53 |

### 3.2 실제 적용한 정책

캘리브레이션에 필요한 robot 변환과 camera-board 변환이 유한한 정상 형태의 강체변환인지 확인했다. 이번 제외 사유는 18개 모두 `nonfinite_or_wrong_shape_transform`이다. 이는 코너 수가 적거나 오차가 크다는 이유의 품질 선별과 구분된다. 원본 기록을 삭제한 것이 아니라 계산 입력에서 제외하고 사유를 기록했다.

다음 항목은 포함 여부를 결정하는 필터로 사용하지 않았다.

- 임의의 최소 코너 수(예: 12개)
- 저장된 PnP reprojection 오차의 상한
- `capture_gate.pass`, ChArUco `ok` 플래그
- 다른 marker ID 검출 기록
- 회전 다양성·포즈 간격에 따른 관측 제거

실제로 저장 코너 수가 12개 미만인 관측 **8개**, `capture_gate.pass=false`인 관측 **90개**가 포함됐다. 근접 포즈를 묶는 절차는 평가 분할을 위한 것이며 관측을 제거하지 않았다. 별도 robust loss, outlier 제거, 합성 노이즈도 적용하지 않았다. OpenCV 알고리즘 내부의 수학적 처리까지 변경한 실험은 아니다.

관측별 포함 여부와 이유는 [audit.json](../results/real_260910_unified_20260917_v2/audit.json)에 보존되어 있다.

### 3.3 촬영 설정과 좌표계

- 고정카메라 serial: cam0 `039422061216`, cam1 `319522062138`, cam3 `912322060991`.
- 카메라별 기존 intrinsic `K`, distortion `D`를 고정했다. serial과 해상도 1280×720의 일치를 검사했다. 단, 촬영 당시 intrinsic 파일의 해시는 없어 현재 파일과 촬영 당시 파일의 동일성을 독립적으로 증명하지는 못한다.
- 보드: 9×6 squares, square 25 mm, marker 18 mm, `DICT_4X4_250`, marker ID 90–116, legacy pattern. 세 세션의 보드 설정이 일치한다.
- 변환 표기는 `T_destination_source`, 행렬의 이동 단위는 metre다.
- 문서화된 robot 입력은 tool1/flange 기준이며, 위치 mm 및 각도 `[rz, ry, rx]` degree, 회전 구성은 `Rz @ Ry @ Rx`다. event별 tool 값은 null이므로 프레임을 개별 기록으로 재확인할 수는 없다.

캘리브레이션에는 저장된 `T_cam_board_4x4`를 그대로 사용했다. 이미지에서 코너를 재검출한 것은 pixel 평가용이며, 이를 이용해 PnP 입력을 다시 맞추거나 교체하지 않았다.

## 4. 세션 통합과 계산 방법

### 4.1 “카메라별 결과 한 벌”의 의미

cam0는 cam0·1 및 cam0·3 세션의 관측을 합쳤고, cam1은 cam0·1 및 cam1·3 세션을, cam3는 cam0·3 및 cam1·3 세션을 합쳤다. 따라서 같은 카메라에 세션별로 다른 결과를 두는 대신, 알고리즘마다 cam0·1·3의 `T_base_camera`를 한 벌로 저장했다.

서로 다른 시각의 두 카메라 촬영을 가상의 동시 3-camera event로 합친 것은 아니다. 각 관측은 원래 robot pose와 연결되어 있으며, event ID는 `session:event_id`로 구분한다.

통합은 세션 사이에 카메라 설치 위치와 flange-board 부착 관계가 유지됐다는 가정 아래 수행했다. 장치 serial과 보드 설정의 일치는 확인했지만, 물리적으로 움직이지 않았다는 사실은 metadata만으로 입증하지 못한다.

또한 이번 통합은 **동일 카메라의 입력 세션을 합치는 방식**이다. 세 카메라를 하나의 pixel 목적함수로 joint optimization하거나, 모든 카메라에 하나의 공통 board mount를 강제한 것은 아니다. 각 카메라는 독립적으로 추정하고 카메라별 mount도 저장한다.

### 4.2 알고리즘 범위

| 방법 | 이번 구현에서의 추정 방식 |
| --- | --- |
| Shah | Robot-world/hand-eye 관계 `A_e X_i = Y_i B_ie`에서 카메라 외부변환과 board mount를 추정 |
| Tsai / Park / Horaud / Andreff / Daniilidis | OpenCV hand-eye 방법을 eye-to-hand convention으로 연결해 카메라 외부변환을 추정하고, 해당 학습 입력에서 board mount를 계산 |

구현은 [unified_real_evaluation.py](../SOTA_Simulation/unified_real_evaluation.py)와 기존 solver/helper를 사용한다. Shah와 다른 5개 방법의 robot 입력 반전 convention 차이는 기존 wrapper 및 합성 self-test로 확인한다.

이번 완료 범위는 위 **6개 방법**이다. Tabb & Ahmad Yousef, Allegro, Ha, Generalized RWHEC를 포함한 최초 목록의 전체 10개 방법 실험을 완료했다는 뜻은 아니다.

### 4.3 전체 입력 추정과 held-out 평가의 분리

두 종류의 추정을 별도로 수행했다.

1. **평가용 train 추정:** cam0 44개, cam1 57개, cam3 62개 관측으로만 fit하고, 제외해 둔 held-out에서 평가했다. board mount 추정에도 held-out을 쓰지 않았다.
2. **전체 입력 추정:** cam0 59개, cam1 75개, cam3 82개를 모두 사용해 최종 검토용 변환을 생성했다. 이 변환으로 계산한 in-sample 값을 held-out 성능으로 보고하지 않았다.

117개 event는 근접 robot pose 기준(이동 1.5 mm 미만 및 회전 1° 미만)으로 115개 그룹에 묶었다. 시간순으로 정렬한 그룹의 매 네 번째 그룹을 held-out으로 고정했다. 결과는 Train 89 event / Held-out 28 event이며, 사용 가능한 camera 관측으로는 163 / 53개다. 같은 event 및 같은 근접 그룹은 양쪽에 갈라지지 않는다. 이 문턱값은 입력 선별 기준이 아니라 분할 누출 방지 기준이다.

모든 방법에 동일한 [split.json](../results/real_260910_unified_20260917_v2/split.json)을 사용했다. 한 번의 고정 분할 결과이며, 반복 교차검증 평균이나 신뢰구간은 아니다.

## 5. 평가지표 정의

다음 기호를 사용한다.

```text
A_e  = T_base_gripper(e)      : 기록된 robot pose
B_ie = T_camera_i_board(e)    : 기록된 PnP board pose
Y_i  = T_base_camera_i        : train에서 추정한 카메라 pose
X_i  = T_gripper_board        : train에서 추정한 카메라별 board mount
```

### 5.1 Held-out

held-out의 두 경로 `A_e X_i`와 `Y_i B_ie`를 비교한다. 이동 성분 차이의 Euclidean norm을 mm로, 상대 회전의 각도를 degree로 계산한다. 전체 위치·회전 값은 **53개 event-camera 관측의 산술평균**이다. 카메라마다 같은 가중치를 주는 macro 평균은 아니다.

이는 학습에 사용하지 않은 관측에서의 **내부 chain 일관성 잔차**다. 실제 board pose의 독립 GT와 비교한 절대 위치 오차는 아니다. 시뮬레이션 문서의 GT 기반 held-out 값과 동일한 정확도 정의라고 주장하지 않는다.

### 5.2 Camera pose와 Registration 정확도

[기존 Shah 평가 문서](shah_calibration_evaluation.md)의 Camera pose는 추정 외부변환과 GT 외부변환의 오차이며, Registration은 추정 카메라 간 상대변환과 GT 상대변환의 오차다. 이번 데이터에는 독립적인 외부변환·상대변환 GT가 없으므로 위치·회전 정확도는 모두 N/A(`null`)다. N/A는 0 또는 계산 성공을 뜻하지 않는다.

**카메라 pose 자체가 없는 것은 아니다.** 실제 추정한 4×4 `T_base_camera`와 `T_gripper_board`는 [full_data_estimates.json](../results/real_260910_unified_20260917_v2/full_data_estimates.json)에 있다. 없는 것은 그 추정값의 정확도를 측정할 독립 정답이다.

Registration을 팀에서 GT 정확도가 아니라 실제 관측 간 정합 잔차로 의미한 것이라면, 비교 대상과 집계 정의를 합의해야 한다. 본 보고서는 현재 GT 기반 정확도와 아래 보조 일관성을 분리하여 보여준다.

### 5.3 Reprojection

`Bhat_ie = inverse(Y_i) A_e X_i`로 held-out camera-board pose를 예측한 뒤, 실제 보드의 3D 코너를 고정 `K_i`, `D_i`로 원본 왜곡 영상 좌표에 투영한다. 관측 코너와의 2D 거리로 계산한다.

```text
RMSE_px = sqrt( sum_k[(u_hat_k-u_k)^2 + (v_hat_k-v_k)^2] / N_corners )
N_corners = 1,374
```

53개 held-out 관측 전부에서 평가 코너를 확보했고, 방법마다 동일한 1,374개 코너를 사용했다. 전체 값은 **corner-weighted RMSE**이며 프레임별 RMSE의 단순평균이 아니다. 분모를 `2N`으로 두는 좌표별 RMSE와도 다르다. 코너 수는 cam0 360개, cam1 511개, cam3 503개다. 큰 오차 관측도 알고리즘별로 제거하지 않았다.

### 5.4 보조: Registration consistency

같은 원본 event에서 두 카메라의 board pose를 모두 사용할 수 있을 때, `inverse(Y_j) Y_i B_ie`로 카메라 i의 board pose를 j로 전달하여 `B_je`와 비교한다. 총 **25개 held-out 카메라 쌍 관측**의 이동·회전 잔차를 평균한다.

이는 카메라 간 내부 closure 검사이며 GT registration 정확도가 아니다. 두 카메라의 PnP 오차 등도 포함되고, 여러 경로에 공통으로 존재하는 systematic error는 작게 보일 수 있다. 같은 event ID라는 이유만으로 하드웨어 동기화 정확도까지 검증된 것은 아니다.

## 6. 결과

### 6.1 요청된 네 지표 형식의 전체 결과

| 방법 | Held-out 위치 평균 (mm) | Camera pose GT 오차 (mm) | Registration GT 오차 (mm) | Held-out Reprojection RMSE (px) |
| --- | ---: | ---: | ---: | ---: |
| Shah | 7.471 | N/A | N/A | 9.269 |
| Tsai | 13.236 | N/A | N/A | 19.105 |
| Park | 11.047 | N/A | N/A | 15.896 |
| Horaud | 11.103 | N/A | N/A | 15.968 |
| Andreff | 105.641 | N/A | N/A | 9105.530 |
| Daniilidis | 8.343 | N/A | N/A | 11.006 |

위 Held-out 열은 5.1절의 내부 잔차이며 GT 정확도는 아니다. 제공된 시뮬레이션 표의 노이즈 조건·GT·카메라 구성과 달라 수치를 직접 대조해 실세계 우열을 결론내리지 않는다.

### 6.2 회전 잔차와 registration 보조 지표

| 방법 | Held-out 회전 평균 (deg) | Registration consistency 평균 (mm) | Registration consistency 평균 (deg) |
| --- | ---: | ---: | ---: |
| Shah | 1.547 | 19.603 | 3.267 |
| Tsai | 1.738 | 31.296 | 3.826 |
| Park | 1.546 | 30.632 | 3.248 |
| Horaud | 1.547 | 30.237 | 3.266 |
| Andreff | 1.564 | 237.865 | 3.311 |
| Daniilidis | 1.599 | 24.478 | 3.128 |

### 6.3 카메라별 held-out 결과

아래 결과도 평가용 train 추정에서 나온 값이다. 카메라별 held-out 관측 수는 모든 방법에서 cam0 15개, cam1 18개, cam3 20개로 같다.

| 방법 | 카메라 | 위치 평균 (mm) | 회전 평균 (deg) | Reprojection RMSE (px) |
| --- | --- | ---: | ---: | ---: |
| Shah | cam0 | 1.278 | 0.310 | 1.751 |
| Shah | cam1 | 2.652 | 0.556 | 2.214 |
| Shah | cam3 | 16.451 | 3.366 | 15.083 |
| Tsai | cam0 | 1.214 | 0.312 | 2.248 |
| Tsai | cam1 | 5.645 | 0.697 | 7.862 |
| Tsai | cam3 | 29.083 | 3.745 | 30.506 |
| Park | cam0 | 1.218 | 0.311 | 1.765 |
| Park | cam1 | 4.844 | 0.557 | 7.233 |
| Park | cam3 | 24.001 | 3.363 | 25.196 |
| Horaud | cam0 | 1.224 | 0.310 | 1.775 |
| Horaud | cam1 | 4.836 | 0.556 | 7.238 |
| Horaud | cam3 | 24.153 | 3.367 | 25.318 |
| Andreff | cam0 | 7.647 | 0.311 | 10.831 |
| Andreff | cam1 | 27.249 | 0.552 | 32.501 |
| Andreff | cam3 | 249.689 | 3.414 | 15049.203 |
| Daniilidis | cam0 | 1.196 | 0.312 | 2.017 |
| Daniilidis | cam1 | 4.075 | 0.608 | 2.524 |
| Daniilidis | cam3 | 17.544 | 3.456 | 17.930 |

표의 표시값은 소수점 이하 3자리로 반올림했으며, 반올림 전 값은 결과 JSON/CSV에 저장되어 있다.

## 7. 해석과 남은 확인 사항

1. **이번 전체 위치·pixel 평가에서는 Shah가 가장 낮은 오차를 보였다.** Daniilidis가 뒤를 이었고 Park와 Horaud는 가까운 값이었다. 다만 방법의 일반적 우월성, 외부 GT 순위, 전체 작업 영역에서의 정확도를 증명한 것은 아니다. 회전의 최솟값은 지표에 따라 다르다.
2. **cam3는 공통으로 확인해야 할 부분이다.** Shah에서도 cam0 1.278 mm / cam1 2.652 mm에 비해 cam3는 16.451 mm였다. 다른 모든 방법에서도 cam3의 위치·pixel 오차가 가장 컸다. 저장 PnP, intrinsic, robot-image 대응, 세션 사이의 고정 관계 등은 확인 후보지만, 이 결과만으로 원인을 특정하지 않았다.
3. **Andreff는 계산값을 반환했지만 큰 오차를 보였다.** 특히 cam3 reprojection은 15049.203 px였다. 계산 성공 플래그와 이용 가능한 정확도는 별개이며, 나쁜 값을 평균에서 제외하지 않고 보고했다.
4. **요청된 GT 정확도 평가는 미완이다.** Camera pose 및 Registration의 GT 비교에는 독립 기준이 필요하다. 내부 일관성이 좋아도 절대 정확도가 보장되지는 않는다.
5. **세션 통합의 물리 가정은 확인이 필요하다.** 실제 설치 위치·board mount가 변했다면 단일 고정변환으로 모든 세션을 설명한다는 전제에 맞지 않는다. 통합하면 좋아진다고 가정하지 않고 작업 기록 등으로 확인해야 한다.
6. **이번 요청에서는 레포트만 작성하고 추가 수정이나 재촬영을 하지 않았다.** 외부 GT 측정, cam3 원인 진단, 미실행 4개 방법, 3-camera joint optimization은 별도 작업으로 구분한다.

회의에서 확인할 사항은 (a) Registration을 GT 정확도와 관측 간 정합 잔차 중 무엇으로 정의할지, (b) 세 세션에서 고정 관계가 유지됐는지, (c) 독립 GT를 어떻게 확보할지다. 현 단계에서는 Shah를 내부 평가상 검토 후보로 두며, 로봇 적용을 위한 확정값으로 취급하지 않는다.

## 8. 저장된 산출물과 검증

기준 폴더: `results/real_260910_unified_20260917_v2/`

| 파일 | 내용 |
| --- | --- |
| [analysis.json](../results/real_260910_unified_20260917_v2/analysis.json) | 전체 지표, 입력 수, 정의, 가정, 실행 환경과 provenance |
| [full_data_estimates.json](../results/real_260910_unified_20260917_v2/full_data_estimates.json) | 전체 216개 관측을 사용한 방법별 cam0·1·3 변환 한 벌 |
| [prepared.json](../results/real_260910_unified_20260917_v2/prepared.json) | 확정 입력, intrinsic, 평가 코너, 입력·구현 해시 |
| [audit.json](../results/real_260910_unified_20260917_v2/audit.json) | 원본 234개 관측의 포함·제외 감사 기록 |
| [split.json](../results/real_260910_unified_20260917_v2/split.json) | 공통 train/held-out 및 근접 그룹 |
| `shah.json` 등 방법별 JSON | train 추정·전체 입력 추정, train/held-out 원시 지표 |
| [summary.csv](../results/real_260910_unified_20260917_v2/summary.csv) | 전체 비교표의 반올림 전 값 |
| [per_camera.csv](../results/real_260910_unified_20260917_v2/per_camera.csv) | 카메라별 비교표의 반올림 전 값 |
| [event_metrics.csv](../results/real_260910_unified_20260917_v2/event_metrics.csv) | event-camera 단위 train/held-out 잔차 |
| [registration_pairs.csv](../results/real_260910_unified_20260917_v2/registration_pairs.csv) | held-out 카메라 쌍의 closure 잔차 |

실행 환경은 Python 3.13.7 / OpenCV 4.14.0 / NumPy 2.5.3 / SciPy 1.18.1이다. 분석 기록의 기준 commit은 `830a404e71a23ef2bce1e8b11c5bcdc978d9bc4a`이며 실제 구현 파일 해시도 기록했다. 기준 commit만으로 실행 당시 추가 코드까지 표현한다고 해석하지 않는다.

재계산 완료 시 다음을 확인했다.

- unified 및 기존 real evaluation 테스트: **23 passed**.
- 원본 metadata의 포함 판정과 대조하여 사용 관측 216개 및 공통 split의 일치를 확인했다.
- 전체 입력 18개 fit의 성공, 전 방법의 held-out 53관측·1,374코너 평가를 확인했다.
- 원본 metadata를 읽고 chain 및 pixel 집계를 독립적으로 재계산해 결과와 대조했다.
- 입력 metadata SHA-256이 유지되며 기존 추적 대상 데이터·결과 파일을 변경하지 않았음을 확인했다.

중단 후 첫 준비 폴더 `results/real_260910_unified_20260917/`는 완료된 평가 결과가 아니다. Windows의 한글 포함 경로에서 이미지 읽기를 바이트열 decode 방식으로 대응한 뒤 생성한 `_v2`를 본 레포트의 완성 결과로 사용했다. 원본 이미지·PnP·intrinsic을 수정한 대응은 아니다.

## 9. 재현·분할 실행

저장소 루트에서 기존 `.venv-unified`를 사용한다. 새 출력 이름을 선택하고 CPU 부하를 줄여 방법별로 순차 실행한다. runner는 OpenCV도 1 thread로 설정하며 방법별 결과 파일이 있으면 덮어쓰기를 거부한다.

```powershell
$env:OPENBLAS_NUM_THREADS = '1'
$env:OMP_NUM_THREADS = '1'
$env:MKL_NUM_THREADS = '1'
$runOutput = 'results/real_260910_unified_NEW_RUN'

.\.venv-unified\Scripts\python.exe -m SOTA_Simulation.unified_real_evaluation --output $runOutput --method prepare
if ($LASTEXITCODE -ne 0) { throw 'prepare failed' }

foreach ($calibrationMethod in @('shah', 'tsai', 'park', 'horaud', 'andreff', 'daniilidis')) {
    .\.venv-unified\Scripts\python.exe -m SOTA_Simulation.unified_real_evaluation --output $runOutput --method $calibrationMethod
    if ($LASTEXITCODE -ne 0) { throw "calculation failed: $calibrationMethod" }
}

.\.venv-unified\Scripts\python.exe -m SOTA_Simulation.unified_real_evaluation --output $runOutput --method collect
if ($LASTEXITCODE -ne 0) { throw 'collect failed' }
```

중단 후에는 준비된 폴더에서 아직 JSON이 저장되지 않은 방법만 실행한다. 전체 6개 방법이 모이면 `collect`한다. 입력 해시가 변하면 중단하므로 서로 다른 입력 결과가 혼합되지 않는다.

테스트 재확인 명령:

```powershell
.\.venv-unified\Scripts\python.exe -m pytest -q tests/test_unified_real_evaluation.py tests/test_real_evaluation.py
```
