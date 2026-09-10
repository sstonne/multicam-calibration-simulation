# 재촬영 데이터 정식 분석 결과 (2026-09-10)

## 결론

- 9월 4일 세션은 카메라 이동 전 백업으로 명시적으로 제외했다. fitting, held-out 평가, 방법 비교 어디에도 사용하지 않았다.
- 9월 5일 재촬영 두 세션은 각각 독립적으로 평가했다. 공통 품질 기준을 통과한 pose는 14/17개(cam0·1), 13/16개(cam0·3)다.
- 고정 held-out split에서 Tsai가 두 9월 5일 세션의 가장 낮은 pixel RMSE를 보였지만, held-out pose가 각 3개뿐이고 외부 ground truth가 없으므로 최종 우승 알고리즘으로 확정하지 않는다.
- 9월 6일 cam1·3 세션은 8 pose뿐이고 학습 pose의 평균 상대 회전이 2.63°다. 보조 확인용이며 9월 5일 결과와 직접 합치거나 동일 조건처럼 비교하지 않는다.

## 데이터 선택 및 분할

| 세션 | 용도 | raw → 사용 | 제외 event | train / held-out |
| --- | --- | ---: | --- | ---: |
| 20260905_134509 (cam0·1) | 본 재촬영 | 17 → 14 | 3, 6, 7 | 11 / 3 |
| 20260905_141521 (cam0·3) | 본 재촬영 | 16 → 13 | 5, 9, 11 | 10 / 3 |
| 20260906_170504_ddec38 (cam1·3) | 소회전 보조 | 8 → 8 | 없음 | 6 / 2 |

제외된 event에는 목표 보드 범위 밖 marker ID가 기록돼 있다. 이는 오검출 또는 다른 marker 혼입 가능성을 뜻하며, pose 자체가 틀렸다고 단정하는 근거는 아니다. 두 카메라에 동일한 event 집합을 쓰기 위해 보수적으로 제외했다. 근접·중복 robot pose는 먼저 같은 group으로 묶어 train/held-out에 갈라지지 않게 했다. split은 알고리즘 결과를 보기 전에 event 순서만으로 고정했다.

## Held-out 결과

수치는 두 카메라의 held-out event를 합쳐 계산한 평균 chain translation / 평균 chain rotation / corner-weighted pixel RMSE다.

| 세션 | 방법 | mm | deg | px |
| --- | --- | ---: | ---: | ---: |
| 09-05 13:45 | Shah | 1.665 | 0.487 | 2.359 |
|  | Tsai | **1.602** | 0.478 | **1.787** |
|  | Park | 1.769 | 0.487 | 2.013 |
|  | Horaud | 1.769 | 0.487 | 2.015 |
|  | Andreff | 8.954 | 0.490 | 23.584 |
|  | Daniilidis | 1.897 | **0.471** | 2.486 |
| 09-05 14:15 | Shah | 2.364 | 0.266 | 1.841 |
|  | Tsai | **2.281** | 0.266 | **1.697** |
|  | Park | 2.371 | 0.266 | 1.927 |
|  | Horaud | 2.374 | 0.266 | 1.932 |
|  | Andreff | 2.696 | 0.265 | 2.924 |
|  | Daniilidis | 2.398 | **0.264** | 2.452 |
| 09-06 보조 | Shah | **0.809** | 0.092 | **0.243** |
|  | Tsai | 실패 | 실패 | 실패 |
|  | Park | 1.151 | **0.092** | 1.305 |
|  | Horaud | 1.151 | **0.092** | 1.305 |
|  | Andreff | 11.616 | 0.216 | 16.714 |
|  | Daniilidis | 1.049 | 0.096 | 0.972 |

Tsai는 9월 6일 train set에서 유효한 회전 pair가 0개라 실패 처리했다. OpenCV Tsai 구현은 작은 회전 pair를 건너뛰고 유효 pair가 2개 미만이면 계산할 수 없다. 이 경우 로그만 남긴 채 항등 회전/영 이동이 반환될 수 있어 분석기가 사전 검사한다. 구현 근거: [OpenCV calibration_handeye.cpp](https://github.com/opencv/opencv/blob/4.x/modules/calib3d/src/calibration_handeye.cpp).

## 해석과 제한

- 9월 5일에서는 Tsai/Park/Horaud/Shah의 held-out 성능이 비슷하고 Andreff가 특히 첫 세션에서 크게 나쁘다.
- 낮은 held-out 오차만으로 extrinsic의 절대 정확도를 증명할 수 없다. 외부 GT가 없으며 동일한 PnP 관측으로 일관성을 평가한 값이다.
- Shah의 train pose-group 하나씩 제거 시 camera extrinsic translation 변화 최댓값은 첫 세션 cam0 4.72 mm, cam1 10.80 mm, 둘째 세션 cam0 6.41 mm, cam3 9.74 mm다. 9월 6일은 cam1 14.64 mm, cam3 16.52 mm로 더 민감하다.
- 9월 5일 metadata에는 원본 corner pixel이 없어 JPEG에서 다시 검출한 pixel을 **평가에만** 사용했다. solver 입력 PnP pose는 촬영 시 저장값을 유지했다.
- 촬영 metadata에 intrinsic hash가 없어, 현재 intrinsic 파일은 serial·해상도 및 실제 SHA-256으로 기록했지만 촬영 당시 파일과 byte 단위 동일함은 증명할 수 없다.

따라서 현재 권고는 9월 5일 Tsai를 우선 후보, Shah/Park/Horaud를 교차확인 후보로 두는 것이다. 로봇에 배포하기 전에는 더 넓은 회전·이동 범위의 독립 validation pose와 거리/좌표가 알려진 물리 target으로 절대 정확도를 확인해야 한다. `full_data_estimates.json`은 전 pose 임시 추정치이며 자동 배포 대상이 아니다.

## 재실행

```powershell
py -3.13 -m venv .venv-real
.\.venv-real\Scripts\python.exe -m pip install -r requirements-real-lock.txt
.\.venv-real\Scripts\python.exe -m pytest -q tests/test_real_evaluation.py tests/test_smoke.py
.\.venv-real\Scripts\python.exe -m SOTA_Simulation.real_evaluation --output results/real_recapture_NEW_RUN
```

입력 세션·제외 정책은 `datasets/real_analysis_manifest.json`, 전체 요약은 `results/real_recapture_20260910/summary.csv`, 세션별 audit·split·상세 metric·provisional transform은 각 결과 하위 디렉터리에 있다. 기존 결과 디렉터리는 덮어쓰지 않도록 분석기가 거부한다.
