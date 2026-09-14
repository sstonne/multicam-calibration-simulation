# 2026-09-10 실제 캘리브레이션 실행 결과

슬랙에서 요청된 9월 10일 촬영 데이터의 캘리브레이션을 실행했다. 품질평가에서 고정한 event만 사용했으며, 원본 이미지와 `meta.json`은 변경하지 않았다.

## 결론

`cam1-3_02`의 유효 34 pose로 계산한 Shah 결과를 1차 후보로 저장했다. 이 세션은 모든 알고리즘에서 leave-one-out 위치 잔차가 비슷했고, 여섯 방법이 모두 실패 없이 계산됐다. Shah는 현재 문제의 `A X = Y B`를 직접 풀면서 `T_base_camera`와 `T_gripper_board`를 함께 추정하므로 기준 결과로 선택했다.

| 세션 | 방법 | 두 카메라 LOO 위치 중앙값 평균 | 두 카메라 LOO P90 평균 | camera별 mount 차이 평균 |
| --- | --- | ---: | ---: | ---: |
| cam0-1_02 | Shah | 2.64 mm | 4.71 mm | 3.87 mm |
| cam1-3_02 | **Shah** | 1.32 mm | 2.02 mm | 2.20 mm |
| cam1-3_02 | Tsai | 1.36 mm | 2.23 mm | 2.94 mm |
| cam1-3_02 | Park | 1.32 mm | 2.18 mm | 3.00 mm |
| cam1-3_02 | Horaud | 1.31 mm | 2.19 mm | 3.01 mm |
| cam1-3_02 | Andreff | 1.49 mm | 2.77 mm | 3.20 mm |
| cam1-3_02 | Daniilidis | **1.23 mm** | 2.13 mm | 2.22 mm |
| cam0-3_01 | Shah | 2.62 mm | 5.01 mm | 1.85 mm |

LOO 수치는 제외한 pose를 calibration과 board-mount 추정 양쪽에서 모두 빼고 계산한 내부 일관성 잔차다. 절대 위치 ground truth 오차가 아니다. Daniilidis의 중앙값이 근소하게 낮지만 Shah 대비 우위가 작고, 이 데이터만으로 알고리즘 우승을 선언하지 않는다.

## 산출물

- `cam1-3_02_shah_candidate.json`: cam1과 cam3의 1차 calibration 후보. 34개 event ID, serial, 보드 설정, `T_base_camera`, `T_gripper_board`, LOO 검증 포함.
- `calibration_results.json`: 세 세션에 Shah, Tsai, Park, Horaud, Andreff, Daniilidis를 모두 실행한 전체 결과.
- `../260910_quality/event_selection.json`: calibration 입력으로 사용한 고정 event 목록.

두 JSON의 상태는 `provisional_not_deployed`다. 로봇 적용 전 독립적인 물리 기준점으로 절대 정확도를 확인해야 한다. `cam0-1_02`는 보완 촬영 권장, `cam0-3_01`은 추가 촬영 전 참고값으로만 사용한다.

## 재실행

```powershell
.\.venv-real\Scripts\python.exe reports/260910_quality/calibrate_algorithms.py
```

실행기는 평가 당시 SHA-256을 LF 기준으로 확인하므로 Windows의 CRLF 체크아웃은 허용하지만, metadata 내용이 바뀌면 중단한다. Shah convention 합성 self-test도 계산 전에 수행한다.
