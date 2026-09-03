# ZEUS-RealSense Shah 실험: 수집 준비 상태 감사

작성일: 2026-09-03  
범위: 코드 변경이나 하드웨어 실행 없이 수행한 정적 감사. 이 문서는 팀 전달 및 백업용으로, 실제 수집 원본 프로젝트와 SOTA 비교 프로젝트의 역할을 분리해 기록한다.

## 1. 결론

Shah Robot-World/Hand-Eye 실험의 실제 수집 원본은 이 저장소의 `i611usr/Step2_capture_multi_cam.py`가 아니라 다음 PC 로컬 프로젝트다.

```text
/home/sprout/Desktop/**jiwoo/rb-ArucoCube_Robot_multi_calibration
```

이 원본 프로젝트의 `Step2_capture.py`는 로봇 socket 메시지와 다중 RealSense 프레임을 하나의 `meta.json` event에 병합한다. 따라서 Shah용 실제 데이터는 원본 프로젝트의 세션 형식을 기준으로 준비한다. 현재 저장소 `multicam-calibration-simulation`은 SOTA solver 비교 및 평가 계약을 위한 저장소이며, `i611usr/`는 SSH 로봇 코드의 참고용 복사본이다. `i611usr/` 내부 코드는 수정 대상이 아니다.

## 2. 물리 구성과 Shah 식

- 타깃: ZEUS gripper에 강체로 고정된 30 mm ArUco cube.
- 카메라: 작업 공간에 고정된 RealSense `cam0`, `cam1`, `cam3`.
- 별도 gripper camera: `cam2`; ChArUco board 관측을 위한 eye-in-hand 경로이므로 fixed-camera Shah 입력과 섞지 않는다.
- 변환 표기: `T_destination_source`.

Fixed camera `i`와 event `k`에 대해 Shah가 만족시킬 관계는 다음과 같다.

```text
T_base_gripper(k) @ T_gripper_cube
    = T_base_camera_i @ T_camera_i_cube(k)
```

`SOTA_Simulation/shah_solver.py`는 eye-to-hand에서 `T_base_gripper`와 `T_camera_board`를 반전하지 않고 전달하며, 주 출력으로 `T_base_fixed_i`, 부가 출력으로 `T_gripper_board`를 반환한다. `docs/shah_calibration_evaluation.md`의 변환 규약도 동일하다.

## 3. 권위 있는 수집 경로

### PC 로컬 수집 프로젝트

| 역할 | 파일 | 근거 |
| --- | --- | --- |
| 수집 진입점 | `Step2_capture.py` | 로봇 수동 socket 모드와 `meta.json` 기록을 구현한다. |
| RealSense 스트림 | `camera.py` | camera별 frame ring buffer, `get_at(target_ts)` software sync를 구현한다. |
| cube/PnP | `aruco_cube.py` | marker 검출, cube 3D-2D correspondence, PnP를 구현한다. |
| cube 정의 | `config.py` | cube geometry의 single source of truth다. |
| robot pose 변환 | `robot_comm.py` | `[x,y,z,rz,ry,rx]`을 4x4 transform으로 변환한다. |
| intrinsics/장치 매핑 | `intrinsics/cam*.npz`, `intrinsics/device_map.json` | K, D, 해상도, serial, camera index 역할을 저장한다. |

### SOTA 비교 프로젝트

| 역할 | 파일 |
| --- | --- |
| Shah wrapper | `SOTA_Simulation/shah_solver.py` |
| synthetic 평가 계약 | `docs/shah_calibration_evaluation.md` |
| SSH 로봇 코드 참고본 | `i611usr/` |

## 4. 로봇 pose 계약

원본 수집기는 robot server가 보낸 `capture_gripper_pose_6dof`를 event record에 저장한다. `robot_comm.euler_deg_to_matrix()`의 입력은 다음과 같다.

```text
[x_mm, y_mm, z_mm, rz_deg, ry_deg, rx_deg]
```

- translation은 mm 입력을 metre로 변환한다.
- rotation은 `Rz(rz) @ Ry(ry) @ Rx(rx)`이다.
- 결과는 `robot_pose_matrix_4x4` 및 `capture_gripper_pose_matrix_4x4`로 저장된다.
- Shah 입력에는 이 matrix를 `T_base_gripper(k)`로 사용한다. 단, robot server가 이 필드에 실제 flange(tool1) pose를 보냈다는 운영 규약을 캡처 전에 확인해야 한다.

참고용 `i611usr/c1.py`와 `i611usr/robot_calb/c2.py`는 tool1(0 offset)으로 flange pose를 읽은 뒤 tool3으로 복귀해 전송한다. 반면 `i611usr/calibration.py`의 구 경로는 active TCP를 전송하므로, 동일 세션에서 섞어 쓰면 안 된다.

## 5. 카메라·타깃 계약

원본 `config.py`의 기본 cube 정의는 다음과 같다.

| 항목 | 값 |
| --- | --- |
| cube edge | 0.030 m |
| marker side | 0.022 m |
| dictionary | `DICT_4X4_50` |
| marker IDs | `0, 1, 2, 3, 4` |
| face | `+Z, +X, +Y, -X, -Y` |

`Step2_capture.py`는 accepted capture에서 camera별 RGB JPG와 aligned depth PNG를 저장한다. 각 camera record에는 RealSense color-frame timestamp `ts_ms`, marker IDs, marker별 PnP, cube PnP의 `T_cam_cube_4x4`, reprojection/depth 품질 지표가 저장된다. 원본 `camera.py`의 `ts_ms`는 `color.get_timestamp()` 값이다.

다중 카메라는 hardware sync가 아니다. 수집기는 각 camera의 최신 timestamp 중 최솟값을 기준 timestamp로 잡고, 다른 camera ring buffer에서 가장 가까운 frame을 선택한다. `capture_span_ms`를 계산하고, 기본 허용 상한은 120 ms다. 실제 세션에서는 이 gate 결과를 event마다 검토해야 한다.

## 6. 현재 확인된 장치 및 intrinsic

원본 `intrinsics/device_map.json` 기준:

| camera | serial | 역할 | intrinsic size |
| --- | --- | --- | --- |
| cam0 | 314522062542 | fixed | 640x480 @ 15 fps |
| cam1 | 319522062138 | fixed | 640x480 @ 15 fps |
| cam2 | 752112070297 | gripper | 640x480 @ 15 fps |
| cam3 | 912322060991 | fixed | 640x480 @ 15 fps |

`Step2_capture.py`의 기본 stream도 640x480 @ 15 fps라 원본 intrinsics와 일치한다. SOTA 저장소에 복사된 intrinsics는 1280x720이므로, 원본 수집 세션의 K/D와 무심코 교체하면 안 된다.

## 7. meta.json event 데이터 계약

원본 `data/session/meta.json`에는 현재 57개 event가 있으며, event마다 다음 핵심 필드가 존재한다.

```text
event_id
capture_index
robot_pose_6dof
robot_pose_matrix_4x4
capture_gripper_pose_6dof
capture_gripper_pose_matrix_4x4
capture_robot_joints_6dof
set_cube_center_6dof
set_index
capture_span_ms
capture_gate
cams[cam_id].{rgb_path, depth_path, ts_ms, is_gripper,
              marker_ids, markers, cube_pnp, cube_visible}
```

`cube_pnp.T_cam_cube_4x4`가 fixed camera의 Shah visual input `T_camera_i_cube(k)`에 해당한다. RGB와 depth는 원본 파일로 남으므로 PnP/marker 결과는 재검출 및 재계산할 수 있다.

## 8. event ID와 재촬영 규칙

`event_id`는 로컬 수집기가 실제 저장에 성공했을 때만 증가한다. 로봇 `capture_index`는 메시지에 포함되어 그대로 event record에 함께 저장된다. capture gate가 실패하면 local event는 저장되지 않고 robot 측에는 `skipped` 응답이 돌아간다.

따라서 두 번호는 “항상 같은 정수”라는 가정으로 join하면 안 된다. 분석에서는 `meta.json` 한 event 안에 함께 저장된 `event_id`와 `capture_index`를 사용하고, 누락/재촬영은 `capture_gate.status`, `reason`, waypoint 기록으로 추적한다.

## 9. Shah 실행 전 체크리스트

- fixed camera `cam0`, `cam1`, `cam3`만 Shah dataset에 포함한다.
- 모든 선택 event에 `robot_pose_matrix_4x4`와 해당 fixed camera의 `cube_pnp.T_cam_cube_4x4`가 있는지 확인한다.
- `capture_gate.pass == true` 및 `capture_span_ms`가 실험에서 정한 동기화 한계 안인지 확인한다.
- robot pose가 실제 flange(tool1) 기준인지, 한 세션 전체에서 같은 tool-frame 규약인지 확인한다.
- session `cube_config`와 실제 물리 cube의 30 mm edge, 22 mm marker, ID/face orientation을 캘리퍼 및 육안으로 확인한다.
- session별 intrinsic과 stream resolution이 정확히 일치하는지 확인한다.
- calibration/held-out event split을 solver 실행 전에 고정한다.
- robot translation unit을 metre로 통일하고, `T_base_gripper` 및 `T_camera_cube`의 방향을 재확인한다.

## 10. 잔여 위험과 후처리

- Software sync는 hardware trigger가 아니므로 `capture_span_ms`만으로 robot-image 시간 오차가 완전히 측정되지는 않는다. robot capture timestamp가 meta에 없으므로, 높은 정확도가 필요하면 server-side capture timestamp 기록이 필요하다.
- 현재 capture code는 `cv2.imwrite()` 반환값을 검사하지 않고 `saved: true`를 기록한다. 촬영 후 경로 존재성 및 파일 decode 검증을 수행한다.
- Shah는 absolute robot pose에 직접 민감하다. robot FK/tool-frame error, thermal drift, gripper-to-cube rigidity, and image timing error를 held-out chain/reprojection 및 camera 간 `T_gripper_cube` spread로 검증한다.
- 이 저장소에는 실제 `meta.json`을 Shah 입력 리스트로 변환하는 loader가 없다. 후처리 단계에서 fixed camera별 event filtering, transform 단위/방향 검증, train/held-out split, OpenCV 입력 배열 구성이 필요하다.

## 11. 운영 원칙

1. 수집 원본 프로젝트와 `i611usr/` 참고 복사본은 수정하지 않는다.
2. SOTA 저장소에서는 원본 session을 입력으로 하는 별도 loader/runner만 추가한다.
3. 신규 세션의 `meta.json`, intrinsics, `device_map.json`, `capture_waypoints.json`, RGB/depth를 함께 백업한다.
4. Shah 결과에는 solver 입력 event 목록, camera serial/index, cube config, intrinsic file, capture-gate 통계를 함께 기록한다.
