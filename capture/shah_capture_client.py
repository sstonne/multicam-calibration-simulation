#!/usr/bin/env python3
"""PC측 촬영 세션 레코더 — 고정 RealSense 3대 + 로봇 팔 끝의 ChArUco 보드.

로봇측 상대 스크립트: capture/robot/shah_capture_server.py (Python 2, ZEUS 컨트롤러)

무엇을 하는가
  로봇이 한 자세에 멈춰 촬영을 요청하면, 세 카메라를 한꺼번에 잡아
  ChArUco 를 검출하고, 이미지·depth·로봇 자세·검출 결과를 하나의 레코드로
  묶어 meta.json 에 적는다. 사람이 손으로 저장하거나 이름을 붙이지 않는다.

왜 한 레코드로 묶는가
  calibration 의 입력은 이미지도 로봇 자세도 아니라 '둘의 짝'이다.

      T_base_gripper(k)  @  X  ==  Y_i  @  T_cam_i_board(k)
      (로봇에서)                     (이미지에서)  ← 같은 k 여야 한다

  7번 이미지에 6번 자세가 붙으면 알고리즘은 경고 없이 수렴하고 틀린 답을 낸다.
  파일명 규칙으로는 이 짝을 보장할 수 없으므로 원자적으로 함께 기록한다.

스키마
  jiwoo/rb-calibration-marker-experiment 의 meta.json 형식을 최대한 따른다.
  그 저장소의 Step3~Step5 와 CP_* 분석 코드가 전부 이 형식만 읽으므로,
  형식을 맞춰 두면 하류를 재사용할 수 있다. 큐브 전용 필드(cube_pnp,
  set_index, grasp_id 등)는 이 리그에 대응물이 없어 빠졌고, 대신
  board_config / foreign_marker_ids 가 추가됐다. 차이는
  docs/real_shah_capture.md 의 "meta.json 스키마" 절에 정리돼 있다.

규약 (어기면 결과가 조용히 틀어진다)
  * 로봇 자세는 tool 1(플랜지) 기준 [x,y,z mm, rz,ry,rx deg]
  * 회전은 Rz @ Ry @ Rx — jiwoo/robot_comm.py:euler_deg_to_matrix 와 동일
  * 저장되는 변환은 전부 T_destination_source, 병진 단위는 metre

실행 예
  python capture/shah_capture_client.py \
      --session-dir ~/shah_data/session01 \
      --robot-host 192.168.0.23 \
      --robot-board-id-start 50 \
      --show
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from itertools import combinations
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT))

from board_config import ROBOT_BOARD, TABLE_BOARD, BoardConfig

SCHEMA_VERSION = "shah_capture_v1"
DEFAULT_CAMERAS = ("cam0", "cam1", "cam3")   # 테이블 고정 3대
CAPTURE_BLOCK = "B_eyetohand"                # 보드가 gripper 에 고정 = eye-to-hand


# ────────────────────────────────────────────────────────────────
# 자세 변환 — jiwoo/robot_comm.py:euler_deg_to_matrix 와 동일해야 한다
# ────────────────────────────────────────────────────────────────

def zeus_pose_to_matrix(pose6) -> np.ndarray:
    """[x_mm, y_mm, z_mm, rz_deg, ry_deg, rx_deg] -> 4x4 (병진 단위 metre)."""
    x, y, z, rz, ry, rx = (float(v) for v in pose6)
    rx, ry, rz = np.deg2rad([rx, ry, rz])
    Rz = np.array([[np.cos(rz), -np.sin(rz), 0], [np.sin(rz), np.cos(rz), 0], [0, 0, 1]])
    Ry = np.array([[np.cos(ry), 0, np.sin(ry)], [0, 1, 0], [-np.sin(ry), 0, np.cos(ry)]])
    Rx = np.array([[1, 0, 0], [0, np.cos(rx), -np.sin(rx)], [0, np.sin(rx), np.cos(rx)]])
    T = np.eye(4)
    T[:3, :3] = Rz @ Ry @ Rx
    T[:3, 3] = np.array([x, y, z]) / 1000.0
    return T


def invert(T: np.ndarray) -> np.ndarray:
    out = np.eye(4)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return out


# ────────────────────────────────────────────────────────────────
# ChArUco
# ────────────────────────────────────────────────────────────────

class BoardDetector:
    """한 보드에 대한 검출기. 보드가 두 장이므로 항상 어느 보드인지 명시한다."""

    def __init__(self, board: BoardConfig, marker_id_start: int):
        self.board_config = board
        self.marker_id_start = int(marker_id_start)
        dictionary = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, board.dictionary_name))
        self.ids = np.arange(
            self.marker_id_start,
            self.marker_id_start + board.marker_count, dtype=np.int32).reshape(-1, 1)
        self.grid = cv2.aruco.CharucoBoard(
            (board.squares_x, board.squares_y),
            board.square_length_m, board.marker_length_m, dictionary, self.ids)
        self.detector = cv2.aruco.CharucoDetector(self.grid)
        self.id_set = {int(v) for v in self.ids.ravel()}

    def detect(self, image_bgr, K, D) -> dict:
        """검출 결과를 meta.json 의 cams[i]['charuco'] 형태로 반환한다."""
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        corners, corner_ids, _, marker_ids = self.detector.detectBoard(gray)

        seen = [] if marker_ids is None else sorted(int(v) for v in marker_ids.ravel())
        result = {
            "ok": False,
            "n_corners": 0 if corners is None else int(len(corners)),
            "reproj_error_px": None,
            "rvec": None,
            "tvec": None,
            "T_cam_board_4x4": None,
            "marker_ids": [i for i in seen if i in self.id_set],
            # 이 보드에 속하지 않는 마커. 테이블 보드가 함께 보이면 여기 쌓인다.
            # 비어 있지 않은데 ID 대역이 겹치는 상황이면 검출이 섞였을 수 있다.
            "foreign_marker_ids": [i for i in seen if i not in self.id_set],
            "_draw": (corners, corner_ids),
        }
        if corners is None or len(corners) < 4:
            return result

        object_points, image_points = self.grid.matchImagePoints(corners, corner_ids)
        if object_points is None or len(object_points) < 4:
            return result

        ok, rvec, tvec = cv2.solvePnP(
            object_points, image_points, K, D, flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            return result
        rvec, tvec = cv2.solvePnPRefineLM(object_points, image_points, K, D, rvec, tvec)

        projected, _ = cv2.projectPoints(object_points, rvec, tvec, K, D)
        residual = projected.reshape(-1, 2) - image_points.reshape(-1, 2)
        rmse = float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1))))

        T = np.eye(4)
        T[:3, :3] = cv2.Rodrigues(rvec)[0]
        T[:3, 3] = tvec.reshape(3)

        result.update({
            "ok": True,
            "reproj_error_px": rmse,
            "rvec": rvec.reshape(3).tolist(),
            "tvec": tvec.reshape(3).tolist(),
            "T_cam_board_4x4": T.tolist(),
        })
        return result


# ────────────────────────────────────────────────────────────────
# RealSense
# ────────────────────────────────────────────────────────────────

class Camera:
    def __init__(self, name, index, serial, K, D, width, height, fps, save_depth):
        import pyrealsense2 as rs
        self.rs = rs
        self.name = name
        self.index = index
        self.serial = serial
        self.K = K
        self.D = D
        self.save_depth = save_depth
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(serial)
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        if save_depth:
            config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        self.config = config
        self.align = rs.align(rs.stream.color) if save_depth else None
        self.photometry = None

    def start(self, warmup_frames=15, stagger_s=0.0):
        """USB 대역폭 경합을 피하려 순차 기동한다.

        2026-09-03 실측: 3대를 1280x720@15 로 동시에 열면 한 대가
        'Frame didn't arrive' 로 실패했으나 단독으로는 10/10 정상이었다.
        기동 간격과 해상도가 완화 수단이다.
        """
        if stagger_s:
            time.sleep(stagger_s)
        profile = self.pipeline.start(self.config)
        for _ in range(warmup_frames):          # 자동노출 수렴 대기
            try:
                self.pipeline.wait_for_frames(timeout_ms=5000)
            except Exception:
                break
        self._lock_photometry(profile)

    def _lock_photometry(self, profile):
        """자동노출을 수렴시킨 뒤 고정한다.

        촬영 도중 노출이 바뀌면 코너 검출 품질이 자세마다 달라져, 자세 오차와
        노출 변화가 뒤섞인다. 기존 파이프라인도 같은 이유로 잠근다.
        """
        try:
            sensor = profile.get_device().first_color_sensor()
            values = {
                "exposure_us": float(sensor.get_option(self.rs.option.exposure)),
                "gain": float(sensor.get_option(self.rs.option.gain)),
                "white_balance": float(sensor.get_option(self.rs.option.white_balance)),
            }
            sensor.set_option(self.rs.option.enable_auto_exposure, 0)
            sensor.set_option(self.rs.option.enable_auto_white_balance, 0)
            self.photometry = {"locked": True, "values": values,
                               "source": "auto_converged"}
        except Exception as error:
            self.photometry = {"locked": False, "values": None,
                               "source": f"lock_failed: {error}"}

    def grab(self, timeout_ms=5000):
        frames = self.pipeline.wait_for_frames(timeout_ms=timeout_ms)
        if self.align is not None:
            frames = self.align.process(frames)
        color = frames.get_color_frame()
        if not color:
            return None
        depth = frames.get_depth_frame() if self.save_depth else None
        return {
            "color": np.asanyarray(color.get_data()),
            "depth": np.asanyarray(depth.get_data()) if depth else None,
            "ts_ms": float(color.get_timestamp()),
            "host_monotonic_ts_ms": time.monotonic() * 1000.0,
            "device_ts_ms": float(color.get_frame_metadata(
                self.rs.frame_metadata_value.sensor_timestamp))
            if color.supports_frame_metadata(
                self.rs.frame_metadata_value.sensor_timestamp) else None,
        }

    def stop(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass


def open_cameras(intrinsics_dir: Path, names, width, height, fps, save_depth,
                 stagger_s):
    import pyrealsense2 as rs

    connected = {d.get_info(rs.camera_info.serial_number) for d in rs.context().devices}
    print(f"연결된 RealSense: {sorted(connected)}")

    cameras = []
    for name in names:
        npz_path = intrinsics_dir / f"{name}.npz"
        if not npz_path.exists():
            raise FileNotFoundError(f"intrinsics 없음: {npz_path}")
        payload = np.load(npz_path)
        serial = str(payload["serial"])
        if bool(payload["is_gripper"]):
            print(f"  [주의] {name} 은 intrinsics 에 is_gripper=True 로 기록돼 있다"
                  f" (과거 손목 카메라). 지금 테이블에 고정된 게 맞는지 확인하라.")
        if serial not in connected:
            raise RuntimeError(f"{name}(serial={serial}) 가 연결돼 있지 않다")
        stored = (int(payload["color_w"]), int(payload["color_h"]))
        if stored != (width, height):
            raise RuntimeError(
                f"{name}: intrinsics 해상도 {stored} != 촬영 해상도 {(width, height)}. "
                f"K 가 해상도에 종속이므로 그대로 쓰면 pose 가 틀어진다.")
        index = int("".join(c for c in name if c.isdigit()))
        cameras.append(Camera(
            name, index, serial,
            np.asarray(payload["color_K"], float),
            np.asarray(payload["color_D"], float).reshape(-1),
            width, height, fps, save_depth))
        print(f"  {name} (index {index}) <- serial {serial}")

    unused = sorted(connected - {c.serial for c in cameras})
    if unused:
        print(f"  [주의] 사용하지 않는 연결 장치: {unused}"
              f" — USB 대역폭을 나눠 쓰므로 뽑아두는 편이 안전하다")

    for order, camera in enumerate(cameras):
        camera.start(stagger_s=stagger_s if order else 0.0)
        print(f"  {camera.name}: 스트림 시작")
    return cameras


# ────────────────────────────────────────────────────────────────
# 세션 종료 직후 Shah 로 품질 확인
# ────────────────────────────────────────────────────────────────

def solve_and_report(records, camera_names):
    """실험실을 떠나기 전에 세션이 쓸 만한지 확인한다.

    실데이터에는 ground truth 가 없으므로, 카메라마다 독립적으로 구한
    X = T_gripper_board 가 서로 얼마나 일치하는지를 품질 대리 지표로 쓴다.
    보드는 물리적으로 하나뿐이니 세 추정치는 같아야 한다.
    """
    from scipy.spatial.transform import Rotation
    from SOTA_Simulation.shah_solver import solve_shah_eye_to_hand
    from SOTA_Simulation.sota_simulation import pose_error

    def usable(record):
        return all(record["cams"].get(str(n)) is not None
                   and record["cams"][str(n)].get("charuco", {}).get("T_cam_board_4x4")
                   for n in camera_names)

    indices = [int("".join(c for c in n if c.isdigit())) for n in camera_names]
    good = [r for r in records
            if all(r["cams"].get(str(i), {}).get("charuco", {}).get("T_cam_board_4x4")
                   for i in indices)]

    print("\n" + "=" * 66)
    print(f"세션 검증: 세 카메라 모두 검출된 자세 {len(good)} / 기록 {len(records)}")
    if len(good) < 3:
        print("자세가 3개 미만이라 Shah 를 풀 수 없다.")
        return None

    robot = [np.asarray(r["robot_pose_matrix_4x4"], float) for r in good]
    angles = [np.degrees(np.linalg.norm(
                  cv2.Rodrigues((invert(a) @ b)[:3, :3])[0]))
              for a, b in combinations(robot, 2)]
    diversity = float(np.mean(angles))
    verdict = "충분" if diversity >= 40 else "부족 — 더 기울여 재촬영 권장"
    print(f"자세 간 평균 상대회전: {diversity:.1f}deg  ({verdict})")

    estimates, boards = {}, []
    for name, index in zip(camera_names, indices):
        visual = [np.asarray(r["cams"][str(index)]["charuco"]["T_cam_board_4x4"], float)
                  for r in good]
        result = solve_shah_eye_to_hand(robot, visual, method="SHAH")
        estimates[name] = result.T_base_fixed_i
        boards.append(result.T_gripper_board)

    mean_board = np.eye(4)
    mean_board[:3, :3] = Rotation.from_matrix(
        np.stack([b[:3, :3] for b in boards])).mean().as_matrix()
    mean_board[:3, 3] = np.mean([b[:3, 3] for b in boards], axis=0)
    spread = [pose_error(b, mean_board) for b in boards]
    spread_mm = float(np.mean([s[0] for s in spread]))
    spread_deg = float(np.mean([s[1] for s in spread]))

    print(f"\nX = T_gripper_board (보드가 플랜지에 붙은 위치, 세 카메라 공통이어야 함)")
    print(f"  평균 translation : {np.round(1000 * mean_board[:3, 3], 2)} mm")
    print(f"  카메라 간 흩어짐 : {spread_mm:.2f} mm / {spread_deg:.3f} deg")
    print(f"  => 예상 extrinsic 오차 대략 {spread_mm * 5.4:.1f} mm")
    print(f"     (시뮬레이션에서 얻은 5.4배 계수. 이 리그 기하 기준이며 절대값이 아니다)")
    print("\nY = T_base_camera (주 출력)")
    for name in camera_names:
        print(f"  {name}: t = {np.round(1000 * estimates[name][:3, 3], 1)} mm")
    print("=" * 66)

    return {
        "rotation_diversity_deg": diversity,
        "usable_pose_count": len(good),
        "T_gripper_board_mean_4x4": mean_board.tolist(),
        "T_gripper_board_spread_mm": spread_mm,
        "T_gripper_board_spread_deg": spread_deg,
        "T_base_camera_4x4": {n: estimates[n].tolist() for n in camera_names},
        "note": "현장 확인용 요약. 정식 결과는 Step3 에 해당하는 분석 단계에서 산출한다.",
    }


# ────────────────────────────────────────────────────────────────

def build_session_header(args, cameras, detector, session_dir):
    return {
        "schema_version": SCHEMA_VERSION,
        "root_folder": str(session_dir),
        "session_allocation": None,
        "gripper_cam_idx": None,          # 이 리그에는 손목 카메라가 없다
        "n_fixed_cams": len(cameras),
        "n_gripper_cams": 0,
        "cam_indices": [c.index for c in cameras],
        "cam_serials": {str(c.index): c.serial for c in cameras},
        "board_config_source": "capture/board_config.py:ROBOT_BOARD",
        "board_config": {**detector.board_config.to_dict(),
                         "marker_id_start": detector.marker_id_start},
        "table_board_config": TABLE_BOARD.to_dict(),
        "capture_config": {
            "schema_version": "capture_config_v1",
            "intrinsics_dir": str(Path(args.intrinsics_dir).resolve()),
            "width": args.width,
            "height": args.height,
            "fps": args.fps,
            "save_depth": bool(args.save_depth),
            "settle_time_s": args.settle_time_s,
            "cross_camera_timestamp_basis": "host_monotonic_receipt_v1",
            "min_charuco_corners": args.min_corners,
            "max_charuco_reproj_px": args.max_reproj_px,
            "robot_pose_convention":
                "tool 1 (flange), [x,y,z mm, rz,ry,rx deg], R = Rz@Ry@Rx",
            "transform_convention": "T_destination_source, translation in metre",
            "color_photometry": {str(c.index): c.photometry for c in cameras},
        },
        "captures": [],
    }


def capture_once(cameras, detector, message, session_dir, args):
    """촬영 1회. meta.json 의 captures[] 에 들어갈 레코드를 만든다."""
    index = int(message["index"])
    started = time.monotonic()
    time.sleep(args.settle_time_s)          # 잔진동이 가라앉기를 기다린다

    pose6 = message["flange_pose_6dof"]
    record = {
        "event_id": index,
        "capture_index": index,
        "capture_block": CAPTURE_BLOCK,
        "robot_pose_6dof": [float(v) for v in pose6],
        "robot_pose_matrix_4x4": zeus_pose_to_matrix(pose6).tolist(),
        # Step3 계열이 찾는 이름. 이 리그에서는 플랜지 자세와 같은 값이다.
        "capture_gripper_pose_6dof": [float(v) for v in pose6],
        "capture_gripper_pose_matrix_4x4": zeus_pose_to_matrix(pose6).tolist(),
        "capture_robot_joints_6dof": message.get("joints_6dof"),
        "tool": message.get("tool", 1),
        "cams": {},
    }

    counts, detail = {}, []
    for camera in cameras:
        key = str(camera.index)
        try:
            frame = camera.grab()
        except Exception as error:
            record["cams"][key] = {"saved": False, "is_gripper": False,
                                   "skip_reason": f"grab_failed: {error}"}
            counts[camera.name] = 0
            detail.append(f"{camera.name}:취득실패")
            continue
        if frame is None:
            record["cams"][key] = {"saved": False, "is_gripper": False,
                                   "skip_reason": "no_color_frame"}
            counts[camera.name] = 0
            detail.append(f"{camera.name}:프레임없음")
            continue

        charuco = detector.detect(frame["color"], camera.K, camera.D)
        draw = charuco.pop("_draw")
        counts[camera.name] = charuco["n_corners"]

        # 이미지는 검출 성공 여부와 무관하게 저장한다. 나중에 검출기를 바꿔
        # 다시 뽑을 수 있어야 하고, 실패 사례 자체가 품질의 근거이기 때문이다.
        rgb_rel = f"cam{camera.index}/rgb_{index:05d}.jpg"
        cv2.imwrite(str(session_dir / rgb_rel), frame["color"],
                    [int(cv2.IMWRITE_JPEG_QUALITY), args.jpg_quality])
        depth_rel = None
        if args.save_depth and frame["depth"] is not None:
            depth_rel = f"cam{camera.index}/depth_{index:05d}.png"
            cv2.imwrite(str(session_dir / depth_rel), frame["depth"])

        reason = None
        if not charuco["ok"]:
            reason = f"charuco_pose_failed(corners={charuco['n_corners']})"
        elif charuco["n_corners"] < args.min_corners:
            reason = (f"too_few_corners({charuco['n_corners']}"
                      f"<{args.min_corners})")
        elif charuco["reproj_error_px"] > args.max_reproj_px:
            reason = (f"reproj_too_high({charuco['reproj_error_px']:.2f}"
                      f">{args.max_reproj_px})")
        if reason:
            detail.append(f"{camera.name}:{reason}")

        record["cams"][key] = {
            "saved": True,
            "is_gripper": False,
            "rgb_path": rgb_rel,
            "depth_path": depth_rel,
            "ts_ms": frame["ts_ms"],
            "host_monotonic_ts_ms": frame["host_monotonic_ts_ms"],
            "device_ts_ms": frame["device_ts_ms"],
            "n_markers_detected": len(charuco["marker_ids"]),
            "marker_ids": charuco["marker_ids"],
            "foreign_marker_ids": charuco["foreign_marker_ids"],
            "charuco_detect_n": charuco["n_corners"],
            "charuco": charuco,
            "gate_reason": reason,          # None 이면 이 카메라는 통과
        }

        if args.show:
            preview = frame["color"].copy()
            if draw[0] is not None:
                cv2.aruco.drawDetectedCornersCharuco(preview, draw[0], draw[1])
            cv2.imshow(camera.name, cv2.resize(preview, (640, 360)))

    if args.show:
        cv2.waitKey(1)

    passed = [c for c in cameras
              if record["cams"][str(c.index)].get("gate_reason") is None
              and record["cams"][str(c.index)].get("saved")]
    record["capture_span_ms"] = (time.monotonic() - started) * 1000.0
    record["capture_gate"] = {
        "pass": len(passed) == len(cameras),
        "n_cams_passed": len(passed),
        "n_cams_required": len(cameras),
        "reason": "; ".join(detail) if detail else "all cameras passed",
    }
    return record, counts, detail


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--session-dir", required=True)
    parser.add_argument("--robot-host", default="192.168.0.23")
    parser.add_argument("--robot-port", type=int, default=12348)
    parser.add_argument("--intrinsics-dir", default=str(ROOT / "intrinsics"))
    parser.add_argument("--cameras", nargs="+", default=list(DEFAULT_CAMERAS))
    parser.add_argument("--robot-board-id-start", type=int, default=None,
                        help="로봇 보드의 마커 ID 시작번호. board_config.py 에 "
                             "기입돼 있으면 생략 가능. check_board_ids.py 로 확인.")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--save-depth", action="store_true", default=True)
    parser.add_argument("--no-save-depth", dest="save_depth", action="store_false")
    parser.add_argument("--settle-time-s", type=float, default=1.5)
    parser.add_argument("--startup-stagger-s", type=float, default=0.8)
    parser.add_argument("--min-corners", type=int, default=12,
                        help=f"로봇 보드 전체 코너는 {ROBOT_BOARD.corner_count}개")
    parser.add_argument("--max-reproj-px", type=float, default=1.5)
    parser.add_argument("--jpg-quality", type=int, default=95)
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()

    id_start = args.robot_board_id_start
    if id_start is None:
        id_start = ROBOT_BOARD.marker_id_start
    if id_start is None:
        print("로봇 보드의 마커 ID 시작번호를 알 수 없다.", file=sys.stderr)
        print("  python capture/check_board_ids.py --live  로 확인한 뒤", file=sys.stderr)
        print("  board_config.py 에 기입하거나 --robot-board-id-start 로 넘겨라.",
              file=sys.stderr)
        return 2

    detector = BoardDetector(ROBOT_BOARD, id_start)
    print(ROBOT_BOARD.describe().replace("ID 미확인", f"ID {id_start}~"
                                         f"{id_start + ROBOT_BOARD.marker_count - 1}"))
    table_span = TABLE_BOARD.id_range()
    clash = sorted(detector.id_set & set(range(table_span[0], table_span[1] + 1)))
    if clash:
        print(f"[경고] 로봇 보드 ID 가 테이블 보드({table_span[0]}~{table_span[1]})와 "
              f"{len(clash)}개 겹친다: {clash}")
        print("       두 보드가 한 화면에 들어오면 검출이 섞여 pose 가 조용히 틀어진다.")
        print("       촬영 중에는 테이블 보드를 치우거나 덮어라.")
        print("       각 촬영의 foreign_marker_ids 를 확인해 섞임을 감시한다.")

    session_dir = Path(args.session_dir).resolve()
    session_dir.mkdir(parents=True, exist_ok=True)
    cameras = open_cameras(Path(args.intrinsics_dir), args.cameras,
                           args.width, args.height, args.fps,
                           args.save_depth, args.startup_stagger_s)
    for camera in cameras:
        (session_dir / f"cam{camera.index}").mkdir(exist_ok=True)

    meta = build_session_header(args, cameras, detector, session_dir)
    meta_path = session_dir / "meta.json"

    def flush():
        with meta_path.open("w", encoding="utf-8") as handle:
            json.dump(meta, handle, indent=2, ensure_ascii=False)

    flush()
    print(f"\n세션: {session_dir}")
    print(f"로봇 접속: {args.robot_host}:{args.robot_port} ...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((args.robot_host, args.robot_port))
    print("접속됨. 로봇 터미널에서 조작하라.\n")

    buffer = ""
    try:
        while True:
            chunk = sock.recv(8192).decode("utf-8")
            if not chunk:
                print("로봇 연결 종료")
                break
            buffer += chunk
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                if not line.strip():
                    continue
                message = json.loads(line)
                command = message.get("command")

                if command == "capture":
                    record, counts, detail = capture_once(
                        cameras, detector, message, session_dir, args)
                    meta["captures"].append(record)
                    flush()          # 매 촬영마다 기록한다. 중단돼도 데이터가 남는다.
                    status = "pass" if record["capture_gate"]["pass"] else "gate_fail"
                    print(f"[{record['event_id']:3d}] {status}  corners={counts}"
                          + (f"  | {'; '.join(detail)}" if detail else ""))
                    sock.sendall((json.dumps({
                        "status": "saved" if record["capture_gate"]["pass"]
                                  else "rejected",
                        "detail": record["capture_gate"]["reason"],
                        "corners_per_camera": counts,
                    }) + "\n").encode("utf-8"))

                elif command in ("quit", "session_end"):
                    print(f"세션 종료 신호: {command}")
                    raise SystemExit(0)

    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            meta["shah_field_check"] = solve_and_report(meta["captures"], args.cameras)
        except Exception as error:
            print(f"[주의] 현장 검증 생략: {error}")
            meta["shah_field_check"] = None
        flush()
        print(f"\n기록 완료: {meta_path}  (촬영 {len(meta['captures'])}건)")
        for camera in cameras:
            camera.stop()
        if args.show:
            cv2.destroyAllWindows()
        sock.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
