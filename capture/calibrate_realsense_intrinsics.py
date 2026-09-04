#!/usr/bin/env python3
"""새 RealSense의 색상 intrinsic을 ChArUco로 보정한다.

이 도구는 로봇이나 다른 카메라를 열지 않는다. 대상 장치는 반드시 serial로
선택하며, 결과는 기존 cam0.npz를 덮어쓰지 않고 후보 NPZ로만 저장한다.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))

from board_config import TABLE_BOARD, BoardConfig


TARGET_SERIAL = "039422061216"
WIDTH, HEIGHT, FPS = 1280, 720, 15
EXPECTED_KEYS = (
    "serial", "is_gripper", "color_K", "color_D", "depth_K", "depth_D",
    "depth_scale_m_per_unit", "color_w", "color_h", "fps",
    "R_depth_to_color", "t_depth_to_color", "factory_color_K",
    "factory_color_D", "intrinsics_source", "charuco_reproj_error_px",
    "charuco_num_views", "charuco_calibrated_at",
)


@dataclass
class Detection:
    object_points: np.ndarray
    image_points: np.ndarray
    charuco_corners: np.ndarray
    charuco_ids: np.ndarray
    marker_count: int
    sharpness: float
    overlay: np.ndarray

    @property
    def descriptor(self) -> tuple[float, float, float]:
        """정규화한 중심과 화면 점유율로 거의 같은 view를 경고한다."""
        points = self.charuco_corners.reshape(-1, 2)
        width = max(float(self.overlay.shape[1]), 1.0)
        height = max(float(self.overlay.shape[0]), 1.0)
        center = points.mean(axis=0)
        span = points.max(axis=0) - points.min(axis=0)
        return (float(center[0] / width), float(center[1] / height),
                float(np.linalg.norm(span / np.array([width, height]))))


def make_board(config: BoardConfig = TABLE_BOARD):
    """보드와 사전 정의 dictionary를 현장 설정 한 곳에서 생성한다."""
    if config.marker_id_start is None:
        raise ValueError(f"{config.name}의 marker ID 시작번호가 없다")
    dictionary = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, config.dictionary_name))
    ids = np.arange(config.marker_id_start,
                    config.marker_id_start + config.marker_count,
                    dtype=np.int32).reshape(-1, 1)
    board = cv2.aruco.CharucoBoard(
        (config.squares_x, config.squares_y),
        config.square_length_m, config.marker_length_m, dictionary, ids)
    return board, dictionary


def detect_charuco(image_bgr: np.ndarray, board, dictionary) -> Detection | None:
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    sharpness = float(cv2.Laplacian(gray, cv2.CV_64F).var())
    detector = cv2.aruco.CharucoDetector(board)
    corners, corner_ids, marker_corners, marker_ids = detector.detectBoard(gray)

    overlay = image_bgr.copy()
    if marker_ids is not None and marker_corners is not None:
        cv2.aruco.drawDetectedMarkers(overlay, marker_corners, marker_ids)
    if corners is not None and corner_ids is not None:
        cv2.aruco.drawDetectedCornersCharuco(overlay, corners, corner_ids)
    if corners is None or corner_ids is None:
        return None

    object_points, image_points = board.matchImagePoints(corners, corner_ids)
    if object_points is None or image_points is None:
        return None
    return Detection(
        object_points=np.asarray(object_points, dtype=np.float32),
        image_points=np.asarray(image_points, dtype=np.float32),
        charuco_corners=np.asarray(corners, dtype=np.float32),
        charuco_ids=np.asarray(corner_ids, dtype=np.int32),
        marker_count=0 if marker_ids is None else int(len(marker_ids)),
        sharpness=sharpness,
        overlay=overlay,
    )


def acceptance_reason(detection: Detection | None, min_corners: int,
                      min_sharpness: float) -> str | None:
    if detection is None:
        return "ChArUco corner를 찾지 못함"
    count = len(detection.image_points)
    if count < min_corners:
        return f"corner 부족 ({count} < {min_corners})"
    if detection.sharpness < min_sharpness:
        return (f"초점 흐림: Laplacian variance {detection.sharpness:.1f} "
                f"< {min_sharpness:.1f}")
    return None


def is_similar_view(candidate: Detection, accepted: list[Detection]) -> bool:
    if not accepted:
        return False
    cx, cy, coverage = candidate.descriptor
    for previous in accepted:
        px, py, old_coverage = previous.descriptor
        center_distance = float(np.hypot(cx - px, cy - py))
        coverage_ratio = coverage / max(old_coverage, 1e-6)
        if center_distance < 0.08 and 0.8 < coverage_ratio < 1.25:
            return True
    return False


def factory_matrix(intrinsics) -> tuple[np.ndarray, np.ndarray]:
    K = np.array([[intrinsics.fx, 0.0, intrinsics.ppx],
                  [0.0, intrinsics.fy, intrinsics.ppy],
                  [0.0, 0.0, 1.0]], dtype=np.float64)
    D = np.asarray(intrinsics.coeffs, dtype=np.float64).reshape(-1, 1)
    return K, D


def extract_factory_profile(profile) -> dict[str, np.ndarray | float | str]:
    """SDK profile에서 기존 NPZ의 depth->color 정의로 공장값을 읽는다."""
    import pyrealsense2 as rs

    color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
    depth_profile = profile.get_stream(rs.stream.depth).as_video_stream_profile()
    color_K, color_D = factory_matrix(color_profile.get_intrinsics())
    depth_K, depth_D = factory_matrix(depth_profile.get_intrinsics())

    # Calling depth_profile makes this the SDK's depth-source to color-target transform.
    extrinsics = depth_profile.get_extrinsics_to(color_profile)
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
    return {
        "depth_K": depth_K,
        "depth_D": depth_D,
        "depth_scale_m_per_unit": float(depth_scale),
        "R_depth_to_color": np.asarray(extrinsics.rotation, dtype=np.float64).reshape(3, 3),
        "t_depth_to_color": np.asarray(extrinsics.translation, dtype=np.float64).reshape(3, 1),
        "factory_color_K": color_K,
        "factory_color_D": color_D,
        "factory_color_distortion_model": str(color_profile.get_intrinsics().model),
        "factory_depth_distortion_model": str(depth_profile.get_intrinsics().model),
    }


def calibrate_views(views: list[Detection]) -> tuple[float, np.ndarray, np.ndarray, list[float]]:
    if len(views) < 3:
        raise ValueError("보정에는 최소 3개의 유효 view가 필요하다")
    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(
        [view.object_points for view in views],
        [view.image_points for view in views],
        (WIDTH, HEIGHT), None, None)
    D = np.asarray(D, dtype=np.float64).reshape(-1, 1)
    errors = []
    for view, rvec, tvec in zip(views, rvecs, tvecs):
        projected, _ = cv2.projectPoints(view.object_points, rvec, tvec, K, D)
        residual = projected.reshape(-1, 2) - view.image_points.reshape(-1, 2)
        errors.append(float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1)))))
    return float(rms), np.asarray(K, dtype=np.float64), D, errors


def candidate_payload(serial: str, K: np.ndarray, D: np.ndarray, factory: dict,
                      rms: float, num_views: int, calibrated_at: str) -> dict:
    return {
        "serial": np.asarray(serial),
        "is_gripper": np.asarray(False),
        "color_K": np.asarray(K, dtype=np.float64).reshape(3, 3),
        "color_D": np.asarray(D, dtype=np.float64).reshape(5, 1),
        "depth_K": np.asarray(factory["depth_K"], dtype=np.float64).reshape(3, 3),
        "depth_D": np.asarray(factory["depth_D"], dtype=np.float64).reshape(5, 1),
        "depth_scale_m_per_unit": np.asarray(factory["depth_scale_m_per_unit"], dtype=np.float64),
        "color_w": np.asarray(WIDTH, dtype=np.int64),
        "color_h": np.asarray(HEIGHT, dtype=np.int64),
        "fps": np.asarray(FPS, dtype=np.int64),
        "R_depth_to_color": np.asarray(factory["R_depth_to_color"], dtype=np.float64).reshape(3, 3),
        "t_depth_to_color": np.asarray(factory["t_depth_to_color"], dtype=np.float64).reshape(3, 1),
        "factory_color_K": np.asarray(factory["factory_color_K"], dtype=np.float64).reshape(3, 3),
        "factory_color_D": np.asarray(factory["factory_color_D"], dtype=np.float64).reshape(5, 1),
        "intrinsics_source": np.asarray("charuco"),
        "charuco_reproj_error_px": np.asarray(rms, dtype=np.float64),
        "charuco_num_views": np.asarray(num_views, dtype=np.int64),
        "charuco_calibrated_at": np.asarray(calibrated_at),
    }


def validate_candidate(path: Path, reference_path: Path, serial: str) -> list[str]:
    """저장 후 다시 열어 reader 및 기존 schema와의 호환성을 확인한다."""
    errors: list[str] = []
    with np.load(path, allow_pickle=False) as candidate, np.load(reference_path, allow_pickle=False) as reference:
        if tuple(candidate.files) != EXPECTED_KEYS:
            errors.append(f"key 순서/구성이 다름: {candidate.files}")
        for key in EXPECTED_KEYS:
            if key not in candidate:
                errors.append(f"필수 key 누락: {key}")
                continue
            if key not in reference:
                errors.append(f"기준 파일 key 누락: {key}")
                continue
            if candidate[key].shape != reference[key].shape:
                errors.append(f"{key} shape {candidate[key].shape} != {reference[key].shape}")
            if candidate[key].dtype != reference[key].dtype:
                errors.append(f"{key} dtype {candidate[key].dtype} != {reference[key].dtype}")

        if str(candidate["serial"]) != serial:
            errors.append(f"serial 불일치: {candidate['serial']!r}")
        if bool(candidate["is_gripper"]):
            errors.append("is_gripper가 False가 아님")
        if (int(candidate["color_w"]), int(candidate["color_h"]), int(candidate["fps"])) != (WIDTH, HEIGHT, FPS):
            errors.append("color 해상도 또는 FPS가 1280x720@15가 아님")
        for key in ("color_K", "color_D", "depth_K", "depth_D", "R_depth_to_color",
                    "t_depth_to_color", "factory_color_K", "factory_color_D"):
            if key in candidate and not np.isfinite(candidate[key]).all():
                errors.append(f"{key}에 NaN/Inf가 있음")
        K = candidate["color_K"]
        if not (WIDTH * 0.25 < K[0, 0] < WIDTH * 4 and HEIGHT * 0.25 < K[1, 1] < HEIGHT * 4):
            errors.append("focal length가 합리적인 범위를 벗어남")
        if not (0.0 < K[0, 2] < WIDTH and 0.0 < K[1, 2] < HEIGHT):
            errors.append("principal point가 이미지 범위를 벗어남")
        R = candidate["R_depth_to_color"]
        if not np.allclose(R.T @ R, np.eye(3), atol=1e-4) or not np.isclose(np.linalg.det(R), 1.0, atol=1e-4):
            errors.append("R_depth_to_color가 유효한 회전행렬이 아님")
    return errors


def write_manifest(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")


def annotate(image: np.ndarray, detection: Detection | None, saved_count: int,
             min_corners: int, min_views: int) -> np.ndarray:
    canvas = image.copy() if detection is None else detection.overlay.copy()
    corners = 0 if detection is None else len(detection.image_points)
    sharpness = 0.0 if detection is None else detection.sharpness
    lines = [
        f"ChArUco corners: {corners} (min {min_corners})",
        f"Valid views: {saved_count} (min {min_views}, recommended 20-30)",
        f"Sharpness: {sharpness:.1f}",
        "SPACE: save valid view | q / ESC: calibrate and quit",
        "Move board across corners, edges, distances, and tilts.",
    ]
    for row, line in enumerate(lines):
        cv2.putText(canvas, line, (16, 32 + 28 * row), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (20, 220, 20), 2, cv2.LINE_AA)
    return canvas


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", default=TARGET_SERIAL,
                        help="대상 RealSense serial. 앞자리 0을 포함한 문자열")
    parser.add_argument("--candidate-path", type=Path,
                        default=ROOT / "intrinsics" / "cam0_candidate_039422061216.npz")
    parser.add_argument("--reference-path", type=Path,
                        default=ROOT / "intrinsics" / "cam0.npz")
    parser.add_argument("--session-dir", type=Path, default=None,
                        help="원본/overlay와 manifest를 보관할 폴더")
    parser.add_argument("--min-corners", type=int, default=24)
    parser.add_argument("--min-sharpness", type=float, default=60.0)
    parser.add_argument("--min-views", type=int, default=15)
    parser.add_argument("--overwrite-candidate", action="store_true",
                        help="기존 후보 파일만 명시적으로 교체한다. cam0.npz에는 적용하지 않는다.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.serial != TARGET_SERIAL:
        print(f"안전상 이 도구는 새 cam0({TARGET_SERIAL})만 허용한다.", file=sys.stderr)
        return 2
    if args.min_corners < 4 or args.min_views < 15:
        print("min-corners는 4 이상, min-views는 15 이상이어야 한다.", file=sys.stderr)
        return 2
    if args.candidate_path.exists() and not args.overwrite_candidate:
        print(f"후보 파일이 이미 있다: {args.candidate_path}", file=sys.stderr)
        print("결과를 검토하거나, 의도적으로 다시 만들 때만 --overwrite-candidate를 사용하라.", file=sys.stderr)
        return 2
    if not args.reference_path.exists():
        print(f"기준 schema 파일이 없다: {args.reference_path}", file=sys.stderr)
        return 2

    board, dictionary = make_board()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    session_dir = (args.session_dir or ROOT / "intrinsics" / "charuco_sessions" /
                   f"cam0_{args.serial}_{timestamp}").resolve()
    session_dir.mkdir(parents=True, exist_ok=False)
    raw_dir, overlay_dir = session_dir / "raw", session_dir / "overlay"
    raw_dir.mkdir()
    overlay_dir.mkdir()
    manifest_path = session_dir / "manifest.json"
    manifest = {
        "kind": "charuco_intrinsic_calibration_v1",
        "target_serial": args.serial,
        "board": TABLE_BOARD.to_dict(),
        "stream": {"color": [WIDTH, HEIGHT, FPS], "depth": [WIDTH, HEIGHT, FPS]},
        "accepted_views": [],
        "rejected_save_attempts": [],
        "factory_profile": None,
    }
    write_manifest(manifest_path, manifest)

    import pyrealsense2 as rs

    connected = {device.get_info(rs.camera_info.serial_number) for device in rs.context().devices}
    if args.serial not in connected:
        print(f"대상 serial {args.serial}이 연결되어 있지 않다: {sorted(connected)}", file=sys.stderr)
        return 2

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(args.serial)
    config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
    config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
    accepted: list[Detection] = []
    try:
        profile = pipeline.start(config)
        factory = extract_factory_profile(profile)
        manifest["factory_profile"] = {
            key: value.tolist() if isinstance(value, np.ndarray) else value
            for key, value in factory.items()
        }
        write_manifest(manifest_path, manifest)
        print(f"대상 cam0: serial={args.serial}, color={WIDTH}x{HEIGHT}@{FPS}")
        print(TABLE_BOARD.describe())
        print("보드를 중앙뿐 아니라 네 모서리/가장자리, 가까이/멀리, 다양한 기울기로 보인다.")
        print("SPACE를 눌러 유효 view만 저장한다. 최소 15장, 권장 20~30장. q 또는 ESC로 계산한다.")

        while True:
            frames = pipeline.wait_for_frames(timeout_ms=5000)
            color = frames.get_color_frame()
            if not color:
                continue
            image = np.asanyarray(color.get_data())
            detection = detect_charuco(image, board, dictionary)
            preview = annotate(image, detection, len(accepted), args.min_corners, args.min_views)
            cv2.imshow("cam0 intrinsic calibration", preview)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            if key != ord(" "):
                continue

            reason = acceptance_reason(detection, args.min_corners, args.min_sharpness)
            if reason:
                print(f"[저장하지 않음] {reason}")
                manifest["rejected_save_attempts"].append({"reason": reason})
                write_manifest(manifest_path, manifest)
                continue
            assert detection is not None
            similar = is_similar_view(detection, accepted)
            if similar:
                print("[다양성 경고] 이전 view와 중심/점유율이 거의 같다. 다른 위치·거리·기울기를 권장.")

            index = len(accepted)
            raw_path = raw_dir / f"view_{index:03d}.jpg"
            overlay_path = overlay_dir / f"view_{index:03d}.jpg"
            raw_ok = cv2.imwrite(str(raw_path), image, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
            overlay_ok = cv2.imwrite(str(overlay_path), detection.overlay,
                                     [int(cv2.IMWRITE_JPEG_QUALITY), 95])
            if not raw_ok or not overlay_ok:
                raise RuntimeError(f"view 이미지 저장 실패: raw={raw_ok}, overlay={overlay_ok}")
            accepted.append(detection)
            manifest["accepted_views"].append({
                "index": index,
                "raw_path": str(raw_path.relative_to(session_dir)),
                "overlay_path": str(overlay_path.relative_to(session_dir)),
                "charuco_corners": int(len(detection.image_points)),
                "markers": detection.marker_count,
                "sharpness": detection.sharpness,
                "similarity_warning": similar,
                "charuco_ids": detection.charuco_ids.reshape(-1).tolist(),
            })
            write_manifest(manifest_path, manifest)
            print(f"[저장] view {index:03d}: corners={len(detection.image_points)}, "
                  f"sharpness={detection.sharpness:.1f}, total={len(accepted)}")
    except KeyboardInterrupt:
        print("사용자 중단. 후보 NPZ는 만들지 않는다.")
        return 130
    finally:
        try:
            pipeline.stop()
        except Exception:
            pass
        cv2.destroyAllWindows()

    if len(accepted) < args.min_views:
        print(f"유효 view가 {len(accepted)}장이다. 최소 {args.min_views}장이므로 후보 NPZ를 만들지 않는다.")
        return 2

    rms, K, D, view_errors = calibrate_views(accepted)
    print(f"\nChArUco calibration RMS: {rms:.4f} px ({len(accepted)} views)")
    for index, error in enumerate(view_errors):
        flag = "  <-- 0.5 px 초과" if error > 0.5 else ""
        print(f"  view {index:03d}: {error:.4f} px{flag}")
    print("권장 기준: 전체 RMS <= 0.5 px (기존 cam0은 약 0.3 px).")

    calibrated_at = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    payload = candidate_payload(args.serial, K, D, factory, rms, len(accepted), calibrated_at)
    args.candidate_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(args.candidate_path, **payload)
    validation_errors = validate_candidate(args.candidate_path, args.reference_path, args.serial)
    manifest["result"] = {
        "candidate_path": str(args.candidate_path.resolve()),
        "rms_px": rms,
        "per_view_reprojection_error_px": view_errors,
        "validation_errors": validation_errors,
    }
    write_manifest(manifest_path, manifest)
    if validation_errors:
        print("[실패] 후보 schema 검증:")
        for error in validation_errors:
            print(f"  - {error}")
        return 1
    print(f"[성공] 후보 파일: {args.candidate_path.resolve()}")
    print(f"[성공] 세션 근거: {session_dir}")
    print("cam0.npz는 변경하지 않았다. RMS와 view별 오차를 검토한 뒤에만 적용 여부를 결정한다.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
