#!/usr/bin/env python3
"""RealSense 한 대의 color intrinsic 을 ChArUco 로 새로 캘리브레이션한다.

언제 쓰는가:
  물리 카메라 유닛을 교체했을 때. Intrinsic 은 센서 유닛 고유값이라 같은
  D415 라도 유닛마다 fx/fy, cx/cy, 왜곡계수가 다르다. 옛 유닛의 K 로 새
  유닛 이미지를 해석하면 검출은 성공하고 reprojection 도 그럴듯한데
  Shah 결과가 조용히 편향된다.

산출물:
  intrinsics/<name>.npz — shah_capture_client.open_cameras 가 읽는 스키마
  (serial, is_gripper, color_w, color_h, color_K, color_D).
  기존 파일이 있으면 <name>.npz.bak-YYYYMMDD-HHMMSS 로 옮긴다.

사용법:
  python capture/calibrate_intrinsics.py --serial 039422061216 --name cam0
  python capture/calibrate_intrinsics.py --serial 039422061216 --name cam0 --frames 25
  python capture/calibrate_intrinsics.py --serial 039422061216 --name cam0 --board robot

키 입력 (라이브 창이 포커스여야 함):
  SPACE  현재 프레임 캡처. 코너 12개 이상 잡힐 때만 저장된다.
  u      마지막 캡처 되돌리기.
  c      지금까지 모은 프레임으로 캘리브레이션 실행 (frames 도달해도 자동 실행).
  q      저장하지 않고 종료.

촬영 요령 — 이 넷을 안 지키면 K 가 편향된다:
  * 보드가 화면의 여러 위치(중앙 / 네 모서리 / 상하좌우 가장자리)에 오도록
  * 카메라~보드 거리를 30cm ~ 1.5m 사이에서 여러 값 섞기
  * 보드를 좌우 ±30°, 상하 ±30°, 롤 ±20° 정도 기울여서
  * 항상 정지 상태에서 캡처 — 흐릿한 프레임은 왜곡을 왜곡한다
"""

from __future__ import annotations

import argparse
import shutil
import sys
import time
from pathlib import Path
from datetime import datetime

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
from board_config import ROBOT_BOARD, TABLE_BOARD, BoardConfig  # noqa: E402


REPROJ_GOOD_PX = 0.5
REPROJ_WARN_PX = 0.8


def make_board(board_config: BoardConfig, marker_id_start: int):
    dictionary = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, board_config.dictionary_name))
    ids = np.arange(
        marker_id_start,
        marker_id_start + board_config.marker_count,
        dtype=np.int32).reshape(-1, 1)
    grid = cv2.aruco.CharucoBoard(
        (board_config.squares_x, board_config.squares_y),
        board_config.square_length_m,
        board_config.marker_length_m,
        dictionary, ids)
    detector = cv2.aruco.CharucoDetector(grid)
    return grid, detector


def draw_overlay(frame, corners, corner_ids, captured, target, board_name, serial):
    display = frame.copy()
    if corners is not None and len(corners) > 0:
        cv2.aruco.drawDetectedCornersCharuco(display, corners, corner_ids,
                                             cornerColor=(0, 255, 0))
    h, w = display.shape[:2]
    n_corners = 0 if corners is None else len(corners)
    lines = [
        f"serial: {serial}   board: {board_name}",
        f"captured: {captured}/{target}   corners now: {n_corners}",
        "SPACE capture   u undo   c calibrate now   q quit",
    ]
    for i, line in enumerate(lines):
        y = 24 + i * 22
        cv2.putText(display, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(display, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (255, 255, 255), 1, cv2.LINE_AA)
    return display


def calibrate(all_object_points, all_image_points, image_size):
    """OpenCV 4.x 의 aruco.calibrateCameraCharuco 대신, matchImagePoints 로 얻은
    3D-2D 대응을 그대로 cv2.calibrateCamera 에 넣는다. ChArUco 여부에 무관하게
    작동하고, extended API 이름 변경 이슈도 피할 수 있다."""
    flags = 0     # RealSense 는 왜곡이 크지 않으니 rational_model 은 쓰지 않는다
    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(
        all_object_points, all_image_points, image_size, None, None, flags=flags)
    return float(rms), K, D.reshape(-1)


def per_view_reproj(all_object_points, all_image_points, K, D):
    errors = []
    for obj, img in zip(all_object_points, all_image_points):
        ok, rvec, tvec = cv2.solvePnP(obj, img, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            errors.append(float("nan"))
            continue
        proj, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
        residual = proj.reshape(-1, 2) - img.reshape(-1, 2)
        errors.append(float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1)))))
    return errors


def save_intrinsics(path: Path, serial: str, is_gripper: bool,
                    width: int, height: int, K: np.ndarray, D: np.ndarray) -> None:
    if path.exists():
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        backup = path.with_name(f"{path.name}.bak-{stamp}")
        shutil.move(str(path), str(backup))
        print(f"기존 파일 백업: {backup.name}")
    np.savez(
        path,
        serial=str(serial),
        is_gripper=bool(is_gripper),
        color_w=int(width),
        color_h=int(height),
        color_K=np.asarray(K, dtype=np.float64),
        color_D=np.asarray(D, dtype=np.float64).reshape(-1),
    )
    print(f"저장: {path}")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--serial", required=True,
                        help="캘리브레이션할 RealSense serial (앞의 0 포함, 예: 039422061216)")
    parser.add_argument("--name", required=True,
                        help="저장 파일 이름. intrinsics/<name>.npz 로 저장된다 (예: cam0)")
    parser.add_argument("--intrinsics-dir", default=str(ROOT / "intrinsics"))
    parser.add_argument("--frames", type=int, default=30,
                        help="목표 캡처 수 (기본 30, 권장 25~40)")
    parser.add_argument("--min-corners", type=int, default=12,
                        help="한 프레임을 유효로 인정할 최소 코너 수 (기본 12)")
    parser.add_argument("--width", type=int, default=1280,
                        help="촬영 해상도 폭. record_dataset 기본과 일치해야 함")
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--board", choices=["table", "robot"], default="table",
                        help="캘리브레이션에 쓸 보드. 기본 table (코너 60개)")
    parser.add_argument("--robot-board-id-start", type=int, default=None,
                        help="board=robot 이고 board_config 에 값이 없을 때만 지정")
    parser.add_argument("--is-gripper", action="store_true",
                        help="이 카메라가 손목(그리퍼) 장착이면 켠다. 고정 카메라면 생략.")
    parser.add_argument("--dry-run", action="store_true",
                        help="캘리브레이션만 하고 파일에 쓰지 않는다")
    return parser.parse_args()


def main() -> None:
    import pyrealsense2 as rs
    args = parse_args()

    intrinsics_dir = Path(args.intrinsics_dir)
    intrinsics_dir.mkdir(parents=True, exist_ok=True)
    out_path = intrinsics_dir / f"{args.name}.npz"

    if args.board == "table":
        board_config = TABLE_BOARD
        marker_id_start = TABLE_BOARD.marker_id_start
    else:
        board_config = ROBOT_BOARD
        marker_id_start = args.robot_board_id_start or ROBOT_BOARD.marker_id_start
        if marker_id_start is None:
            sys.exit("로봇 보드 마커 ID 시작번호를 알 수 없다. "
                     "board_config.py 에 기입하거나 --robot-board-id-start 로 지정하라.")

    print(f"보드: {board_config.name}  {board_config.squares_x}x{board_config.squares_y} "
          f"corners={board_config.corner_count}  markers ID {marker_id_start}~"
          f"{marker_id_start + board_config.marker_count - 1}")

    grid, detector = make_board(board_config, marker_id_start)

    connected = {d.get_info(rs.camera_info.serial_number) for d in rs.context().devices}
    if args.serial not in connected:
        sys.exit(f"serial {args.serial} 가 연결돼 있지 않다. 연결된 장치: {sorted(connected)}")
    print(f"카메라 열기: serial={args.serial}  {args.width}x{args.height}@{args.fps}")

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(args.serial)
    config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
    pipeline.start(config)

    all_object_points: list[np.ndarray] = []
    all_image_points: list[np.ndarray] = []
    all_frames: list[np.ndarray] = []
    image_size = (args.width, args.height)

    window = f"calibrate {args.name} (serial {args.serial})"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, args.width, args.height)

    last_capture_time = 0.0
    try:
        while True:
            frames = pipeline.wait_for_frames(timeout_ms=5000)
            color = frames.get_color_frame()
            if not color:
                continue
            frame = np.asanyarray(color.get_data())
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, corner_ids, _, _ = detector.detectBoard(gray)

            display = draw_overlay(frame, corners, corner_ids,
                                   len(all_frames), args.frames,
                                   board_config.name, args.serial)
            cv2.imshow(window, display)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                print("사용자 종료. 저장하지 않는다.")
                return
            if key == ord('u') and all_frames:
                all_object_points.pop()
                all_image_points.pop()
                all_frames.pop()
                print(f"되돌림. 남은 캡처: {len(all_frames)}")
            if key == ord(' '):
                now = time.time()
                if now - last_capture_time < 0.4:      # 스페이스 연타 무시
                    continue
                if corners is None or len(corners) < args.min_corners:
                    n = 0 if corners is None else len(corners)
                    print(f"[skip] 코너 {n} 개 (< {args.min_corners}) — 자세를 잡고 다시.")
                    continue
                obj, img = grid.matchImagePoints(corners, corner_ids)
                if obj is None or len(obj) < args.min_corners:
                    print("[skip] matchImagePoints 실패.")
                    continue
                all_object_points.append(np.asarray(obj, dtype=np.float32))
                all_image_points.append(np.asarray(img, dtype=np.float32))
                all_frames.append(frame.copy())
                last_capture_time = now
                print(f"  캡처 {len(all_frames)}/{args.frames}  코너 {len(obj)}")

            do_calibrate = (key == ord('c')) or (len(all_frames) >= args.frames)
            if do_calibrate:
                if len(all_frames) < 8:
                    print(f"프레임이 너무 적다 ({len(all_frames)}). 20장 이상 권장.")
                    continue
                break
    finally:
        pipeline.stop()
        cv2.destroyWindow(window)

    print(f"\n캘리브레이션 실행: {len(all_frames)} 프레임")
    rms, K, D = calibrate(all_object_points, all_image_points, image_size)
    per_view = per_view_reproj(all_object_points, all_image_points, K, D)

    print("\n=== 결과 ===")
    print(f"overall RMS reproj error : {rms:.4f} px")
    print(f"per-view mean            : {np.nanmean(per_view):.4f} px")
    print(f"per-view max             : {np.nanmax(per_view):.4f} px")
    print(f"K =\n{np.array2string(K, precision=4, suppress_small=True)}")
    print(f"D = {np.array2string(D, precision=6, suppress_small=True)}")

    if rms <= REPROJ_GOOD_PX:
        verdict = "OK — 촬영 진행 가능"
    elif rms <= REPROJ_WARN_PX:
        verdict = "경계값 — 자세 다양성 부족 의심. 가능하면 다시."
    else:
        verdict = f"불량 (>{REPROJ_WARN_PX}px) — 재촬영 필요"
    print(f"판정: {verdict}")

    if args.dry_run:
        print("\n--dry-run 이므로 저장하지 않는다.")
        return

    if rms > REPROJ_WARN_PX:
        answer = input("불량 판정인데 그래도 저장할까? [y/N]: ").strip().lower()
        if answer != 'y':
            print("저장 취소.")
            return

    save_intrinsics(out_path, args.serial, args.is_gripper,
                    args.width, args.height, K, D)
    print("\n다음 단계:")
    print(f"  python capture/check_board_ids.py --live --serial {args.serial}")
    print("  → [4] 검증에서 코너 잘 잡히는지 확인 후 record_dataset.py 실행")


if __name__ == "__main__":
    main()