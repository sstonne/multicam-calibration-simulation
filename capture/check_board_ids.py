#!/usr/bin/env python3
"""실물 ChArUco 보드 ID와 설정된 칸 배치/인쇄 패턴을 검사한다.

현재 로봇 보드: ID 90~116, 9x6칸, checker 25mm / marker 18mm,
legacy 패턴. 테이블 보드: ID 5~42, 11x7칸.

  python capture/check_board_ids.py --live --serial 319522062138
  python capture/check_board_ids.py --image 사진.jpg

카메라를 향한 로봇 보드가 보이는 프레임을 사용한다. 기본 카메라의 시야에
테이블 보드만 들어올 수도 있다. 일부 ID가 가려지거나 오검출될 수 있으므로
검출된 최솟값을 시작 ID로 자동 저장하지 않는다. 물리 길이는 별도 확인한다.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from board_config import ROBOT_BOARD, TABLE_BOARD, BoardConfig

DICT_VARIANTS = ["DICT_4X4_50", "DICT_4X4_100", "DICT_4X4_250", "DICT_4X4_1000"]


def detect_marker_ids(gray, dictionary_name: str) -> list[int]:
    dictionary = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, dictionary_name))
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    _, ids, _ = detector.detectMarkers(gray)
    return [] if ids is None else sorted(int(v) for v in ids.ravel())


def charuco_corner_count(gray, board: BoardConfig, id_start: int,
                         squares=None, legacy_pattern=None) -> int:
    """주어진 보드 정의로 몇 개의 체커 코너가 잡히는지.

    코너가 많이 잡히면 그 정의가 실물과 맞다는 증거다. 규격을 잘못 알고 있으면
    (예: 가로세로가 뒤바뀌었으면) 0 에 가깝게 나온다.
    """
    squares = squares or (board.squares_x, board.squares_y)
    dictionary = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, board.dictionary_name))
    count = (squares[0] * squares[1]) // 2
    ids = np.arange(id_start, id_start + count, dtype=np.int32).reshape(-1, 1)
    grid = cv2.aruco.CharucoBoard(
        squares, board.square_length_m, board.marker_length_m, dictionary, ids)
    grid.setLegacyPattern(board.legacy_pattern if legacy_pattern is None
                          else legacy_pattern)
    corners, _, _, _ = cv2.aruco.CharucoDetector(grid).detectBoard(gray)
    return 0 if corners is None else len(corners)


def report(image_bgr) -> None:
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    table_span = TABLE_BOARD.id_range()
    table_ids = set(range(table_span[0], table_span[1] + 1))

    print("\n[1] 딕셔너리별 검출된 마커 ID")
    detected = {}
    for name in DICT_VARIANTS:
        ids = detect_marker_ids(gray, name)
        detected[name] = ids
        print(f"  {name:16s}: {len(ids):3d}개  {ids if ids else '검출 없음'}")

    ids = detected["DICT_4X4_250"]
    if not ids:
        print("\n마커가 검출되지 않았다.")
        print("보드가 화면에 크게, 초점이 맞은 상태로 들어오도록 다시 찍어라.")
        return

    print(f"\n[2] 테이블 보드(ID {table_span[0]}~{table_span[1]})와의 관계")
    outside = sorted(set(ids) - table_ids)
    inside = sorted(set(ids) & table_ids)
    print(f"  테이블 대역 밖 ID : {outside if outside else '없음'}")
    print(f"  테이블 대역 안 ID : {inside if inside else '없음'}")

    print("\n[3] 설정된 로봇 보드 ID 확인")
    start = ROBOT_BOARD.marker_id_start
    if start is None:
        print("  로봇 보드 시작 ID가 아직 설정되지 않았다.")
        print("  일부 마커가 가려질 수 있으므로 검출된 최솟값만으로 시작 ID를 정하지 않는다.")
        return
    expected = set(range(start, start + ROBOT_BOARD.marker_count))
    found = sorted(set(ids) & expected)
    missing = sorted(expected - set(ids))
    unrelated = sorted(set(ids) - expected - table_ids)
    print(f"  설정: ID {start}~{max(expected)}, 마커 {len(expected)}개")
    print(f"  검출: {len(found)}/{len(expected)}개 {found}")
    print(f"  미검출: {missing or '(없음)'}")
    if unrelated:
        print(f"  어느 보드에도 속하지 않는 ID (오검출/다른 마커 가능): {unrelated}")
    clash = sorted(expected & table_ids)
    print(f"  테이블과 ID 충돌: {clash or '(없음)'}")

    print("\n[4] 칸 배치와 인쇄 패턴 확인")
    sx, sy = ROBOT_BOARD.squares_x, ROBOT_BOARD.squares_y
    trials = [
        (f"로봇 {sx}x{sy} 설정 패턴", ROBOT_BOARD, start, (sx, sy), ROBOT_BOARD.legacy_pattern),
        (f"로봇 {sx}x{sy} 반대 패턴", ROBOT_BOARD, start, (sx, sy), not ROBOT_BOARD.legacy_pattern),
        (f"로봇 {sy}x{sx} 전치", ROBOT_BOARD, start, (sy, sx), ROBOT_BOARD.legacy_pattern),
        ("테이블", TABLE_BOARD, TABLE_BOARD.marker_id_start,
         (TABLE_BOARD.squares_x, TABLE_BOARD.squares_y), TABLE_BOARD.legacy_pattern),
    ]
    for label, board, id_start, squares, legacy in trials:
        total = (squares[0] - 1) * (squares[1] - 1)
        found = charuco_corner_count(gray, board, id_start, squares, legacy)
        print(f"  {label}: legacy={legacy} 코너 {found}/{total}")
    print("  코너 검출은 배치를 확인한다. 실제 칸/마커 길이(mm)는 실측 또는 인쇄 규격으로 확인한다.")


def grab_live(serial: str | None, save_path: str):
    import pyrealsense2 as rs

    devices = [d.get_info(rs.camera_info.serial_number) for d in rs.context().devices]
    if not devices:
        sys.exit("연결된 RealSense 가 없다")
    chosen = serial or devices[0]
    print(f"연결된 장치: {devices}\n사용: {chosen}")

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(chosen)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 15)
    pipeline.start(config)
    try:
        image = None
        for _ in range(20):                     # 자동노출 안정화
            frames = pipeline.wait_for_frames(timeout_ms=5000)
            color = frames.get_color_frame()
            if color:
                image = np.asanyarray(color.get_data())
    finally:
        pipeline.stop()

    if image is None:
        sys.exit("프레임을 받지 못했다")
    cv2.imwrite(save_path, image)
    print(f"프레임 저장: {save_path}  {image.shape[1]}x{image.shape[0]}")
    return image


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", help="검사할 이미지 파일 (휴대폰 사진도 가능)")
    parser.add_argument("--live", action="store_true", help="RealSense 에서 한 장 취득")
    parser.add_argument("--serial", help="특정 RealSense serial (미지정 시 첫 장치)")
    parser.add_argument("--save", default="board_check.jpg", help="취득 프레임 저장 경로")
    args = parser.parse_args()

    print(ROBOT_BOARD.describe())
    print(TABLE_BOARD.describe())

    if args.image:
        image = cv2.imread(args.image)
        if image is None:
            sys.exit(f"이미지를 읽을 수 없다: {args.image}")
        print(f"\n이미지: {args.image}  {image.shape[1]}x{image.shape[0]}")
    elif args.live:
        image = grab_live(args.serial, args.save)
    else:
        sys.exit("--live 또는 --image 중 하나를 지정하라")

    report(image)


if __name__ == "__main__":
    main()
