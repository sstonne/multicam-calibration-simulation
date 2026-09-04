#!/usr/bin/env python3
"""로봇 보드의 마커 ID 시작번호를 실물에서 읽어낸다.

왜 필요한가:
  로봇 보드를 인쇄한 PDF 에 ID 가 적혀 있지 않고, PC 어디에도 그 보드의 생성
  기록이 없다(2026-09-03 전수 확인). 테이블 보드는 규격이 두 곳에 기록돼
  있지만 로봇 보드는 새 보드라 아직 파이프라인에 등록된 적이 없다.
  따라서 실물 보드가 유일한 진실의 출처다.

  ID 를 알아야 하는 이유는 두 가지다.
    1. 검출기에 marker_id_start 를 넣어야 ChArUco 보간이 성립한다.
    2. 테이블 보드(ID 5~42)와 겹치면 두 보드가 한 화면에 들어올 때
       검출이 섞여 조용히 틀린 pose 가 나온다.

사용법:
  python capture/check_board_ids.py --live                    # 첫 RealSense
  python capture/check_board_ids.py --live --serial 314522062542
  python capture/check_board_ids.py --image 사진.jpg           # 휴대폰 사진도 가능

로봇을 켤 필요가 없다. 보드를 카메라 앞에 들거나 사진 한 장이면 된다.
확인한 값은 capture/board_config.py 의 ROBOT_BOARD.marker_id_start 에 적어라.
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
                         squares=None) -> int:
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

    print("\n[3] 로봇 보드 ID 시작번호 추정")
    if outside:
        start = min(outside)
        last = start + ROBOT_BOARD.marker_count - 1
        print(f"  테이블 대역 밖 ID 가 있다 -> 로봇 보드는 Id:{start} 부터로 보인다"
              f" ({start}~{last}, 마커 {ROBOT_BOARD.marker_count}개)")
        print("  => 테이블 보드와 충돌 없음. 그대로 촬영 가능.")
        print(f"  => board_config.py 의 ROBOT_BOARD.marker_id_start = {start} 로 기입하라.")
    else:
        start = 0
        print("  검출된 ID 가 전부 테이블 대역 안에 있다.")
        print("  * 화면에 테이블 보드도 함께 있었다면: 로봇 보드만 다시 찍어라.")
        print("  * 로봇 보드만 찍은 결과라면: 두 보드가 ID 대역을 공유한다.")
        print("    [경고] 이대로 촬영하면 두 보드가 한 화면에 들어올 때 pose 가 조용히 틀어진다.")
        print("    대응은 docs/real_shah_capture.md 의 '마커 ID 충돌' 절 참조.")

    print("\n[4] 보드 정의 검증 (코너가 많이 잡히면 그 정의가 실물과 맞다)")
    trials = [
        ("로봇 보드 (7x5)", ROBOT_BOARD, start, (7, 5), ROBOT_BOARD.corner_count),
        ("로봇 보드 전치 (5x7)", ROBOT_BOARD, start, (5, 7), ROBOT_BOARD.corner_count),
        ("테이블 보드 (11x7)", TABLE_BOARD, TABLE_BOARD.marker_id_start, (11, 7),
         TABLE_BOARD.corner_count),
    ]
    for label, board, id_start, squares, total in trials:
        found = charuco_corner_count(gray, board, id_start, squares)
        verdict = "  <== 일치" if found >= total * 0.5 else ""
        print(f"  {label:22s} (Id{id_start:3d}): 코너 {found:2d}/{total}{verdict}")


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
