 # Step2_capture_multi_cam.py
 # 멀티캠 캡처 (연결된 RealSense 카메라 수만큼 자동 등록 / 해상도는 실행 인자로 설정)

"""-----------------------------------------------------------------------------
 실행 예시
 -----------------------------------------------------------------------------
 1) 가장 안전한 기본 테스트 (권장 시작점)
 python Step2_capture_multi_cam.py \
   --root_folder ./data/cube_session_01 \
   --intrinsics_dir ./intrinsics \
   --fps 6 --width 640 --height 480 \
   --min_markers 1 \
   --save_depth \
   --show

 2) 자동 저장 사용
 python Step2_capture_multi_cam.py \
   --root_folder ./data/cube_session_01 \
   --intrinsics_dir ./intrinsics \
   --fps 6 --width 640 --height 480 \
   --min_markers 2 \
   --auto_save --stable_frames 3 --cooldown_ms 700 \
   --save_depth \
   --show

 3) 해상도 올려서 테스트
 python Step2_capture_multi_cam.py \
   --root_folder ./data/cube_session_01 \
   --intrinsics_dir ./intrinsics \
   --fps 6 --width 848 --height 480 \
   --min_markers 1 \
   --save_depth \
   --show

 4) 1280x720 테스트 (카메라 수 많으면 fps 6~10 권장)
 python Step2_capture_multi_cam.py \
   --root_folder ./data/cube_session_01 \
   --intrinsics_dir ./intrinsics \
   --fps 6 --width 1280 --height 720 \
   --min_markers 1 \
   --save_depth \
   --show

 5) 타임아웃/성능 로그 켜기
 python Step2_capture_multi_cam.py \
   --root_folder ./data/cube_session_01 \
   --intrinsics_dir ./intrinsics \
   --fps 6 --width 640 --height 480 \
   --min_markers 1 \
   --save_depth \
   --camera_frame_timeout_ms 3000 \
   --log_cam_timeouts \
   --log_cam_errors \
   --log_cam_stats_sec 5 \
   --show

 6) depth 정렬 끄기 (부하 줄이기)
 python Step2_capture_multi_cam.py \
   --root_folder ./data/cube_session_01 \
   --intrinsics_dir ./intrinsics \
   --fps 6 --width 640 --height 480 \
   --min_markers 1 \
   --save_depth \
   --no_align_depth_to_color \
   --show

 -----------------------------------------------------------------------------
 특징
 -----------------------------------------------------------------------------
 - 연결된 RealSense 카메라 수만큼 자동 등록
 - device_map.json 있으면 cam_idx 고정, 없으면 serial 정렬 fallback
 - width / height / fps 모두 실행 인자로 설정
 - depth 저장 옵션 지원
 - 카메라 시작을 순차적으로 수행하여 멀티캠 초기화 안정성 향상
 - 검출은 다운스케일 이미지로 수행 가능하여 고해상도 부하 완화
 - 4~5대 연결 시에도 자동으로 모두 사용
 - SPACE: 수동 저장 / ESC 또는 q: 종료

 -----------------------------------------------------------------------------
"""

import os
import json
import time
import argparse
from typing import Dict, Tuple, Optional

import cv2

from src3._camera import RealSenseCamera
from src3._aruco_cube import CubeConfig, ArucoCubeTarget


def ensure_dir(p: str) -> str:
    os.makedirs(p, exist_ok=True)
    return p


def load_device_map(intr_dir: str):
    map_path = os.path.join(intr_dir, "device_map.json")
    if not os.path.exists(map_path):
        return None, None

    with open(map_path, "r") as f:
        m = json.load(f)

    serial_to_idx = m.get("serial_to_idx", {})
    return serial_to_idx, map_path


def build_camera_index_pairs(devs: Dict[str, str], intr_dir: str):
    """
    devs: {serial: device_name}
    반환:
        idx_serial_pairs: [(cam_idx, serial), ...]
        map_path
    """
    serial_to_idx, map_path = load_device_map(intr_dir)

    if serial_to_idx is None:
        print("[WARN] device_map.json not found. Falling back to sorted(serial). (권장X)")
        serials = sorted(devs.keys())
        idx_serial_pairs = [(i, s) for i, s in enumerate(serials)]
        return idx_serial_pairs, None

    idx_serial_pairs = []
    for serial in devs.keys():
        if serial not in serial_to_idx:
            print(f"[WARN] serial not in device_map.json: {serial} (Step1 다시 실행 권장)")
            continue
        idx_serial_pairs.append((int(serial_to_idx[serial]), serial))

    idx_serial_pairs.sort(key=lambda x: x[0])
    return idx_serial_pairs, map_path


def resize_for_detect(img, detect_width: int):
    """
    검출 부하를 줄이기 위해 detection 전용 resize
    반환:
        resized_img, scale
    scale = resized_w / original_w
    """
    h, w = img.shape[:2]
    if detect_width <= 0 or w <= detect_width:
        return img, 1.0

    scale = detect_width / float(w)
    resized = cv2.resize(
        img,
        (int(round(w * scale)), int(round(h * scale))),
        interpolation=cv2.INTER_AREA
    )
    return resized, scale


def resize_for_show(img, show_scale: float):
    if show_scale <= 0 or show_scale == 1.0:
        return img
    return cv2.resize(
        img,
        None,
        fx=show_scale,
        fy=show_scale,
        interpolation=cv2.INTER_AREA
    )


def detect_markers(cube: ArucoCubeTarget, color, detect_width: int):
    """
    원본 color 이미지는 저장용으로 유지하고,
    detection만 다운스케일 이미지에서 수행 후 corner를 원본 좌표계로 복원.
    """
    det_img, scale = resize_for_detect(color, detect_width)
    corners, ids = cube.detect(det_img)

    if corners is not None and len(corners) > 0 and scale != 1.0:
        inv = 1.0 / scale
        corners = [c * inv for c in corners]

    return corners, ids


def save_capture_set(
    root: str,
    event_id: int,
    frames: Dict[int, dict],
    save_depth: bool,
    save_jpg_quality: int,
):
    """
    frames 안의 모든 cam frame 저장
    반환:
        cap_rec (meta.json에 기록할 dict)
    """
    cap_rec = {"event_id": int(event_id), "cams": {}}
    fid = int(event_id)

    for ci in sorted(frames.keys()):
        fr = frames[ci]

        rgb_rel = f"cam{ci}/rgb_{fid:05d}.jpg"
        rgb_abs = os.path.join(root, rgb_rel)
        cv2.imwrite(
            rgb_abs,
            fr["color"],
            [int(cv2.IMWRITE_JPEG_QUALITY), int(save_jpg_quality)],
        )

        depth_rel = None
        if save_depth and (fr["depth"] is not None):
            depth_rel = f"cam{ci}/depth_{fid:05d}.png"
            depth_abs = os.path.join(root, depth_rel)
            cv2.imwrite(depth_abs, fr["depth"])

        cap_rec["cams"][str(ci)] = {
            "saved": True,
            "ts_ms": fr["ts_ms"],
            "rgb_path": rgb_rel,
            "depth_path": depth_rel,
            "ids": fr["ids"],
        }

    return cap_rec


def main():
    parser = argparse.ArgumentParser()

    # 필수 입력
    parser.add_argument("--root_folder", required=True)
    parser.add_argument("--intrinsics_dir", required=True)

    # 카메라 스트림 설정
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)

    # 검출 조건
    parser.add_argument("--min_markers", type=int, default=2)

    # 자동 저장
    parser.add_argument("--auto_save", action="store_true")
    parser.add_argument("--stable_frames", type=int, default=3)
    parser.add_argument("--cooldown_ms", type=int, default=700)

    # depth / timeout / 로그
    parser.add_argument("--save_depth", action="store_true")
    parser.add_argument("--no_align_depth_to_color", action="store_true")
    parser.add_argument("--camera_frame_timeout_ms", type=int, default=3000)
    parser.add_argument("--log_cam_timeouts", action="store_true")
    parser.add_argument("--log_cam_errors", action="store_true")
    parser.add_argument("--log_cam_stats_sec", type=float, default=0.0)

    # 표시
    parser.add_argument("--show", action="store_true")
    parser.add_argument("--show_scale", type=float, default=1.0)

    # 멀티캠 안정화 옵션
    parser.add_argument("--startup_stagger_sec", type=float, default=0.8)
    parser.add_argument("--min_started_cams", type=int, default=1)
    parser.add_argument("--require_all_detected", action="store_true")

    # 검출 성능 최적화
    parser.add_argument("--detect_width", type=int, default=640)
    parser.add_argument("--detect_every_n", type=int, default=1)

    # 저장 품질
    parser.add_argument("--save_jpg_quality", type=int, default=95)

    args = parser.parse_args()

    if args.detect_every_n < 1:
        raise ValueError("--detect_every_n must be >= 1")

    root = ensure_dir(args.root_folder)
    intr_dir = ensure_dir(args.intrinsics_dir)

    devs = RealSenseCamera.list_devices()
    if len(devs) == 0:
        raise RuntimeError("No RealSense devices found.")

    idx_serial_pairs, map_path = build_camera_index_pairs(devs, intr_dir)

    if len(idx_serial_pairs) == 0:
        raise RuntimeError(
            "No usable cameras found after applying device_map.json. "
            "Run Step1_dump_intrinsics.py again or reconnect mapped devices."
        )

    print("[INFO] detected devices:")
    for idx, serial in idx_serial_pairs:
        print(f"  cam{idx}: {serial}  ({devs.get(serial, '?')})")

    if map_path is not None:
        print(f"[INFO] using device_map: {map_path}")

    print(f"[INFO] requested stream: {args.width}x{args.height} @ {args.fps}fps")
    print(f"[INFO] save_depth: {args.save_depth}")
    if args.save_depth:
        print("[INFO] depth align_to_color:", "OFF" if args.no_align_depth_to_color else "ON")

    if args.save_depth and len(idx_serial_pairs) >= 4 and (args.width >= 1280 or args.height >= 720) and args.fps >= 15:
        print("[WARN] high-res + depth + >=4 cams + >=15fps 조합은 타임아웃 가능성이 높을 수 있습니다.")
        print("[WARN] 그래도 허브/USB 환경이 충분하면 테스트 가능. 문제 발생 시 fps를 6~10으로 낮추세요.")

    # 카메라 시작
    cams: Dict[int, RealSenseCamera] = {}
    start_failures = {}

    for ci, serial in idx_serial_pairs:
        try:
            print(f"[INFO] starting cam{ci} ({serial}) ...")
            cam = RealSenseCamera(
                serial=serial,
                width=args.width,
                height=args.height,
                fps=args.fps,
                use_color=True,
                use_depth=args.save_depth,
                align_depth_to_color=(not args.no_align_depth_to_color),
                warmup_frames=10,
                frame_timeout_ms=args.camera_frame_timeout_ms,
                log_timeouts=args.log_cam_timeouts,
                log_errors=args.log_cam_errors,
            )
            cam.start()
            cams[ci] = cam
            print(f"[INFO] started cam{ci}")
            time.sleep(args.startup_stagger_sec)
        except Exception as e:
            start_failures[ci] = str(e)
            print(f"[WARN] failed to start cam{ci}: {e}")

    if len(cams) < args.min_started_cams:
        raise RuntimeError(
            f"Only {len(cams)} cameras started successfully. "
            f"Need at least {args.min_started_cams}."
        )

    if args.require_all_detected and len(cams) != len(idx_serial_pairs):
        failed = sorted(set(ci for ci, _ in idx_serial_pairs) - set(cams.keys()))
        raise RuntimeError(
            f"Started {len(cams)} / expected {len(idx_serial_pairs)} cameras. "
            f"Failed cams: {failed}"
        )

    print(f"[INFO] started cameras: {sorted(cams.keys())}")

    # intrinsics 존재 확인
    for ci in cams:
        intr_path = os.path.join(intr_dir, f"cam{ci}.npz")
        if not os.path.exists(intr_path):
            print(f"[WARN] intrinsics not found: {intr_path} (Step1_dump_intrinsics.py 필요)")

    cfg = CubeConfig()
    cube = ArucoCubeTarget(cfg)

    meta = {"root_folder": os.path.abspath(root), "captures": []}
    meta_path = os.path.join(root, "meta.json")

    for ci in cams:
        ensure_dir(os.path.join(root, f"cam{ci}"))

    event_id = 0
    stable_cnt = {ci: 0 for ci in cams}
    last_save_t = 0.0

    last_stats_log_t = time.monotonic()
    prev_cam_stats = {ci: cam.get_stats() for ci, cam in cams.items()}

    last_detect_result: Dict[int, Tuple[Optional[list], Optional[object]]] = {
        ci: (None, None) for ci in cams
    }
    loop_idx = 0

    print("\nControls:")
    print("  SPACE : save once (all active cams must satisfy min_markers & stable_frames)")
    print("  ESC/q : quit\n")

    try:
        while True:
            loop_idx += 1
            run_detect_this_loop = (loop_idx % args.detect_every_n == 0)

            frames: Dict[int, dict] = {}
            all_ok = True

            for ci, cam in cams.items():
                color, depth, ts_ms = cam.get_latest()

                if color is None:
                    all_ok = False
                    stable_cnt[ci] = 0
                    continue

                if run_detect_this_loop:
                    corners, ids = detect_markers(cube, color, args.detect_width)
                    last_detect_result[ci] = (corners, ids)
                else:
                    corners, ids = last_detect_result.get(ci, (None, None))

                ok = (ids is not None) and (len(ids) >= args.min_markers)

                if ok:
                    stable_cnt[ci] += 1
                else:
                    stable_cnt[ci] = 0
                    all_ok = False

                frames[ci] = {
                    "color": color,
                    "depth": depth,
                    "ts_ms": ts_ms,
                    "ok": ok,
                    "ids": ([] if ids is None else [int(x) for x in ids]),
                    "corners": corners,
                    "ids_np": ids,
                }

            if len(frames) != len(cams):
                all_ok = False

            # 화면 표시
            if args.show:
                for ci in sorted(frames.keys()):
                    img = frames[ci]["color"].copy()
                    ids_np = frames[ci]["ids_np"]
                    corners = frames[ci]["corners"]

                    if ids_np is not None:
                        try:
                            cv2.aruco.drawDetectedMarkers(img, corners, ids_np)
                        except Exception:
                            pass

                    txt = f"cam{ci} ok={frames[ci]['ok']} stable={stable_cnt[ci]} ids={frames[ci]['ids']}"
                    cv2.putText(
                        img,
                        txt,
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (255, 255, 255),
                        2,
                    )

                    img = resize_for_show(img, args.show_scale)
                    cv2.imshow(f"cam{ci}", img)

            # 카메라 통계 로그
            if args.log_cam_stats_sec > 0.0:
                now_mono = time.monotonic()
                dt = now_mono - last_stats_log_t
                if dt >= args.log_cam_stats_sec:
                    print(f"[CAM_STATS] dt={dt:.2f}s")
                    for ci in sorted(cams.keys()):
                        s = cams[ci].get_stats()
                        p = prev_cam_stats.get(ci, {})

                        d_frames = int(s.get("frames_received", 0)) - int(p.get("frames_received", 0))
                        d_timeouts = int(s.get("wait_timeouts", 0)) - int(p.get("wait_timeouts", 0))
                        d_errors = int(s.get("loop_errors", 0)) - int(p.get("loop_errors", 0))
                        d_stale = int(s.get("stale_frames", 0)) - int(p.get("stale_frames", 0))

                        fps_est = (d_frames / dt) if dt > 0.0 else 0.0

                        print(
                            f"  cam{ci} fps~{fps_est:.1f} "
                            f"(+frames={d_frames}, +timeouts={d_timeouts}, "
                            f"+errors={d_errors}, +stale={d_stale})"
                        )
                        prev_cam_stats[ci] = s

                    last_stats_log_t = now_mono

            key = cv2.waitKey(1) & 0xFF
            now_ms = time.time() * 1000.0

            manual_trigger = (key == 32)   # SPACE
            quit_trigger = (key == 27) or (key == ord("q"))

            if quit_trigger:
                break

            if args.auto_save:
                if (
                    all_ok
                    and all(stable_cnt[ci] >= args.stable_frames for ci in cams)
                    and (now_ms - last_save_t) >= args.cooldown_ms
                ):
                    manual_trigger = True

            if manual_trigger:
                if not frames:
                    print("[INFO] save blocked: no valid frames")
                    continue

                if not (
                    all_ok
                    and all(stable_cnt[ci] >= args.stable_frames for ci in cams)
                ):
                    print("[INFO] save blocked: not all cams stable/ok")
                    continue

                cap_rec = save_capture_set(
                    root=root,
                    event_id=event_id,
                    frames=frames,
                    save_depth=args.save_depth,
                    save_jpg_quality=args.save_jpg_quality,
                )

                meta["captures"].append(cap_rec)
                with open(meta_path, "w") as f:
                    json.dump(meta, f, indent=2)

                print(f"[SAVE] event_id={event_id} -> meta.json updated ({len(meta['captures'])} captures)")
                event_id += 1
                last_save_t = now_ms

    finally:
        for cam in cams.values():
            try:
                cam.stop()
            except Exception:
                pass
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()