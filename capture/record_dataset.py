# -*- coding: utf-8 -*-
"""엔터 한 번으로 로봇 자세와 촬영을 한 레코드로 적는 PC측 데이터셋 레코더.

로봇측 상대 스크립트: capture/robot/pose_query_server.py (Python 2, ZEUS 컨트롤러)

무엇을 하는가
  로봇을 원하는 자세로 옮긴 뒤 PC 터미널에서 엔터를 치면
    1. 로봇에 현재 상태를 물어 flange 자세(TCP)와 관절값을 받고
    2. (카메라를 쓰면) 테이블 고정 3대를 한꺼번에 잡아 ChArUco 를 검출하고
    3. 자세·관절·이미지·검출 결과를 하나의 레코드로 묶어 meta.json 에 즉시 적는다.

capture/shah_capture_client.py 와 다른 점
  meta.json 스키마는 같고(SCHEMA_VERSION, 레코드 구조, 게이트 규칙 전부 공유).
  누가 촬영을 시작하는지가 다름.

      shah_capture_client.py : 로봇 터미널이 주도. 로봇이 capture 를 보내면 PC 가 응답
      record_dataset.py      : PC 터미널이 주도. 엔터를 치면 PC 가 로봇에 상태를 요청

  자세를 미리 티칭해 두고 한 번에 순회할 때는 전자가, 손으로 자세를 잡아 가며
  데이터셋을 조금씩 늘릴 때는 후자가 편하다. 산출물은 같은 형식이므로 하류
  분석 코드는 둘을 구분할 필요가 없다.

규약 (어기면 결과가 틀어질 수 있으니 주의 — docs/real_shah_capture.md §8.2, §8.3)
  * 로봇 자세는 tool 1(플랜지) 기준 [x,y,z mm, rz,ry,rx deg]
  * 회전은 Rz @ Ry @ Rx
  * 저장되는 변환은 전부 T_destination_source, 병진 단위는 metre

조작 (자세한 목록은 실행 후 help)
  로봇 조작은 SSH 로봇 콘솔의 zeus_jog_onboard.py 에서 계속 한다. 이 레코더는
  자세를 관찰만 하고 로봇을 움직이지 않는다 (그렇게 해서 zeus_jog_onboard.py
  와 컨트롤러를 두고 싸우지 않는다 — capture/robot/pose_query_server.py 참조).

  PC 터미널에서 하는 것은 다음 뿐이다:
    Enter        지금 자세를 기록 (자세 + 관절 + 카메라 3대 촬영)
    z / undo     마지막 기록 취소 (이미지도 함께 지운다)
    s            기록하지 않고 현재 자세만 확인
    list / div   기록 목록 / 평균 상대회전
    q            종료

저장 위치 — 이 스크립트를 실행한 PC 안에만 쌓인다
  이미지, meta.json 도 전부 로컬 디스크에 쓴다.
  기본 경로는 --dataset-root 로 정하며 기본값은 홈 디렉터리 아래 ~/shah_data
  저장소 폴더 안에 두고 VS Code 에서 바로 보고 싶다면 --dataset-root ./data
  를 쓰면 된다.

저장 구조 — 어떤 알고리즘의 데이터인지가 폴더 이름이 됨.

  <dataset-root>/
    shah/
      dataset_index.json              이 알고리즘 아래 세션 목록
      session_20260904_1530/
        meta.json                     세션 헤더 + captures[]
        cam0/rgb_00000.jpg ...
        cam1/ ...
        cam3/ ...
    tsai/
      ...

갱신 (데이터셋은 계속 늘어남)
  --resume 를 주면 그 알고리즘의 가장 최근 세션에 이어 적는다. event_id 가
  이어지고, 이어 붙인 이력이 append_history 에 한 줄씩 남는다.

  단 세션 하나는 '카메라가 한 번도 움직이지 않은 구간'이어야 한다. 카메라를
  건드렸거나 보드를 다시 붙였다면 --resume 하지 말고 새 세션으로 시작해야 한다.
  한 meta.json 안에 서로 다른 Y = T_base_cam 이 섞이면 틀린 답을 낸다.

실행 예
  # 자세만 기록
  python capture/record_dataset.py --algorithm shah --no-camera \
      --robot-host 192.168.0.23

  # 촬영까지 함께 기록
  python capture/record_dataset.py --algorithm shah \
      --robot-board-id-start 50 --show

  # 이전 세션 갱신
  python capture/record_dataset.py --algorithm shah --resume
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from datetime import datetime
from itertools import combinations
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT))

from board_config import ROBOT_BOARD, TABLE_BOARD
from shah_capture_client import (
    CAPTURE_BLOCK,
    DEFAULT_CAMERAS,
    SCHEMA_VERSION,
    BoardDetector,
    build_session_header as build_capture_header,
    capture_once,
    invert,
    open_cameras,
    solve_and_report,
    zeus_pose_to_matrix,
)

DATASET_SCHEMA_VERSION = "pose_dataset_v1"
# pose_query_server.py 와 같아야 한다. 12346/12348 은 기존 서버들이,
# 12349 는 i611usr/model.py 가 이미 쓴다.
DEFAULT_ROBOT_PORT = 12350
ROTATION_DIVERSITY_TARGET_DEG = 40.0
RECORDER_ID = "capture/record_dataset.py"

# 로봇 서버마다 자세 필드 이름이 다르다. 먼저 나오는 것을 쓴다.
#   flange_pose_6dof / joints_6dof : capture/robot/pose_query_server.py
#   tcp_6dof / joint_6dof          : i611usr/sam3d_calb/robot_pose_server.py
POSE_FIELDS = ("flange_pose_6dof", "tcp_6dof", "pose_6dof")
JOINT_FIELDS = ("joints_6dof", "joint_6dof")


def now_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


# ────────────────────────────────────────────────────────────────
# 로봇 링크 — 요청/응답 한 쌍 (newline-delimited JSON)
# ────────────────────────────────────────────────────────────────

class RobotLink:
    """pose_query_server.py 에 상태를 묻고 jog 를 시키는 클라이언트."""

    def __init__(self, host: str, port: int, timeout_s: float = 10.0):
        self.address = (host, port)
        self.sock = socket.create_connection(self.address, timeout=timeout_s)
        self.sock.settimeout(timeout_s)
        self.buffer = ""
        self.pose_command = "get_state"
        self.tool_warned = False
        self.joint_warned = False

    def send(self, payload: dict) -> dict:
        """응답을 그대로 돌려준다. status 가 ok 가 아니어도 예외를 내지 않는다."""
        self.sock.sendall((json.dumps(payload) + "\n").encode("utf-8"))
        while "\n" not in self.buffer:
            chunk = self.sock.recv(8192).decode("utf-8")
            if not chunk:
                raise ConnectionError("로봇 연결이 끊겼다")
            self.buffer += chunk
        line, self.buffer = self.buffer.split("\n", 1)
        try:
            return json.loads(line)
        except ValueError as error:
            raise RuntimeError(f"로봇 응답을 읽지 못했다: {error}") from error

    def request(self, payload: dict) -> dict:
        response = self.send(payload)
        if response.get("status") != "ok":
            # detail 은 pose_query_server.py, reason 은 기존 서버가 쓰는 이름이다.
            raise RuntimeError("로봇 오류: {}".format(
                response.get("detail") or response.get("reason") or response))
        return response

    def get_state(self) -> dict:
        """{'flange_pose_6dof', 'joints_6dof', 'tool'} 로 정규화해 돌려준다.

        get_state 를 모르는 서버(i611usr/sam3d_calb/robot_pose_server.py)를 만나면
        한 번만 get_pose 로 물러선다. 필드 이름도 그쪽 이름을 함께 받는다.
        """
        response = self.send({"command": self.pose_command})
        if response.get("status") != "ok" and self.pose_command == "get_state":
            fallback = self.send({"command": "get_pose"})
            if fallback.get("status") == "ok":
                print("[주의] 이 서버는 get_state 를 모른다. get_pose 로 진행한다 "
                      "— 기존 로봇 서버로 보인다. 아래 tool 경고를 반드시 확인하라.")
                self.pose_command = "get_pose"
                response = fallback
        if response.get("status") != "ok":
            raise RuntimeError("로봇 오류: {}".format(
                response.get("detail") or response.get("reason") or response))
        return self._normalize(response)

    def _normalize(self, response: dict) -> dict:
        pose = next((response[k] for k in POSE_FIELDS if response.get(k)), None)
        joints = next((response[k] for k in JOINT_FIELDS if response.get(k)), None)
        if pose is None or len(pose) < 6:
            raise RuntimeError(f"응답에 6자유도 자세가 없다: {sorted(response)}")
        if joints is None and not self.joint_warned:
            print("[경고] 이 서버는 관절값을 주지 않는다. 레코드의 "
                  "capture_robot_joints_6dof 가 null 로 남는다.")
            self.joint_warned = True

        tool = response.get("tool")
        source = response.get("source")
        if tool is None and source == "shm" and not self.tool_warned:
            # shm-only 서버는 컨트롤러의 현재 tool 을 확정할 수 없다. 이건 오류가
            # 아니라 이 방식의 한계다. 한 번만 안내한다.
            print("[안내] pose_query_server.py (shm) 는 컨트롤러의 현재 tool 을 알 수 없다.")
            print("       zeus_jog_onboard.py 가 tool 1(플랜지)을 쓰고 있는지 확인하라 —")
            print("       그리퍼 오프셋이 섞인 tool 이라면 결과가 어긋난다 (docs §8.2).")
            self.tool_warned = True
        elif tool is not None and tool != 1 and not self.tool_warned:
            print("[경고] 이 서버가 돌려주는 자세는 tool {} 기준이다 (tool 1 이 아니다)."
                  .format(tool))
            print("       이 리그의 규약은 tool 1(플랜지)다. 그리퍼 오프셋이 섞인 자세를")
            print("       쓰면 보드 오프셋이 이중으로 들어가 결과가 조용히 틀어진다")
            print("       (docs/real_shah_capture.md §8.2). 로봇측 서버를 확인하라.")
            self.tool_warned = True

        return {
            "flange_pose_6dof": [float(v) for v in pose[:6]],
            "joints_6dof": [float(v) for v in joints[:6]] if joints else None,
            "tool": tool,
            "source": source,
        }

    # 로봇 이동은 이 클라이언트가 하지 않는다. 조작은 SSH 로봇 콘솔의
    # zeus_jog_onboard.py 에서 한다. pose_query_server.py 는 자세만 관찰한다.

    def close(self) -> None:
        try:
            self.sock.sendall((json.dumps({"command": "bye"}) + "\n").encode("utf-8"))
        except OSError:
            pass
        try:
            self.sock.close()
        except OSError:
            pass


# ────────────────────────────────────────────────────────────────
# 회전 다양성 — 촬영 중에 목표에 도달했는지 알기 위한 지표
# docs/real_shah_capture.md §4: 5deg -> 95mm, 40deg -> 5.4mm
# ────────────────────────────────────────────────────────────────

def rotation_diversity_deg(records: list[dict]) -> float:
    """기록된 자세들의 쌍별 상대회전 평균 (deg)."""
    matrices = [np.asarray(r["robot_pose_matrix_4x4"], float) for r in records]
    if len(matrices) < 2:
        return 0.0
    angles = [
        np.degrees(np.linalg.norm(cv2.Rodrigues((invert(a) @ b)[:3, :3])[0]))
        for a, b in combinations(matrices, 2)
    ]
    return float(np.mean(angles))


def diversity_line(records: list[dict]) -> str:
    value = rotation_diversity_deg(records)
    if len(records) < 2:
        verdict = "자세 2개부터 계산된다"
    elif value >= ROTATION_DIVERSITY_TARGET_DEG:
        verdict = "충분"
    elif value >= ROTATION_DIVERSITY_TARGET_DEG * 0.6:
        verdict = "부족 — 더 기울여라"
    else:
        verdict = "심각히 부족 — 평행이동만 하고 있다"
    return (f"자세 {len(records)}개 / 평균 상대회전 {value:.1f}deg "
            f"(목표 {ROTATION_DIVERSITY_TARGET_DEG:.0f}deg) -> {verdict}")


def fmt6(values) -> str:
    return "[" + ", ".join(f"{float(v):.2f}" for v in values) + "]"


# ────────────────────────────────────────────────────────────────
# 세션 디렉터리 — <dataset-root>/<algorithm>/<session>/
# ────────────────────────────────────────────────────────────────

def session_dirs(algorithm_dir: Path) -> list[Path]:
    if not algorithm_dir.exists():
        return []
    found = [p for p in algorithm_dir.iterdir()
             if p.is_dir() and (p / "meta.json").exists()]
    return sorted(found, key=lambda p: (p / "meta.json").stat().st_mtime)


def resolve_session(args) -> tuple[Path, Path, dict | None]:
    """(algorithm_dir, session_dir, 기존 meta 또는 None) 을 정한다."""
    algorithm_dir = Path(args.dataset_root).expanduser().resolve() / args.algorithm

    if args.session:
        session_dir = algorithm_dir / args.session
    elif args.resume:
        existing = session_dirs(algorithm_dir)
        if not existing:
            raise SystemExit(
                f"--resume 인데 이어붙일 세션이 없다: {algorithm_dir}\n"
                f"  처음이라면 --resume 없이 실행하라.")
        session_dir = existing[-1]
    else:
        session_dir = algorithm_dir / datetime.now().strftime("session_%Y%m%d_%H%M%S")

    meta_path = session_dir / "meta.json"
    existing_meta = None
    if meta_path.exists():
        with meta_path.open(encoding="utf-8") as handle:
            existing_meta = json.load(handle)
    return algorithm_dir, session_dir, existing_meta


def stored_capture_mode(meta: dict) -> str:
    """이 세션이 이미지까지 담고 있는지. dataset 블록이 없으면 카메라 목록으로 짐작한다."""
    recorded = (meta.get("dataset") or {}).get("capture_mode")
    if recorded:
        return recorded
    return "pose_and_image" if meta.get("cam_indices") else "pose_only"


def ensure_dataset_block(meta: dict, args, cameras, session_dir,
                         algorithm_dir: Path) -> dict:
    """이어붙일 세션에 dataset 블록을 보강한다.

    shah_capture_client.py 로 찍은 세션에는 이 블록이 없다. 그대로 두면 기록할 때
    KeyError 가 난다. 알 수 있는 값만 채우고, 모르는 값은 지어내지 않는다.
    """
    block = meta.setdefault("dataset", {})
    defaults = {
        "schema_version": DATASET_SCHEMA_VERSION,
        "algorithm": args.algorithm,
        "dataset_root": str(Path(args.dataset_root).expanduser().resolve()),
        "algorithm_dir": str(algorithm_dir),
        "session_id": session_dir.name,
        "capture_mode": stored_capture_mode(meta),
        "recorder": "unknown (record_dataset.py 이전에 만들어진 세션)",
        "created_at": None,          # 언제 만들어졌는지 알 수 없다
        "append_history": [],
    }
    for key, value in defaults.items():
        block.setdefault(key, value)
    block["updated_at"] = now_iso()
    return block


def check_resume_compatible(meta: dict, args, cameras, detector) -> None:
    """이어 붙이기 전에 이 세션과 지금 설정이 같은 실험인지 확인한다.

    해상도나 카메라 구성이 달라지면 K 가 맞지 않고, 보드 ID 가 달라지면
    검출 대상이 달라진다. 어느 쪽이든 한 meta.json 에 섞이면 안 된다.
    """
    config = meta.get("capture_config", {})
    problems = []
    stored_had_cameras = bool(meta.get("cam_indices"))

    if cameras and stored_had_cameras:
        if bool(config.get("save_depth")) != bool(args.save_depth):
            problems.append(
                f"save_depth {config.get('save_depth')} != {args.save_depth}")
        stored_size = (config.get("width"), config.get("height"))
        if stored_size != (args.width, args.height):
            problems.append(
                f"해상도 {stored_size[0]}x{stored_size[1]} != "
                f"{args.width}x{args.height} (K 가 해상도에 종속이다)")
        stored = sorted(meta.get("cam_indices", []))
        current = sorted(c.index for c in cameras)
        if stored and stored != current:
            problems.append(f"카메라 구성 {stored} != {current}")
        stored_id = (meta.get("board_config") or {}).get("marker_id_start")
        if detector is not None and stored_id is not None \
                and int(stored_id) != detector.marker_id_start:
            problems.append(
                f"로봇 보드 marker_id_start {stored_id} != {detector.marker_id_start}")

    mode = stored_capture_mode(meta)
    current_mode = "pose_and_image" if cameras else "pose_only"
    if mode != current_mode:
        problems.append(f"기록 방식 {mode} != {current_mode}")

    if problems:
        print("\n[중단] 이 세션에 이어 붙일 수 없다:", file=sys.stderr)
        for item in problems:
            print(f"  - {item}", file=sys.stderr)
        print("  새 세션으로 시작하라 (--resume 없이 실행).", file=sys.stderr)
        raise SystemExit(2)


def build_header(args, cameras, detector, session_dir, algorithm_dir) -> dict:
    """새 세션 헤더. 카메라를 쓰면 촬영 클라이언트와 동일한 헤더를 재사용한다."""
    if cameras:
        header = build_capture_header(args, cameras, detector, session_dir)
    else:
        header = {
            "schema_version": SCHEMA_VERSION,
            "root_folder": str(session_dir),
            "session_allocation": None,
            "gripper_cam_idx": None,
            "n_fixed_cams": 0,
            "n_gripper_cams": 0,
            "cam_indices": [],
            "cam_serials": {},
            "board_config_source": "capture/board_config.py:ROBOT_BOARD",
            "board_config": ROBOT_BOARD.to_dict(),
            "table_board_config": TABLE_BOARD.to_dict(),
            "capture_config": {
                "schema_version": "capture_config_v1",
                "intrinsics_dir": None,
                "width": None,
                "height": None,
                "fps": None,
                "save_depth": False,
                "settle_time_s": args.settle_time_s,
                "cross_camera_timestamp_basis": None,
                "min_charuco_corners": None,
                "max_charuco_reproj_px": None,
                "robot_pose_convention":
                    "tool 1 (flange), [x,y,z mm, rz,ry,rx deg], R = Rz@Ry@Rx",
                "transform_convention": "T_destination_source, translation in metre",
                "color_photometry": {},
            },
            "captures": [],
        }

    header["dataset"] = {
        "schema_version": DATASET_SCHEMA_VERSION,
        "algorithm": args.algorithm,
        "dataset_root": str(Path(args.dataset_root).expanduser().resolve()),
        "algorithm_dir": str(algorithm_dir),
        "session_id": session_dir.name,
        "capture_mode": "pose_and_image" if cameras else "pose_only",
        "recorder": RECORDER_ID,
        "created_at": now_iso(),
        "updated_at": now_iso(),
        "append_history": [],
    }
    return header


# ────────────────────────────────────────────────────────────────
# 레코드
# ────────────────────────────────────────────────────────────────

def make_pose_record(index: int, state: dict) -> dict:
    """카메라 없이 자세만 기록할 때의 레코드. cams 만 비어 있고 나머지는 동일하다."""
    pose6 = [float(v) for v in state["flange_pose_6dof"]]
    matrix = zeus_pose_to_matrix(pose6).tolist()
    return {
        "event_id": index,
        "capture_index": index,
        "capture_block": CAPTURE_BLOCK,
        "robot_pose_6dof": pose6,
        "robot_pose_matrix_4x4": matrix,
        # Step3 계열이 찾는 이름. 이 리그에서는 플랜지 자세와 같은 값이다.
        "capture_gripper_pose_6dof": pose6,
        "capture_gripper_pose_matrix_4x4": matrix,
        # 로봇이 주지 않은 값을 1 이나 [] 로 채우지 않는다. 없으면 null 로 남긴다.
        "capture_robot_joints_6dof": state.get("joints_6dof"),
        "tool": state.get("tool"),
        "cams": {},
        "capture_gate": {
            "pass": True,
            "n_cams_passed": 0,
            "n_cams_required": 0,
            "reason": "pose_only (카메라 없이 자세만 기록)",
        },
    }


def delete_record_images(record: dict, session_dir: Path) -> None:
    for entry in record.get("cams", {}).values():
        for key in ("rgb_path", "depth_path"):
            relative = entry.get(key)
            if relative:
                (session_dir / relative).unlink(missing_ok=True)


# ────────────────────────────────────────────────────────────────
# 저장
# ────────────────────────────────────────────────────────────────

def write_json(path: Path, payload: dict) -> None:
    """임시 파일에 쓰고 교체한다. 쓰는 도중 중단돼도 이전 파일이 남는다."""
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, ensure_ascii=False)
    temporary.replace(path)


def update_dataset_index(algorithm_dir: Path, session_dir: Path, meta: dict) -> None:
    """알고리즘 폴더 하나에 세션 목록을 모아 둔다. 데이터셋이 늘어나도 한눈에 보인다."""
    path = algorithm_dir / "dataset_index.json"
    index = {"schema_version": DATASET_SCHEMA_VERSION,
             "algorithm": meta["dataset"]["algorithm"], "sessions": {}}
    if path.exists():
        try:
            with path.open(encoding="utf-8") as handle:
                index = json.load(handle)
            index.setdefault("sessions", {})
        except (OSError, ValueError):
            print(f"[주의] {path} 를 읽지 못해 새로 만든다")

    index["sessions"][session_dir.name] = {
        "path": str(session_dir),
        "capture_mode": meta["dataset"]["capture_mode"],
        "n_captures": len(meta["captures"]),
        "n_gate_passed": sum(1 for r in meta["captures"]
                             if r.get("capture_gate", {}).get("pass")),
        "rotation_diversity_deg": round(rotation_diversity_deg(meta["captures"]), 2),
        "created_at": meta["dataset"]["created_at"],
        "updated_at": meta["dataset"]["updated_at"],
    }
    index["updated_at"] = now_iso()
    write_json(path, index)


# ────────────────────────────────────────────────────────────────
# 대화 루프
# ────────────────────────────────────────────────────────────────

# 로봇은 SSH 콘솔의 zeus_jog_onboard.py 에서 조작한다. pose_query_server.py 는
# shm 만 읽는 관찰자이므로 이 레코더는 로봇에 이동 명령을 내리지 않는다.
# 손이 조작키를 두드렸을 때 아무 일도 안 일어나면 프로그램이 멈춘 줄 알기 쉽다.
# 그래서 조작키들도 무시하지 않고 "그건 로봇 콘솔에서 하라"고 알려 준다.
JOG_HINT_KEYS = tuple("wsadrfikjluo")
STEP_HINT_KEYS = tuple("12345") + ("[", "]")

MENU = """
==================================================================
  촬영 기록 레코더 — 엔터 한 번에 자세와 촬영을 함께 적는다
------------------------------------------------------------------
 [ PC 터미널에서 하는 것 ]
   Enter            지금 자세를 기록 (TCP + 관절 + 카메라 3대 촬영)
   s                기록하지 않고 현재 자세만 확인
   z 또는 undo      마지막 기록 취소 (이미지도 함께 지운다)
   list             기록 목록
   div              평균 상대회전 (목표 {target:.0f}deg)
   q                종료

 [ 로봇 조작 — 이 터미널이 아니라 SSH 로봇 콘솔에서 ]
   zeus_jog_onboard.py 를 그대로 쓰라 (w/s a/d r/f, i/k j/l u/o, [ ], 1~5).
   이 레코더는 shm 을 읽어 자세를 가져올 뿐이라 로봇을 움직이지 못한다.
   조작을 위한 SSH 터미널 하나, 이 레코더용 로컬 터미널 하나 - 총 2개면 된다.

 ※ 자세를 잡은 뒤 엔터. 로봇이 완전히 멎었을 때 치라 (진동이 남으면 흐릿함).
 ※ 평행이동만 하지 말 것. 자세마다 rz/ry/rx 를 20~30deg 씩 바꿔라.
==================================================================
"""


def record_once(args, robot, cameras, detector, meta, session_dir) -> dict | None:
    """엔터 한 번의 처리. 자세를 먼저 읽고 그 다음 촬영한다."""
    index = len(meta["captures"])
    state = robot.get_state()

    if cameras:
        message = {
            "index": index,
            "flange_pose_6dof": state["flange_pose_6dof"],
            "joints_6dof": state.get("joints_6dof"),
            "tool": state.get("tool"),
        }
        record, counts, detail = capture_once(
            cameras, detector, message, session_dir, args)
    else:
        record = make_pose_record(index, state)
        counts, detail = {}, []

    record["recorded_at"] = now_iso()
    record["capture_mode"] = meta["dataset"]["capture_mode"]

    if args.reject_gate_fail and not record["capture_gate"]["pass"]:
        # 이미지는 남기지 않고 레코드도 버린다. 기본값은 False —
        # 실패도 기록하는 것이 이 저장소의 원칙이다 (docs §10 설계원칙 1).
        delete_record_images(record, session_dir)
        print(f"  거부됨 (기록하지 않음): {record['capture_gate']['reason']}")
        return None

    meta["captures"].append(record)
    status = "pass" if record["capture_gate"]["pass"] else "gate_fail"
    print(f"[{index:3d}] {status}  flange={fmt6(record['robot_pose_6dof'])}")
    if counts:
        print(f"       corners={counts}"
              + (f"  | {'; '.join(detail)}" if detail else ""))
    return record


def interactive(args, robot, cameras, detector, meta, session_dir,
                algorithm_dir, meta_path, run_entry) -> None:
    def flush():
        meta["dataset"]["updated_at"] = now_iso()
        run_entry["ended_at"] = meta["dataset"]["updated_at"]
        run_entry["records_added"] = len(meta["captures"]) - run_entry["n_before"]
        write_json(meta_path, meta)
        update_dataset_index(algorithm_dir, session_dir, meta)

    flush()
    print(MENU.format(target=ROTATION_DIVERSITY_TARGET_DEG))
    print(f"저장 위치 : {session_dir}   (이 PC 로컬 디스크)")
    print(f"알고리즘  : {args.algorithm}")
    print(f"기록 방식 : {meta['dataset']['capture_mode']}")
    print(f"기존 기록 : {len(meta['captures'])}건")
    print(diversity_line(meta["captures"]))
    print()

    def show_pose():
        state = robot.get_state()
        source = state.get("source") or "?"
        print(f"  flange: {fmt6(state['flange_pose_6dof'])}  (source={source})")
        joints = state.get("joints_6dof")
        print(f"  joints: {fmt6(joints) if joints else '(로봇이 주지 않음)'}")

    while True:
        try:
            typed = input("> ")
        except (EOFError, KeyboardInterrupt):
            print()
            break

        command = typed.strip()
        lowered = command.lower()

        # ─── 기록 ───
        if typed and not command:
            # 스페이스만 치고 엔터 → 현재 자세만 확인. 기록은 순수한 빈 줄일 때만.
            try:
                show_pose()
            except Exception as error:
                print(f"[오류] {type(error).__name__}: {error}")

        elif command == "":
            try:
                record = record_once(args, robot, cameras, detector, meta, session_dir)
            except Exception as error:
                # 한 번의 기록이 실패해도 세션 전체를 잃지 않는다. 지금까지의
                # 레코드는 이미 파일에 있고, 다시 엔터를 치면 이어서 기록된다.
                print(f"[오류] 기록 실패: {type(error).__name__}: {error}")
                continue
            if record is not None:
                flush()
                print(f"       {diversity_line(meta['captures'])}")

        elif lowered in ("s", "p", "show"):
            try:
                show_pose()
            except Exception as error:
                print(f"[오류] {type(error).__name__}: {error}")

        # ─── 기록 취소 · 확인 ───
        elif lowered in ("z", "undo"):
            if not meta["captures"]:
                print("  기록이 비어 있다.")
                continue
            removed = meta["captures"].pop()
            delete_record_images(removed, session_dir)
            flush()
            print(f"  [{removed['event_id']}] 취소됨. {diversity_line(meta['captures'])}")

        elif lowered in ("list", "ls"):
            for record in meta["captures"]:
                mark = " " if record["capture_gate"]["pass"] else "!"
                print(f"  [{record['event_id']:3d}]{mark} "
                      f"{fmt6(record['robot_pose_6dof'])}")
            print(f"  {diversity_line(meta['captures'])}")

        elif lowered in ("div", "diversity"):
            print(f"  {diversity_line(meta['captures'])}")

        elif lowered in ("q", "quit", "exit"):
            break

        elif lowered in ("help", "?", "h"):
            print(MENU.format(target=ROTATION_DIVERSITY_TARGET_DEG))

        # 로봇 조작키를 여기서 눌러도 아무 일도 안 일어난다는 걸 명확히 알려 준다.
        # 무시하면 사용자는 프로그램이 멎은 줄 안다.
        elif lowered in JOG_HINT_KEYS or command in STEP_HINT_KEYS or lowered in ("g", "h", "x", "m"):
            print(f"  [{command}] 로봇 조작 키다. 이 터미널이 아니라 SSH 로봇 콘솔의 "
                  f"zeus_jog_onboard.py 에서 눌러라.")
            print("       이 레코더는 자세를 관찰만 하고 로봇에 이동 명령은 내리지 않는다.")

        else:
            print(f"[오류] 알 수 없는 명령: {command}   (도움말: help)")

    flush()


# ────────────────────────────────────────────────────────────────

def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="엔터로 로봇 자세와 촬영을 함께 기록하는 데이터셋 레코더")

    dataset = parser.add_argument_group("데이터셋")
    dataset.add_argument("--dataset-root", default=str(Path.home() / "shah_data"),
                         help="데이터셋 최상위. 기본 ~/shah_data (저장소 밖)")
    dataset.add_argument("--algorithm", default="shah",
                         help="이 데이터로 돌릴 알고리즘. 폴더 이름이 된다 "
                              "(shah / tsai / park / horaud / andreff / daniilidis)")
    dataset.add_argument("--session", default=None,
                         help="세션 폴더 이름. 생략하면 시각으로 새로 만든다")
    dataset.add_argument("--resume", action="store_true",
                         help="이 알고리즘의 가장 최근 세션에 이어 적는다. "
                              "카메라를 움직이지 않았을 때만 쓸 것")

    robot = parser.add_argument_group("로봇")
    robot.add_argument("--robot-host", default="192.168.0.23")
    robot.add_argument("--robot-port", type=int, default=DEFAULT_ROBOT_PORT,
                       help=f"pose_query_server.py 포트 (기본 {DEFAULT_ROBOT_PORT})")
    robot.add_argument("--robot-timeout-s", type=float, default=10.0)

    camera = parser.add_argument_group("카메라")
    camera.add_argument("--no-camera", dest="use_camera", action="store_false",
                        default=True,
                        help="카메라 없이 자세와 관절만 기록한다")
    camera.add_argument("--intrinsics-dir", default=str(ROOT / "intrinsics"))
    camera.add_argument("--cameras", nargs="+", default=list(DEFAULT_CAMERAS))
    camera.add_argument("--robot-board-id-start", type=int, default=None,
                        help="로봇 보드 마커 ID 시작번호. board_config.py 에 "
                             "기입돼 있으면 생략 가능")
    camera.add_argument("--width", type=int, default=1280)
    camera.add_argument("--height", type=int, default=720)
    camera.add_argument("--fps", type=int, default=15)
    camera.add_argument("--save-depth", action="store_true", default=True)
    camera.add_argument("--no-save-depth", dest="save_depth", action="store_false")
    camera.add_argument("--startup-stagger-s", type=float, default=0.8)
    camera.add_argument("--settle-time-s", type=float, default=0.5,
                        help="엔터를 친 뒤 촬영까지 기다리는 시간. 사람이 자세를 "
                             "잡고 손을 뗀 뒤이므로 자동 순회보다 짧아도 된다")
    camera.add_argument("--min-corners", type=int, default=12,
                        help=f"로봇 보드 전체 코너는 {ROBOT_BOARD.corner_count}개")
    camera.add_argument("--max-reproj-px", type=float, default=1.5)
    camera.add_argument("--jpg-quality", type=int, default=95)
    camera.add_argument("--show", action="store_true")
    camera.add_argument("--reject-gate-fail", action="store_true",
                        help="게이트를 통과하지 못한 촬영을 기록하지 않는다. "
                             "기본은 실패도 gate_reason 과 함께 남긴다")

    return parser.parse_args(argv)


def main(argv=None) -> int:
    # 한글 콘솔(Windows cp949 등)에서 일부 기호를 못 찍어 죽는 것을 막는다.
    # 기록이 아니라 화면 출력만의 문제이므로 글자를 잃더라도 진행하는 편이 낫다.
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(errors="replace")
        except (AttributeError, OSError):
            pass

    args = parse_args(argv)

    algorithm_dir, session_dir, existing_meta = resolve_session(args)

    # ─── 카메라와 검출기 ───
    cameras, detector = [], None
    if args.use_camera:
        id_start = args.robot_board_id_start
        if id_start is None:
            id_start = ROBOT_BOARD.marker_id_start
        if id_start is None:
            print("로봇 보드의 마커 ID 시작번호를 알 수 없다.", file=sys.stderr)
            print("  python capture/check_board_ids.py --live  로 확인한 뒤",
                  file=sys.stderr)
            print("  board_config.py 에 기입하거나 --robot-board-id-start 로 넘겨라.",
                  file=sys.stderr)
            print("  자세만 먼저 모으려면 --no-camera 로 실행하라.", file=sys.stderr)
            return 2
        detector = BoardDetector(ROBOT_BOARD, id_start)
        span = ROBOT_BOARD.marker_count
        print(ROBOT_BOARD.describe().replace(
            "ID 미확인", f"ID {id_start}~{id_start + span - 1}"))
        table_span = TABLE_BOARD.id_range()
        clash = sorted(detector.id_set & set(range(table_span[0], table_span[1] + 1)))
        if clash:
            print(f"[경고] 로봇 보드 ID 가 테이블 보드({table_span[0]}~{table_span[1]})와 "
                  f"{len(clash)}개 겹친다: {clash}")
            print("       촬영 중에는 테이블 보드를 치우거나 덮어라 "
                  "(docs/real_shah_capture.md §3).")

    if existing_meta is not None and existing_meta.get("captures"):
        print(f"\n기존 세션에 이어 붙인다: {session_dir}")
        print(f"  기록 {len(existing_meta['captures'])}건, "
              f"{diversity_line(existing_meta['captures'])}")
        print("  이 세션을 촬영한 뒤로 카메라를 움직였거나 보드를 다시 붙였다면")
        print("  이어 붙이면 안 된다. 그런 경우 Ctrl+C 로 중단하고 새 세션으로 시작하라.")
        answer = input("  계속하려면 yes 를 입력하라: ").strip().lower()
        if answer != "yes":
            print("중단했다.")
            return 1

    session_dir.mkdir(parents=True, exist_ok=True)

    if args.use_camera:
        cameras = open_cameras(Path(args.intrinsics_dir), args.cameras,
                               args.width, args.height, args.fps,
                               args.save_depth, args.startup_stagger_s)
        for camera in cameras:
            (session_dir / f"cam{camera.index}").mkdir(exist_ok=True)

    if existing_meta is not None:
        check_resume_compatible(existing_meta, args, cameras, detector)
        meta = existing_meta
        ensure_dataset_block(meta, args, cameras, session_dir, algorithm_dir)
    else:
        meta = build_header(args, cameras, detector, session_dir, algorithm_dir)

    run_entry = {
        "started_at": now_iso(),
        "ended_at": None,
        "robot_host": f"{args.robot_host}:{args.robot_port}",
        "recorder": RECORDER_ID,
        "n_before": len(meta["captures"]),
        "records_added": 0,
    }
    meta["dataset"]["append_history"].append(run_entry)

    meta_path = session_dir / "meta.json"

    print(f"\n로봇 접속: {args.robot_host}:{args.robot_port} ...")
    try:
        robot = RobotLink(args.robot_host, args.robot_port, args.robot_timeout_s)
    except OSError as error:
        print(f"[중단] 로봇에 접속하지 못했다: {error}", file=sys.stderr)
        print("  로봇에서 pose_query_server.py 가 실행 중인지 확인하라.",
              file=sys.stderr)
        for camera in cameras:
            camera.stop()
        return 2
    print("접속됨.")

    try:
        interactive(args, robot, cameras, detector, meta, session_dir,
                    algorithm_dir, meta_path, run_entry)
    except KeyboardInterrupt:
        # 촬영 도중 Ctrl+C. 아래 finally 가 지금까지의 기록을 마무리한다.
        print("\n중단됨.")
    finally:
        robot.close()
        if cameras and len(meta["captures"]) >= 3:
            try:
                meta["shah_field_check"] = solve_and_report(
                    meta["captures"], args.cameras)
            except Exception as error:          # 현장 확인은 실패해도 데이터는 남는다
                print(f"[주의] 현장 검증 생략: {error}")
                meta["shah_field_check"] = None
        meta["dataset"]["updated_at"] = now_iso()
        run_entry["ended_at"] = meta["dataset"]["updated_at"]
        run_entry["records_added"] = len(meta["captures"]) - run_entry["n_before"]
        write_json(meta_path, meta)
        update_dataset_index(algorithm_dir, session_dir, meta)

        for camera in cameras:
            camera.stop()
        if args.show:
            cv2.destroyAllWindows()

        print(f"\n기록 완료: {meta_path}")
        print(f"  이번 실행에서 {run_entry['records_added']}건 추가 / "
              f"세션 누적 {len(meta['captures'])}건")
        print(f"  {diversity_line(meta['captures'])}")
        print(f"  세션 목록: {algorithm_dir / 'dataset_index.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
