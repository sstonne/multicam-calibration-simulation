"""이 리그에 존재하는 두 ChArUco 보드의 단일 정의.

리그에 보드가 두 장 있고 둘 다 DICT_4X4 라서, 어느 보드를 말하는지 코드에서
항상 명시해야 한다. 규격을 여기 한 곳에만 두고 나머지는 전부 import 한다.

  로봇 보드   : 로봇 팔 끝(플랜지)에 볼트로 고정. Shah 의 관측 대상.
  테이블 보드 : 광학 테이블 위에 정지. 고정 카메라들의 월드 기준.

주의 — 마커 ID 충돌:
  두 보드가 같은 DICT_4X4 를 쓴다. 테이블 보드가 ID 5~42 를 점유하므로,
  로봇 보드가 기본값 ID 0 부터로 인쇄됐다면 5~16 번 17개 중 12개가 겹친다.
  두 보드가 한 화면에 들어오면 검출기가 마커를 섞어 ChArUco 보간을 하고,
  예외도 경고도 없이 틀린 pose 를 낸다.
  대응은 docs/real_shah_capture.md 의 "마커 ID 충돌" 절 참조.
"""

from __future__ import annotations

from dataclasses import dataclass, asdict


@dataclass(frozen=True)
class BoardConfig:
    """ChArUco 보드 하나의 물리 규격.

    squares_x / squares_y 는 OpenCV 순서(가로, 세로)다. 실험실에서 쓰는
    '5*7', '7*11' 표기는 세로*가로이므로 뒤집어서 넣는다.
    """

    name: str
    squares_x: int
    squares_y: int
    square_length_m: float
    marker_length_m: float
    dictionary_name: str
    marker_id_start: int | None      # None = 아직 확인되지 않음
    note: str = ""

    @property
    def marker_count(self) -> int:
        return (self.squares_x * self.squares_y) // 2

    @property
    def corner_count(self) -> int:
        """체커 내부 코너 수. ChArUco pose 를 지지하는 점의 총 개수."""
        return (self.squares_x - 1) * (self.squares_y - 1)

    @property
    def size_mm(self) -> tuple[float, float]:
        return (self.squares_x * self.square_length_m * 1000.0,
                self.squares_y * self.square_length_m * 1000.0)

    def id_range(self) -> tuple[int, int] | None:
        if self.marker_id_start is None:
            return None
        return (self.marker_id_start,
                self.marker_id_start + self.marker_count - 1)

    def describe(self) -> str:
        width, height = self.size_mm
        span = self.id_range()
        ids = f"ID {span[0]}~{span[1]}" if span else "ID 미확인"
        return (f"{self.name}: {self.squares_x}x{self.squares_y} "
                f"square={self.square_length_m*1000:.0f}mm "
                f"marker={self.marker_length_m*1000:.0f}mm "
                f"{self.dictionary_name} {ids} "
                f"| {width:.0f}x{height:.0f}mm, 코너 {self.corner_count}개")

    def to_dict(self) -> dict:
        payload = asdict(self)
        payload["marker_count"] = self.marker_count
        payload["corner_count"] = self.corner_count
        payload["size_mm"] = list(self.size_mm)
        return payload


# 로봇 팔 끝에 고정된 보드. 실험실 표기 '5*7' = 세로 5 x 가로 7.
#
# 현재 marker_id_start=0 은 보드 제작 기본값에 근거한 추정이다. 인쇄에 쓴 PDF 에 ID가
# 적혀 있지 않아, Shah 촬영 전에 실물에서 반드시 다시 읽어야 한다 —
#   python capture/check_board_ids.py --live
# 확인 결과가 다르면 이 값을 고치고 새 세션으로 시작한다.
ROBOT_BOARD = BoardConfig(
    name="robot_board",
    squares_x=7,
    squares_y=5,
    square_length_m=0.017,
    marker_length_m=0.012,
    dictionary_name="DICT_4X4_250",
    marker_id_start=0,
    note="로봇 팔 끝(플랜지)에 볼트 고정. Shah 의 X = T_gripper_board 대상. "
         "ID 0은 추정값 — Shah 촬영 전 check_board_ids.py 로 실물 확인 필요.",
)

# 광학 테이블 위 정지 보드. 규격 출처는 두 곳에서 교차 확인됨:
#   1. jiwoo/rb-calibration-marker-experiment/config.py CharucoBoardConfig
#   2. 같은 저장소 intrinsics/charuco_intrinsics_report.json
# 그리고 cam0 라이브 프레임에서 마커 ID 6~42 를 실측해 일치를 확인했다.
TABLE_BOARD = BoardConfig(
    name="table_board",
    squares_x=11,
    squares_y=7,
    square_length_m=0.025,
    marker_length_m=0.018,
    dictionary_name="DICT_4X4_250",
    marker_id_start=5,
    note="테이블 고정. 고정 카메라 상호등록(camera-to-camera)의 독립 기준. "
         "Shah 입력이 아니라 검증용.",
)

BOARDS = {board.name: board for board in (ROBOT_BOARD, TABLE_BOARD)}


def id_overlap(first: BoardConfig, second: BoardConfig) -> list[int]:
    """두 보드가 공유하는 마커 ID. 같은 딕셔너리일 때만 의미가 있다."""
    if first.dictionary_name != second.dictionary_name:
        return []
    a, b = first.id_range(), second.id_range()
    if a is None or b is None:
        return []
    return sorted(set(range(a[0], a[1] + 1)) & set(range(b[0], b[1] + 1)))


if __name__ == "__main__":
    for board in BOARDS.values():
        print(board.describe())
    print()
    if ROBOT_BOARD.marker_id_start is None:
        print("로봇 보드 ID 미확인 — check_board_ids.py --live 로 확인하라.")
        print("참고: 기본값 0 으로 인쇄됐다면 테이블 보드와 5~16 번 12개가 겹친다.")
    else:
        clash = id_overlap(ROBOT_BOARD, TABLE_BOARD)
        print(f"ID 충돌: {len(clash)}개 {clash if clash else '(없음)'}")
