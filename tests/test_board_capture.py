from dataclasses import replace
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from capture import record_dataset
from capture.board_config import ROBOT_BOARD, TABLE_BOARD, id_overlap
from capture.check_board_ids import charuco_corner_count
from capture.shah_capture_client import BoardDetector


def printed_board_image():
    """Known 9x6 print layout, independent of the production board constructor."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    board = cv2.aruco.CharucoBoard((9, 6), 0.025, 0.018, dictionary,
                                 np.arange(90, 117, dtype=np.int32).reshape(-1, 1))
    board.setLegacyPattern(True)
    gray = board.generateImage((940, 640), marginSize=20)
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


def test_recorder_detects_new_printed_board_and_excludes_table_ids():
    image = printed_board_image()
    detector = BoardDetector(ROBOT_BOARD, ROBOT_BOARD.marker_id_start)
    K = np.array([[900.0, 0, 470], [0, 900.0, 320], [0, 0, 1]])
    result = detector.detect(image, K, np.zeros(5))
    assert result['ok']
    assert result['n_corners'] == 40
    assert result['marker_ids'] == list(range(90, 117))
    assert result['reproj_error_px'] < 0.5
    assert id_overlap(ROBOT_BOARD, TABLE_BOARD) == []
    # Chessboard object coordinates must use the confirmed 25 mm metric scale.
    assert np.allclose(detector.grid.getChessboardCorners()[0], [0.025, 0.025, 0])


def test_even_row_pattern_is_required_by_board_check_and_recorder():
    image = printed_board_image()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    assert charuco_corner_count(gray, ROBOT_BOARD, 90) == 40
    assert charuco_corner_count(gray, ROBOT_BOARD, 90, legacy_pattern=False) == 0
    wrong = BoardDetector(replace(ROBOT_BOARD, legacy_pattern=False), 90)
    corners, _, _, _ = wrong.detector.detectBoard(gray)
    assert corners is None


def resume_inputs():
    return (
        {
            'capture_config': {'save_depth': True, 'width': 1280, 'height': 720},
            'cam_indices': [0, 1, 3],
            'board_config': ROBOT_BOARD.to_dict(),
            'dataset': {'capture_mode': 'pose_and_image'},
        },
        SimpleNamespace(save_depth=True, width=1280, height=720),
        [SimpleNamespace(index=n) for n in (0, 1, 3)],
        BoardDetector(ROBOT_BOARD, 90),
    )


@pytest.mark.parametrize(('field', 'old'), [
    ('squares_x', 7), ('squares_y', 5), ('square_length_m', 0.017),
    ('marker_length_m', 0.012), ('dictionary_name', 'DICT_4X4_1000'),
    ('legacy_pattern', False), ('marker_id_start', 0),
])
def test_cannot_resume_data_from_a_different_board(field, old):
    meta, args, cameras, detector = resume_inputs()
    meta['board_config'][field] = old
    with pytest.raises(SystemExit) as error:
        record_dataset.check_resume_compatible(meta, args, cameras, detector)
    assert error.value.code == 2


def test_same_board_session_can_resume():
    record_dataset.check_resume_compatible(*resume_inputs())
