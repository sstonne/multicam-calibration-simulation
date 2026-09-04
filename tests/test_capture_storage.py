from pathlib import Path

from capture import record_dataset


ROOT = Path(__file__).resolve().parents[1]


def test_default_dataset_root_is_repository_archive_path():
    args = record_dataset.parse_args([])

    assert Path(args.dataset_root) == ROOT / "datasets"


def test_dataset_index_is_updated_for_a_new_session(tmp_path):
    algorithm_dir = tmp_path / "shah"
    session_dir = algorithm_dir / "session_test"
    algorithm_dir.mkdir()
    meta = {
        "captures": [],
        "dataset": {
            "algorithm": "shah",
            "capture_mode": "pose_and_image",
            "created_at": "2026-09-04T00:00:00+09:00",
            "updated_at": "2026-09-04T00:00:01+09:00",
        },
    }

    record_dataset.update_dataset_index(algorithm_dir, session_dir, meta)

    index_path = algorithm_dir / "dataset_index.json"
    assert index_path.exists()
    assert index_path.read_text(encoding="utf-8").find("session_test") >= 0
