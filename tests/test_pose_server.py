"""Python 2 pose server + real recorder over TCP, with no robot hardware.

The fake GELLO endpoint occupies its production port. Shared memory and motion
SDK modules are replaced only in the test subprocess, never in deployed code.
"""
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import socketserver
import subprocess
import sys
import threading
import time

import pytest

from capture import record_dataset

ROOT = Path(__file__).resolve().parents[1]
PYTHON2 = shutil.which("python2.7")
POSE = ROOT / "capture/robot/pose_server.py"


class GelloHandler(socketserver.StreamRequestHandler):
    def handle(self):
        for line in self.rfile:
            request = json.loads(line)
            self.server.requests.append(request)
            op = request.get("op")
            self.wfile.write((json.dumps({"ok": bool(op), "err": "unknown op"}) + "\n").encode())
            if op == "bye":
                return


class GelloServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


@pytest.fixture
def servers(tmp_path):
    if not PYTHON2:
        pytest.skip("Python 2.7 is required")
    # Bind the real defaults: the old pose server would collide with GELLO here.
    gello = GelloServer(("127.0.0.1", 12350), GelloHandler)
    gello.requests = []
    thread = threading.Thread(target=gello.serve_forever, daemon=True)
    thread.start()
    (tmp_path / "i611shm.py").write_text(
        "def shm_read(reg, count):\n"
        "    assert count == 6\n"
        "    if reg == 0x3000:\n"
        "        return '0.4,0.1,0.3,0,0,0'\n"
        "    assert reg == 0x3050\n"
        "    return '0,0,0,0,0,0'\n"
    )
    for module in ("i611_MCS", "i611_extend", "rbsys", "i611_common", "i611_io"):
        (tmp_path / (module + ".py")).write_text(
            "raise AssertionError('Pose server must not import robot control SDK')\n"
        )
    env = dict(os.environ, PYTHONPATH=str(tmp_path), PYTHONDONTWRITEBYTECODE="1")
    log = (tmp_path / "pose.log").open("w")
    proc = None
    try:
        proc = subprocess.Popen([PYTHON2, "-u", str(POSE), "--host", "127.0.0.1"],
                                env=env, stdout=log, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                pytest.fail((tmp_path / "pose.log").read_text())
            try:
                with socket.create_connection(("127.0.0.1", 12352), timeout=0.1):
                    break
            except OSError:
                time.sleep(0.03)
        else:
            pytest.fail("pose server startup timeout")
        yield gello, proc
    finally:
        if proc is not None and proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
        log.close()
        gello.shutdown()
        gello.server_close()
        thread.join(timeout=2)


def test_capture_and_reconnect_do_not_touch_gello(servers, tmp_path):
    gello, proc = servers
    with socket.create_connection(gello.server_address, timeout=2) as motion:
        stream = motion.makefile("rb")
        def control(op):
            motion.sendall((json.dumps({"op": op}) + "\n").encode())
            assert json.loads(stream.readline())["ok"]
        control("stream_start")
        for _ in range(3):
            control("movej")  # Mock only: no actual motion SDK is loaded.
            robot = record_dataset.RobotLink("127.0.0.1", record_dataset.DEFAULT_ROBOT_PORT, 2)
            try:
                assert robot.request({"command": "ping"})["read_only"] is True
                state = robot.get_state()
                assert state["flange_pose_6dof"] == [400, 100, 300, 0, 0, 0]
                assert state["joints_6dof"] == [0] * 6
                assert state["source"] == "shm"
            finally:
                robot.close()
        # Run the actual CLI: Enter writes a pose record; q closes only the reader.
        result = subprocess.run(
            [sys.executable, str(ROOT / "capture/record_dataset.py"),
             "--no-camera", "--robot-host", "127.0.0.1",
             "--dataset-root", str(tmp_path / "data"), "--session", "test"],
            input="\nq\n", text=True, capture_output=True, timeout=15,
        )
        assert result.returncode == 0, result.stdout + result.stderr
        meta = json.loads((tmp_path / "data/shah/test/meta.json").read_text())
        assert len(meta["captures"]) == 1
        assert meta["captures"][0]["robot_pose_6dof"] == [400, 100, 300, 0, 0, 0]
        control("ping")
        stream.close()
    assert proc.poll() is None
    assert [r["op"] for r in gello.requests] == ["stream_start"] + ["movej"] * 3 + ["ping"]


def test_check_robot_creates_no_dataset(servers, tmp_path):
    root = tmp_path / "must_not_exist"
    assert record_dataset.main(["--check-robot", "--robot-host", "127.0.0.1",
                                "--dataset-root", str(root)]) == 0
    assert not root.exists()
    assert not servers[0].requests


def test_pose_server_rejects_motion_and_survives_bad_requests(servers):
    robot = record_dataset.RobotLink("127.0.0.1", 12352, 2)
    try:
        for command in ("jog", "speed", "settool", "changetool", "movel", "movej",
                        "grip", "stop", "stream_start", "stream_stop"):
            assert robot.send({"command": command})["status"] == "error"
        wrong = robot.send({"op": "stream_start"})
        assert wrong["ok"] is False
        assert "12350" in wrong["err"]
        assert robot.send([])["status"] == "error"
        robot.sock.sendall(b"not json\n")
        assert json.loads(robot.sock.recv(8192))["status"] == "error"
        assert robot.get_state()["source"] == "shm"
    finally:
        robot.close()
    assert not servers[0].requests


def test_recorder_rejects_gello_before_connecting(monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail("Recorder connected to GELLO's control port")
    monkeypatch.setattr(socket, "create_connection", forbidden)
    with pytest.raises(ValueError, match="12350"):
        record_dataset.RobotLink("127.0.0.1", 12350)
    with pytest.raises(SystemExit) as error:
        record_dataset.parse_args(["--robot-port", "12350"])
    assert error.value.code == 2


@pytest.mark.skipif(not PYTHON2, reason="Python 2.7 is required")
def test_both_server_names_reject_gello_port_without_loading_sdk():
    for filename in ("pose_server.py", "pose_query_server.py"):
        result = subprocess.run([PYTHON2, "-B", str(POSE.with_name(filename)),
                                 "--port", "12350"], capture_output=True, text=True)
        assert result.returncode == 2
        assert "reserved for zeus_gello.py" in result.stderr
