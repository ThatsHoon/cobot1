import json, os, tempfile
import ros2_move_recoder.playback as pb


def _write_smooth(d):
    path = os.path.join(tempfile.mkdtemp(), "smooth.json")
    with open(path, "w") as f:
        json.dump(d, f)
    return path


def test_missing_file_returns_failure():
    res = pb.play_segment("/no/such/smooth.json")
    assert res.ok is False
    assert "missing" in (res.error or "").lower()


def test_not_autonomous_returns_failure(monkeypatch):
    path = _write_smooth({"waypoints_deg": [[0, 0, 90, 0, 90, 0]], "vel": 30, "acc": 60})
    monkeypatch.setattr(pb, "_dsr", lambda: pb._DSR(
        amovesj=lambda *a, **k: 0, check_motion=lambda: False,
        posj=lambda *w: list(w), get_robot_mode=lambda: 0,
        ROBOT_MODE_AUTONOMOUS=1, sstop=lambda: None))
    res = pb.play_segment(path, require_autonomous=True)
    assert res.ok is False
    assert "autonomous" in (res.error or "").lower()
