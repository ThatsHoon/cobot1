import json, os, tempfile, time
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


class FakeGripper:
    def __init__(self):
        self.calls = []
        self._w = 100.0
    def open(self):  self.calls.append(("open", time.monotonic()))
    def close(self): self.calls.append(("close", time.monotonic()))
    def move(self, w): self.calls.append(("move", w))
    def last_state(self): return "open"
    def width_mm(self): return self._w


def test_gripper_events_fire_in_order(monkeypatch):
    sm = {"waypoints_deg": [[0]*6, [10]*6], "vel": 30, "acc": 60,
          "record_duration_sec": 2.0,
          "gripper_events": [
              {"t_norm": 0.0, "kind": "open", "width_mm": 100},
              {"t_norm": 0.9, "kind": "close", "width_mm": 20}]}
    path = _write_smooth(sm)
    motion = {"n": 0}
    def fake_check():
        motion["n"] += 1
        return motion["n"] < 6   # ~0.3s 모션
    monkeypatch.setattr(pb, "_dsr", lambda: pb._DSR(
        amovesj=lambda *a, **k: 0, check_motion=fake_check,
        posj=lambda *w: list(w), get_robot_mode=lambda: 1,
        ROBOT_MODE_AUTONOMOUS=1, sstop=lambda: None))
    g = FakeGripper()
    res = pb.play_segment(path, gripper=g, require_autonomous=False)
    assert res.ok is True
    kinds = [c[0] for c in g.calls if c[0] in ("open", "close")]
    assert kinds == ["open", "close"]   # 순서 보존
