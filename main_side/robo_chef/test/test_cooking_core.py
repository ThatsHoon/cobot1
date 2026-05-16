import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "nodes"))
import cooking_core as cc


def test_run_jobs_emits_status_sequence_and_completes():
    jobs = [
        {"recipe_id": "RAMEN", "qty": 2, "segments": ["r1"]},
        {"recipe_id": "STEAK", "qty": 1, "segments": ["s1", "s2"]},
    ]
    emitted = []
    played = []

    def play(seg):
        played.append(seg)
        return True  # 성공

    final = cc.run_jobs("ORD1", jobs, play_fn=play,
                         emit_fn=lambda s: emitted.append(s),
                         seg_path=lambda s: s)
    # 재생 호출: r1,r1, s1,s2  (qty/segment 전개)
    assert played == ["r1", "r1", "s1", "s2"]
    # 마지막 emit 은 DONE
    assert emitted[-1]["state"] == "DONE"
    assert final["state"] == "DONE"
    # EXECUTING 한 건 스키마 확인
    ex = next(s for s in emitted if s["state"] == "EXECUTING")
    assert set(ex) == {"state", "order_id", "recipe_id", "item_index",
                       "item_total", "qty_index", "qty_total",
                       "segment_name", "segment_index", "segment_total",
                       "error_msg"}
    assert ex["order_id"] == "ORD1"


def test_run_jobs_stops_on_failure_with_error_state():
    jobs = [{"recipe_id": "RAMEN", "qty": 1, "segments": ["r1", "r2"]}]
    emitted = []

    def play(seg):
        return seg != "r2"   # r2 실패

    final = cc.run_jobs("ORD2", jobs, play_fn=play,
                         emit_fn=lambda s: emitted.append(s),
                         seg_path=lambda s: s)
    assert final["state"] == "ERROR"
    assert final["segment_name"] == "r2"
    assert emitted[-1]["state"] == "ERROR"
    assert "r2" in emitted[-1]["error_msg"]
