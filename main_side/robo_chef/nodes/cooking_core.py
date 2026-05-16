"""순수 조리 실행 로직 — ROS/DSR 무관, 단위테스트 대상.

run_jobs(order_id, jobs, play_fn, emit_fn, seg_path) :
  jobs 를 item→qty→segment 3중 루프로 재생. 세그먼트마다 emit_fn 으로
  /cooking_status dict 발행. play_fn(seg_path(seg))->bool. 실패 시 ERROR.
"""
from __future__ import annotations


def _status(state, order_id, recipe_id="", item_index=0, item_total=0,
            qty_index=0, qty_total=0, segment_name="", segment_index=0,
            segment_total=0, error_msg=""):
    return {
        "state": state, "order_id": order_id, "recipe_id": recipe_id,
        "item_index": item_index, "item_total": item_total,
        "qty_index": qty_index, "qty_total": qty_total,
        "segment_name": segment_name, "segment_index": segment_index,
        "segment_total": segment_total, "error_msg": error_msg,
    }


def run_jobs(order_id, jobs, *, play_fn, emit_fn, seg_path):
    m = len(jobs)
    for i, job in enumerate(jobs, 1):
        rid = job["recipe_id"]
        segs = job["segments"]
        qn = int(job.get("qty", 1))
        s_total = len(segs)
        for q in range(1, qn + 1):
            for k, seg in enumerate(segs, 1):
                st = _status("EXECUTING", order_id, rid, i, m, q, qn,
                             seg, k, s_total)
                emit_fn(st)
                if not play_fn(seg_path(seg)):
                    err = _status("ERROR", order_id, rid, i, m, q, qn,
                                  seg, k, s_total,
                                  error_msg=f"segment failed: {seg}")
                    emit_fn(err)
                    return err
    done = _status("DONE", order_id)
    emit_fn(done)
    return done
