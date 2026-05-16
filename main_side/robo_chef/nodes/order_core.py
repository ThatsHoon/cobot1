"""순수 주문 로직 — ROS/Firebase 무관, 단위테스트 대상.

select_next_pending : FIFO(order_time) 로 다음 pending 주문 선택
build_jobs          : order.items[] 를 jobs 로 전개 (recipe_data.segments 결합)
order_transition    : /cooking_status.state → (/orders status, busy 해제 여부)
"""
from __future__ import annotations


class OrderError(Exception):
    pass


def select_next_pending(orders: dict):
    pend = [(oid, o) for oid, o in (orders or {}).items()
            if isinstance(o, dict) and o.get("status") == "pending"]
    if not pend:
        return (None, None)
    pend.sort(key=lambda x: x[1].get("order_time", ""))
    return pend[0]


def build_jobs(order: dict) -> list:
    items = order.get("items") or []
    rdata = order.get("recipe_data") or {}
    jobs = []
    for it in items:
        rid = it.get("recipe_id")
        qty = int(it.get("qty", 1))
        segs = ((rdata.get(rid) or {}).get("segments")) or []
        if not segs:
            raise OrderError(f"no segments for recipe_id={rid}")
        jobs.append({"recipe_id": rid, "qty": qty, "segments": list(segs)})
    if not jobs:
        raise OrderError("order has no items")
    return jobs


def order_transition(state: str):
    """반환: (orders status 새 값 | None, busy 해제 | None)
    None busy = '변화 없음'. True = 해제(다음 pending 진행). False = 유지."""
    if state == "DONE":
        return ("delivered", True)
    if state == "ERROR":
        return ("failed", False)   # 주문은 failed 종결, 그러나 런너 ERROR → busy 유지
    if state == "IDLE":
        return (None, True)        # unlock 재개 신호
    return (None, None)            # EXECUTING 등
