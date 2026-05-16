import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "nodes"))
import order_core as oc


def test_select_next_pending_fifo_by_order_time():
    orders = {
        "B": {"status": "pending", "order_time": "2026-04-26T04:34:10Z"},
        "A": {"status": "pending", "order_time": "2026-04-26T04:34:00Z"},
        "C": {"status": "delivered", "order_time": "2026-04-26T04:33:00Z"},
    }
    oid, order = oc.select_next_pending(orders)
    assert oid == "A"


def test_select_next_pending_none_when_no_pending():
    assert oc.select_next_pending({"A": {"status": "delivered"}}) == (None, None)


def test_build_jobs_expands_items_in_order_with_segments():
    order = {
        "items": [
            {"recipe_id": "RAMEN", "qty": 2},
            {"recipe_id": "STEAK", "qty": 1},
        ],
        "recipe_data": {
            "RAMEN": {"segments": ["r1", "r2"]},
            "STEAK": {"segments": ["s1"]},
        },
    }
    jobs = oc.build_jobs(order)
    assert jobs == [
        {"recipe_id": "RAMEN", "qty": 2, "segments": ["r1", "r2"]},
        {"recipe_id": "STEAK", "qty": 1, "segments": ["s1"]},
    ]


def test_build_jobs_raises_when_segments_missing():
    order = {"items": [{"recipe_id": "X", "qty": 1}], "recipe_data": {"X": {}}}
    try:
        oc.build_jobs(order)
        assert False, "should raise"
    except oc.OrderError as e:
        assert "segments" in str(e)


def test_status_transition_for_cooking_status():
    assert oc.order_transition("DONE") == ("delivered", True)
    assert oc.order_transition("ERROR") == ("failed", False)
    assert oc.order_transition("EXECUTING") == (None, None)
    assert oc.order_transition("IDLE") == (None, True)
