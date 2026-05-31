"""
Firebase Admin SDK 초기화 및 공통 헬퍼
"""
import os
import firebase_admin
from firebase_admin import credentials, db
from datetime import datetime, timezone
import uuid

import config

_initialized = False


def _resolve_cred_path() -> str:
    """FIREBASE_CRED_PATH 환경변수 우선. 없으면 backend/serviceAccountKey.json 사용."""
    env = os.environ.get("FIREBASE_CRED_PATH")
    if env and os.path.isfile(env):
        return env
    return os.path.join(os.path.dirname(__file__), config.SERVICE_ACCOUNT_KEY)


def _resolve_db_url() -> str:
    return os.environ.get("FIREBASE_DB_URL", config.FIREBASE_DATABASE_URL)


def init():
    global _initialized
    if _initialized:
        return
    cred = credentials.Certificate(_resolve_cred_path())
    firebase_admin.initialize_app(cred, {"databaseURL": _resolve_db_url()})
    _initialized = True


def ref(path: str):
    return db.reference(path)


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


# ── Recipes ───────────────────────────────────────────────────────────────────

def get_all_recipes() -> dict:
    return ref("/recipes").get() or {}


def get_recipe(recipe_id: str) -> dict | None:
    return ref(f"/recipes/{recipe_id}").get()


def upsert_recipe(recipe_id: str, data: dict):
    ref(f"/recipes/{recipe_id}").set(data)


# ── Orders ────────────────────────────────────────────────────────────────────

class RecipeNotSeededError(Exception):
    """`/recipes/<id>` 에 segments 배열이 비어있을 때. app.py 는 400 으로 매핑."""


def create_order(items: list, total: int) -> str:
    """주문 생성. 각 item.recipe_id 마다 `/recipes/<id>` 에서 segments 를 lookup 해
    `/orders/<id>.recipe_data` 에 채워 넣는다 — kiosk 는 더 이상 recipe_data 를
    클라이언트에서 조립하지 않는다 (단일 출처: backend).

    segments 가 비어있는 recipe 가 한 건이라도 있으면 RecipeNotSeededError.
    이는 ros2_move_recoder 로 GUI 녹화 후 recipe_seeder 갱신이 필요하다는 신호다.
    """
    if not items:
        raise ValueError("items 가 비어있음")
    recipe_ids = []
    recipe_data = {}
    for it in items:
        rid = (it or {}).get("recipe_id")
        if not rid:
            raise ValueError(f"item 에 recipe_id 누락: {it}")
        if rid in recipe_data:
            continue
        rdoc = get_recipe(rid)
        if rdoc is None:
            raise ValueError(f"미등록 recipe_id={rid} (Firebase /recipes 에 없음)")
        segs = (rdoc.get("segments") or [])
        if not segs:
            raise RecipeNotSeededError(
                f"recipe_id={rid} 의 segments 미시드 — "
                "ros2_move_recoder GUI 로 녹화 후 recipe_seeder 갱신 필요")
        recipe_data[rid] = {"segments": list(segs), "name": rdoc.get("recipe_name", rid)}
        recipe_ids.append(rid)
    order_id = f"ORD_{datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')}_{uuid.uuid4().hex[:6]}"
    primary_rid = recipe_ids[0]
    ref(f"/orders/{order_id}").set({
        "order_id":    order_id,
        "recipe_id":   primary_rid,                 # 호환: 단일 대표 recipe
        "items":       items,
        "recipe_data": recipe_data,                 # 메인 노드 build_jobs 가 사용
        "total":       total,
        "status":      "pending",
        "order_time":  now_iso(),
        "started_at":     None,
        "completed_at":   None,
        "failed_reason":  None,
    })
    return order_id


def prune_orders(keep: int = 10):
    """가장 오래된 주문부터 삭제, 최신 `keep` 건만 유지. order_id 가 timestamp
    prefix 라 키 정렬 = 시간 정렬. kiosk 의 직접 prune 로직을 대체한다."""
    all_orders = ref("/orders").get() or {}
    keys = sorted(all_orders.keys())
    for k in keys[:-keep] if len(keys) > keep else []:
        ref(f"/orders/{k}").delete()


def update_order_status(order_id: str, status: str, **kwargs):
    data = {"status": status, **kwargs}
    ref(f"/orders/{order_id}").update(data)


def get_orders(limit: int = 50) -> dict:
    # order_by_child 는 Firebase 인덱스 필요 → 전체 조회 후 Python 정렬
    all_orders = ref("/orders").get() or {}
    sorted_orders = dict(
        sorted(all_orders.items(),
               key=lambda x: x[1].get("order_time", "") if isinstance(x[1], dict) else "")
    )
    items = list(sorted_orders.items())
    return dict(items[-limit:]) if len(items) > limit else sorted_orders


def get_order(order_id: str) -> dict | None:
    return ref(f"/orders/{order_id}").get()


# ── Robot State ───────────────────────────────────────────────────────────────

def update_robot_state(state: dict):
    ref("/robot_state").update({**state, "last_updated": now_iso()})


def get_robot_state() -> dict:
    return ref("/robot_state").get() or {}


# ── Logs ──────────────────────────────────────────────────────────────────────

def push_log(level: str, message: str, source: str = "system"):
    log_id = f"LOG_{uuid.uuid4().hex[:12]}"
    ref(f"/logs/{log_id}").set({
        "timestamp": now_iso(),
        "level":     level,
        "message":   message,
        "source":    source,
    })
    # 최대 MAX_LOG_ENTRIES 유지
    _prune_logs()


def _prune_logs():
    logs = ref("/logs").order_by_key().get()
    if logs and len(logs) > config.MAX_LOG_ENTRIES:
        keys = sorted(logs.keys())
        for old_key in keys[: len(logs) - config.MAX_LOG_ENTRIES]:
            ref(f"/logs/{old_key}").delete()


def get_recent_logs(limit: int = 50) -> list:
    logs = ref("/logs").order_by_key().limit_to_last(limit).get() or {}
    return sorted(logs.values(), key=lambda x: x.get("timestamp", ""))
