# Flask API 레퍼런스

> Base URL: `http://<sub1_side_ip>:5000`  
> 모든 요청/응답 Content-Type: `application/json`  
> 코드 위치: `sub1_side/web/backend/app.py`

---

## 레시피 (Recipes)

### `GET /api/recipes`
Firebase `/recipes` 전체 조회.

**응답 (200)**
```json
{
  "RAMEN": { "recipe_name": "라면", "segments": ["pour_water", "..."] },
  "STEAK": { "recipe_name": "스테이크", "segments": ["..."] }
}
```

---

### `GET /api/recipes/<recipe_id>`
특정 레시피 조회.

**응답 (200)**
```json
{ "recipe_id": "RAMEN", "recipe_name": "라면", "segments": ["pour_water", "add_noodles", "stir", "serve"] }
```

**오류 (404)**: 레시피 미존재

---

### `POST /api/recipes`
레시피 생성/업데이트 (Firebase `/recipes/<recipe_id>` upsert).

**요청 Body**
```json
{ "recipe_id": "RAMEN", "recipe_name": "라면", "segments": ["pour_water", "add_noodles"] }
```

**응답 (201)**
```json
{ "ok": true, "recipe_id": "RAMEN" }
```

---

## 주문 (Orders)

### `GET /api/orders`
주문 목록 조회.

**쿼리 파라미터**
| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `limit` | 50 | 최대 반환 건수 |

**응답 (200)**
```json
{
  "ORD_20260531_153022_a1b2c3": {
    "order_id": "ORD_20260531_153022_a1b2c3",
    "status": "delivered",
    "order_time": "2026-05-31T15:30:22.123456Z",
    "items": [{ "recipe_id": "RAMEN", "qty": 1, "name": "라면", "price": 8000, "subtotal": 8000 }],
    "total": 8000
  }
}
```

---

### `GET /api/orders/<order_id>`
특정 주문 상세 조회.

**응답 (200)**
```json
{
  "order_id": "ORD_...",
  "status": "delivered",
  "order_time": "2026-05-31T15:30:22.123456Z",
  "started_at": "...",
  "completed_at": "...",
  "failed_reason": null,
  "items": [...],
  "recipe_data": { "RAMEN": { "segments": [...], "name": "라면" } },
  "total": 8000
}
```

**오류 (404)**: 주문 미존재

---

### `POST /api/orders`
새 주문 생성. 단일 진입점 — 직접 Firebase에 쓰지 말 것.

**요청 Body**
```json
{
  "items": [
    { "recipe_id": "RAMEN", "qty": 1, "name": "라면", "price": 8000, "subtotal": 8000 }
  ],
  "total": 8000
}
```

**응답 (201)**
```json
{ "ok": true, "order_id": "ORD_20260531_153022_a1b2c3" }
```

**오류 (400)**: `RecipeNotSeededError` — 해당 레시피의 `segments` 배열이 비어 있음.

**처리 흐름**: `firebase_client.create_order()` → Firebase `/orders/<id>` (status=`pending`) → `firebase_bridge` 감지 → 조리 시작.

---

### `PATCH /api/orders/<order_id>/status`
주문 상태 수동 변경 (관리자용).

**요청 Body**
```json
{ "status": "pending | cooking | delivered | failed" }
```

**응답 (200)**
```json
{ "ok": true }
```

---

## 로봇 상태 (Robot)

### `GET /api/robot/state`
Firebase `/robot_state` 에서 최신 상태 반환.

**응답 (200)**
```json
{
  "robot_status": "IDLE",
  "gripper_status": "open",
  "speed_scale": 1.0,
  "mode": "STANDBY",
  "error_code": "NONE",
  "joint_positions": [0.0, 0.0, 90.0, 0.0, 90.0, 0.0],
  "tcp_position": { "x": 0.0, "y": 0.0, "z": 700.0, "rx": 0.0, "ry": 180.0, "rz": 0.0 },
  "last_updated": "2026-05-31T15:00:00Z"
}
```

---

### `GET /api/robot/coords`
ROS 2 `coord_service`에서 현재 관절·TCP 좌표 조회 (실시간).

**응답 (200)**
```json
{
  "joint": [0.0, 0.0, 90.0, 0.0, 90.0, 0.0],
  "tcp": [0.0, 0.0, 700.0, 0.0, 180.0, 0.0],
  "timestamp": "2026-05-31T15:00:00Z",
  "source": "ros2"
}
```

**오류 (503)**: ROS 2 노드 미응답

---

### `POST /api/robot/stop`
로봇 모션 정지 명령 (Firebase `/commands/robot` 에 `stop` 기록 → `SafetyBridge` 실행).

**응답 (200)**: `{ "ok": true }`

---

### `POST /api/robot/home`
로봇 홈 포지션 이동 (비동기 스레드).

**응답 (200)**: `{ "ok": true }`

---

### `POST /api/robot/mode/autonomous`
자율 모드 전환.

**응답 (200)**: `{ "ok": true | false }`

---

## 로그 (Logs)

### `GET /api/logs`
Firebase `/logs` 최신 로그 조회.

**쿼리 파라미터**: `limit` (기본 50)

**응답 (200)**
```json
[
  { "timestamp": "...", "level": "INFO", "message": "Order dispatched", "source": "system" }
]
```

---

### `POST /api/logs`
로그 항목 수동 기록.

**요청 Body** (모두 선택)
```json
{ "level": "INFO", "message": "...", "source": "kiosk" }
```

**로그 레벨**: `INFO` | `WARN` | `ERROR` | `MOVE`

**응답 (201)**: `{ "ok": true }`

---

## 진단 (Diagnostics)

### `GET /api/health`
서비스 헬스 체크.

**응답 (200)**: `{ "status": "ok", "service": "robo-chef-backend" }`

---

### `GET /api/ros2/status`
ROS 2 토픽 활성 상태 확인.

**응답 (200)**
```json
{
  "order_topic": "/recipe",
  "status_topic": "/cooking_status",
  "domain_id": 24,
  "msg_format": "std_msgs/String (JSON payload)",
  "active_topics": ["/recipe", "/cooking_status", "/dsr01/joint_states"]
}
```

---

## 주문 상태 전이

```
pending → cooking → delivered
                 ↘ failed
```

| 상태 | 의미 | 전이 주체 |
|---|---|---|
| `pending` | 주문 생성 완료, 조리 대기 | `app.py` (POST /api/orders) |
| `cooking` | `firebase_bridge` 가 픽업, 조리 진행 중 | `firebase_bridge.py` |
| `delivered` | 조리 완료 (DONE 수신) | `firebase_bridge.py` |
| `failed` | 조리 실패 (ERROR 수신) 또는 orphan 복구 | `firebase_bridge.py` |

---

## 에러 클래스

| 클래스 | 발생 조건 | HTTP 응답 |
|---|---|---|
| `RecipeNotSeededError` | `segments` 배열이 비어 있거나 레시피 미존재 | 400 |
| `OrderError` | 주문 처리 중 내부 오류 | 400 |
