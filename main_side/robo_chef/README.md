# robo_chef

ROS 2 Humble (`ament_python`) 패키지. Firebase RTDB 에서 주문을 수신하고, 사전 녹화된 모션 세그먼트를 체인 재생해 Doosan m0609 협동로봇으로 조리를 수행한다.

---

## 노드 구성

| 노드명 | 파일 | 역할 |
|---|---|---|
| `firebase_bridge` | `nodes/firebase_bridge.py` | Firebase `/orders` FIFO 감시 → `/recipe` 발행. `/cooking_status` 수신 → 주문 상태 갱신. |
| `sequence_runner` | `nodes/sequence_runner.py` | `/recipe` 수신 → `cooking_core.run_jobs()` 실행 → 세그먼트 재생. DSR 초기화 포함. |

### 순수 로직 모듈 (`nodes/`)

| 모듈 | 역할 |
|---|---|
| `cooking_core.run_jobs()` | item → qty → segment 3중 루프 재생 오케스트레이터. ROS/DSR 무관, 단위테스트 가능. |
| `order_core` | `select_next_pending()` (FIFO), `build_jobs()`, `order_transition()`. |

---

## 인터페이스

### 토픽

| 이름 | 타입 | 방향 | 설명 |
|---|---|---|---|
| `/recipe` | `std_msgs/String` | publish (firebase_bridge) | 주문 JSON: `{order_id, jobs:[{recipe_id, qty, segments:[]}]}` |
| `/recipe` | `std_msgs/String` | subscribe (sequence_runner) | 위 동일 |
| `/cooking_status` | `std_msgs/String` | publish (sequence_runner) | 진행 JSON: `{state, order_id, recipe_id, item_index, item_total, qty_index, qty_total, segment_name, segment_index, segment_total, error_msg}` |
| `/cooking_status` | `std_msgs/String` | subscribe (firebase_bridge) | 위 동일 → Firebase `/robot_status` 미러 |

### 서비스

| 이름 | 타입 | 서버 | 설명 |
|---|---|---|---|
| `unlock_system` | `std_srvs/Trigger` | `sequence_runner` | ERROR 상태 해제 → IDLE 복귀 |

---

## 상태 머신 (sequence_runner)

```
IDLE ──[/recipe 수신 + pre-flight OK]──▶ EXECUTING
  ▲                                           │
  │  [DONE]                              [ERROR]
  └───────────────────────────────────────────┤
         [unlock_system 호출]◀──────── ERROR
```

| 상태 | 진입 조건 | 탈출 조건 |
|---|---|---|
| `IDLE` | 초기·정상 완료·unlock | `/recipe` 수신 + pre-flight 통과 |
| `EXECUTING` | pre-flight 통과 | 조리 완료(DONE) 또는 실패(ERROR) |
| `ERROR` | 세그먼트 실패·예외 | 관리자가 `unlock_system` 서비스 호출 |

**Pre-flight 검사**: `/recipe` 수신 직후 모든 세그먼트 `smooth.json` 존재 여부를 확인한다. 파일 누락 시 즉시 ERROR emit (중간 실패로 인한 비상정지 회피).

---

## 주요 설계 결정

### FIFO 주문 처리
`order_time` 기준 오름차순 정렬 후 첫 번째 `pending` 주문을 선택한다. (`order_core.select_next_pending()`)

### busy 플래그
1건 조리 진행 중에는 다음 주문을 디스패치하지 않는다. 조리 완료(DONE) 또는 unlock 시 해제된다.

### Orphan Order 복구
노드 재기동 시 `status == "cooking"` 인 주문을 자동으로 `failed` 로 전환한다. (`firebase_bridge._recover_orphans()`) 무한 잠금을 방지한다.

### Firebase Credentials 탐색 순서
1. `$FIREBASE_CRED_PATH` 환경변수
2. `~/.config/cobot1/firebase-key.json`
3. `~/cobot_ws/src/cobot1/main_side/robo_chef/config/serviceAccountKey.json`

---

## 실행

```bash
source ~/cobot_ws/install/setup.bash

ros2 run robo_chef firebase_bridge
# 별도 터미널
ros2 run robo_chef sequence_runner
```

`firebase_bridge` 를 먼저 기동해야 `/recipe` 토픽 구독이 연결된다.

---

## 테스트

```bash
cd ~/cobot_ws
colcon test --packages-select robo_chef
colcon test-result --verbose
```

- `test/test_cooking_core.py` — `run_jobs()` 3중 루프 단위 테스트
- `test/test_order_core.py` — FIFO 선택·상태 전이 단위 테스트

---

## 의존성

| 패키지 | 용도 |
|---|---|
| `firebase-admin` | Firebase RTDB 읽기/쓰기 |
| `ros2_move_recoder` (공용) | `playback.play_segment()` — 세그먼트 재생 엔진 |
| DSR_ROBOT2 | Doosan m0609 모션 API (`amovesj`, `check_motion` 등) |
| `std_msgs`, `std_srvs` | ROS 2 메시지 타입 |

전체 세팅은 [`docs/setup.md`](../../docs/setup.md) 참고.
