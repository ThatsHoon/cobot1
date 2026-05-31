# cobot1 — 라면/스테이크 조리 로봇 아키텍처

> 현재 코드(2026-05-24, `fixed-sequence-playback` 모델) 기준 아키텍처 문서.
> 이전(동적 액션 엔진 + ActionManager) 설계와는 호환되지 않으므로 옛 문서를
> 참고 중이라면 이 파일을 우선한다.

---

## 1. 한눈에 보는 전체 그림

손님이 **키오스크** 에서 주문 → **Firebase RTDB `/orders`** 적재 → 메인 PC 의
**`firebase_bridge`** 가 FIFO 로 감지 → `/recipe` 잡 발행 → **`sequence_runner`**
가 사전 녹화된 세그먼트 체인을 차례로 재생 (`ros2_move_recoder.playback`) →
진행 상태가 `/cooking_status` 로 다시 `firebase_bridge` 에 → Firebase
`/robot_status` 갱신 → **고객 현황판/키오스크** 가 표시. 하드웨어 텔레메트리는
**`coord_service`** 가 별도로 Firebase `/robot_state` 에 기록 → **관리자
모니터** 가 표시.

```
[키오스크] ──orders.set──▶ Firebase /orders ──listen──▶ [firebase_bridge] ──/recipe──▶ [sequence_runner]
   ▲                                                              │                          │
   │                                                              │                          │ play_segment(*)
   │                                                              │                          ▼
   │         Firebase /robot_status ◀──set──── /cooking_status ◀──┘                  m0609 (DSR_ROBOT2)
   │         [customer_status / kiosk]                                                       ▲
   │                                                                                         │ /dsr01/joint_states
   │         Firebase /robot_state ◀── coord_service + fk_worker                             │
   │         [admin_monitor]                                                            dsr_controller2
   │
   └── Firebase /recipes ── (kiosk 가 메뉴 로딩)
```

물리적으로 **3개의 위치(side)** 로 코드가 나뉜다.

| side | 위치/역할 | 핵심 패키지 |
|---|---|---|
| `main_side` | 로봇 팔이 붙은 메인 PC. 조리 실행 + 텔레메트리 | `robo_chef`, `realtime_scan` |
| `sub1_side` | 키오스크·대시보드·Firebase 브리지 PC | `web` (Flask backend + 정적 패널) |
| `common` | 양쪽 공용. 동작 녹화/재생 도구이자 재생 엔진 | `ros2_move_recoder` |

세 사이드 모두 **`ROS_DOMAIN_ID=24`** 로 통일. `~/.bashrc` 가 다른 값을 export
하더라도 각 진입점(`start_all.sh`, `run_coord_service.sh`, `run_receiver.sh`,
`app.py startup()`) 가 명시적으로 24 를 강제한다. main↔sub 간 ROS2 토픽 통신은
이 일치가 전제다.

---

## 2. `robo_chef` — 조리 엔진 (메인 PC)

레시피 잡을 받아 **사전 녹화된 세그먼트 체인** 으로 재생하는 ROS 2 패키지.
**노드 2개 + 순수 로직 모듈 2개**. 동적 action verb 플러그인은 더 이상 없다.

### 2.1 노드와 데이터 흐름

```
Firebase /orders ──listen──▶ firebase_bridge ──/recipe──▶ sequence_runner
                                  ▲                              │
                                  └──/cooking_status (state)─────┘
```

| 노드 | 노드명 | 담당 기능 | 주요 인터페이스 |
|---|---|---|---|
| `firebase_bridge` | `firebase_bridge` | Firebase `/orders` FIFO(order_time) 감시 → busy 가 아닐 때 다음 pending 1건을 잡으로 전개·디스패치. `/cooking_status` 수신 시 주문 status 전이(`cooking → delivered/failed`) 및 `/robot_status` 미러. | pub `/recipe`(String), sub `/cooking_status`(String), Firebase RTDB |
| `sequence_runner` | `sequence_runner` | `/recipe` 수신 시 IDLE→EXECUTING 전이, `cooking_core.run_jobs` 로 item→qty→segment 3중 루프 재생. 실패 시 ERROR. `unlock_system` Trigger 로만 IDLE 복귀. DSR 소유 노드(dsr_helper_node 도 함께 spin). | sub `/recipe`, pub `/cooking_status`, srv `unlock_system`(Trigger), DSR_ROBOT2(m0609) |

### 2.2 순수 로직 모듈 (`nodes/*_core.py`)

ROS/DSR 무관. 단위테스트 대상.

- **`cooking_core.run_jobs`**: jobs 를 item→qty→segment 3중 루프로 재생.
  각 세그먼트마다 `emit_fn` 으로 `/cooking_status` dict 발행 → `play_fn(seg_path(seg))`
  호출 → False 면 ERROR dict emit 후 종료. 정상 종료 시 `DONE` dict.
- **`order_core`**: `select_next_pending`(FIFO), `build_jobs`(items[]→jobs),
  `order_transition`(state→orders status + busy 해제 신호).

### 2.3 재생 엔진 — `ros2_move_recoder.playback.play_segment`

`smooth.json` 파일을 받아 `amovesj` 비동기 모션 + 그리퍼 이벤트 타임라인 재생.
PyQt/입력 의존 없음(헤드리스). `abort_event` 로 외부에서 즉시 중단 가능.

세그먼트 경로 규약: `${RECORDS_DIR}/<seg>/smooth.json`, 기본
`~/cobot_ws/src/ros2_move_recoder/records`.

### 2.4 잡 메시지 스키마

`/recipe` 페이로드:
```json
{
  "order_id": "ORD_...",
  "jobs": [{"recipe_id": "RAMEN", "qty": 2, "segments": ["pour_water", "...", "..."]}]
}
```

`/cooking_status` dict:
```json
{
  "state": "EXECUTING|ERROR|DONE|IDLE",
  "order_id": "...", "recipe_id": "...",
  "item_index": 0, "item_total": 0,
  "qty_index": 0,  "qty_total": 0,
  "segment_name": "...", "segment_index": 0, "segment_total": 0,
  "error_msg": ""
}
```

### 2.5 상태/복구

- IDLE → EXECUTING → (정상 종료) IDLE / (실패) ERROR.
- ERROR 에서는 `unlock_system` Trigger 만 IDLE 복귀 가능. 자동 재시도는 없다.
- `_lock` 이 IDLE→EXECUTING 전이를 원자화. `_abort` Event 가 재생 중 외부 중단.

---

## 3. `realtime_scan` — 텔레메트리 & 안전 브리지 (메인 PC)

`DSR_ROBOT2` 를 import 하지 않고 ROS 토픽/서비스만 사용(`sequence_runner` 와의
DSR 노드 충돌 회피). 노드 3개.

| 노드 | 노드명 | 담당 기능 | 주요 인터페이스 |
|---|---|---|---|
| `coord_service` | `robo_chef_coord_service` | `/dsr01/joint_states` 캐시 → `/robo_chef/coords` 2Hz pub + Firebase `telemetry/robot_status`(5Hz raw) + `robot_state`(2Hz merge, joint_positions/last_updated). `SafetyBridge` 가 Firebase `commands/robot` listen → `dsr_msgs2` 서비스 호출 → `commands/robot_ack` 회신. | sub `/dsr01/joint_states`, pub `/robo_chef/coords`, srv `/robo_chef/get_coords`, dsr_msgs2 클라이언트, Firebase 다수 |
| `order_receiver` | `robo_chef_order_receiver` | (legacy 진단 도구) `/robo_chef/order_request` 구독해 콘솔 출력. **현재 발행자 없음** — `firebase_bridge` 가 `/orders` 를 직접 listen 하므로 운영 흐름과 분리됨. | sub `/robo_chef/order_request` |
| `log_bridge` | `dsr_log_bridge` | 모든 `/rosout`(INFO+) 을 0.5s 배치로 Firebase `dsr_log` 에 push. 300개 초과 시 trim. | sub `/rosout`(rcl_interfaces/Log), Firebase `dsr_log` |

안전명령 흐름: 웹 관리자 → Firebase `/commands/robot` → coord_service SafetyBridge
→ `/dsr01/motion/*`, `/dsr01/system/*` 서비스 → `/commands/robot_ack` 회신.
타임아웃: 서비스 연결 3s, 호출 6s.

---

## 4. `web` (sub1_side) — 키오스크 / 대시보드 / Firebase 브리지

Flask 백엔드 + 정적 HTML 패널 3종. Firebase RTDB(`robochef-5d9b6`)가 중앙
컨트롤 플레인.

### 4.1 백엔드 모듈

| 모듈 | 담당 기능 |
|---|---|
| `app.py` | Flask REST. startup() 에서 `ROS_DOMAIN_ID` 강제 + `rclpy.init()` + `coord_client.start()` + `rb.start()` 순서로 진입. 라우터: `/api/recipes`, `/api/orders`, `/api/robot/{state,coords,stop,home,mode/autonomous}`, `/api/logs`, `/api/ros2/status` |
| `config.py` | 상수: Firebase URL, Flask 포트, **ROS2_DOMAIN_ID = 24**, 토픽명 |
| `firebase_client.py` | Firebase Admin SDK 초기화 + `/recipes`, `/orders`, `/robot_state`, `/logs` 헬퍼. `FIREBASE_CRED_PATH`/`FIREBASE_DB_URL` 환경변수 우선 해석 |
| `robot_bridge.py` | **rclpy 네이티브 명령 클라이언트**. 백그라운드 노드 1개 + 미리 생성된 `MoveStop/MoveJoint/SetRobotMode` 클라이언트. 함수: `stop_robot()`, `move_joint(...)`, `set_robot_mode_autonomous()`. 텔레메트리 폴링 루프는 제거 — coord_service + fk_worker 가 담당 |
| `ros2_coord_client.py` | 영속 rclpy 노드. `/robo_chef/coords` 구독 캐시 → `/api/robot/coords` 응답 |
| `fk_worker.py` / `fk_m0609.py` | Firebase `/robot_state/joint_positions` 폴링(2Hz) → m0609 FK → `/robot_state.tcp_position` merge-update |
| `recipe_seeder.py` | 초기 레시피 시드(1회용 standalone) |
| `ros2_pub_once.py` | 단순 토픽 1회 발행 CLI 헬퍼(standalone) |

backend 가 정의하는 Firebase 키:
- `/robot_state` (canonical 하드웨어 텔레메트리) — coord_service + fk_worker + app.py 가 writer.

### 4.2 패널 3종 (정적, http.server)

| 패널 | 포트 | 구독하는 Firebase 키 |
|---|---|---|
| `kiosk` | 3001 | `/recipes`, `/orders`, `/robot_status` (조리 진행 상태로 결제 차단 판단) |
| `customer_status` | 3002 | `/orders`(최근 1건), `/robot_status`(segment 단위 진행 렌더), `/phase`(제어 신호) |
| `admin_monitor` | 3003 | `/robot_state`(하드웨어 텔레메트리), `/logs`, `/dsr_log`, `/commands/robot_ack`. write: `/commands/robot`(안전명령), `/recipes`(편집) |

> ⚠️ `/robot_status` 와 `/robot_state` 는 **별개의 채널**이다. 같은 이름처럼 보이지만
> 전자는 firebase_bridge 가 기록하는 조리 진행 상태(sequence_runner emit), 후자는
> coord_service + fk_worker 가 기록하는 하드웨어 텔레메트리. 두 키를 통일하려는
> 시도는 동작을 깨뜨린다.

`start_all.sh`: 패널 3개 + Flask + fk_worker + cloudflared 일괄 기동.
`ROS_DOMAIN_ID=24` 를 명시 export.

---

## 5. `ros2_move_recoder` (common) — 동작 녹화·재생 도구이자 재생 엔진

`robo_chef.sequence_runner` 가 import 하는 `playback.play_segment` 의 본진.
사람이 직접 m0609 를 잡아 동작을 만들 때 GUI 도구로도 쓰인다.

| 모듈 | 역할 |
|---|---|
| `recorder.py` | `/dsr01/joint_states` BEST_EFFORT 고속 수집 → `raw.json` |
| `smoother.py` | Savitzky-Golay + 정지구간 압축 + arc-length 균등 다운샘플(≤100점) → `smooth.json` |
| `playback.py` | **헤드리스 재생 코어** — `amovesj` 비동기 모션 + gripper 이벤트 타임라인. PyQt/입력 의존 없음. robo_chef 가 직접 import |
| `player.py` | `macro_player` 노드 (단독 재생 CLI) |
| `gui.py` | PyQt5 통합 GUI. **BringupManager** 가 dsr_bringup2 launch 를 spawn/shutdown (SIGINT→wait→SIGKILL→reap 3단 종료) |
| `dualsense_worker.py` | PS5 컨트롤러 60Hz 폴링 → Qt 시그널로 GUI 에 dispatch |
| `gripper_worker.py` / `onrobot.py` | OnRobot RG2/RG6 Modbus TCP 제어 (Queue 기반 단일 worker thread 로 직렬화) |

데이터 포맷 상세: `common/ros2_move_recoder/dev-docs/data-formats.md`.

---

## 6. 통신 채널 요약

### ROS 토픽

| 토픽 | 타입 | 송신 | 수신 |
|---|---|---|---|
| `/recipe` | std_msgs/String | firebase_bridge | sequence_runner |
| `/cooking_status` | std_msgs/String | sequence_runner | firebase_bridge |
| `/dsr01/joint_states` | sensor_msgs/JointState | dsr_controller2 | coord_service, recorder |
| `/robo_chef/coords` | std_msgs/String | coord_service | web (ros2_coord_client) |
| `/rosout` | rcl_interfaces/Log | 전 노드 | log_bridge |
| `/robo_chef/order_request` | std_msgs/String | (없음, legacy) | order_receiver (진단용) |

### ROS 서비스

| 서비스 | 타입 | 서버 | 클라이언트 |
|---|---|---|---|
| `unlock_system` | std_srvs/Trigger | sequence_runner | (관리자, 외부) |
| `/robo_chef/get_coords` | std_srvs/Trigger | coord_service | (거의 미사용, REST 가 토픽 캐시 사용) |
| `/dsr01/motion/move_stop` | dsr_msgs2/MoveStop | dsr_controller2 | robot_bridge, coord_service SafetyBridge |
| `/dsr01/motion/move_joint` | dsr_msgs2/MoveJoint | dsr_controller2 | robot_bridge |
| `/dsr01/system/set_robot_mode` | dsr_msgs2/SetRobotMode | dsr_controller2 | robot_bridge, coord_service SafetyBridge |

### Firebase RTDB 경로

| 경로 | writer | reader |
|---|---|---|
| `/orders` | **backend `POST /api/orders` 단일 출처** (kiosk 직접 write 폐기, 2026-05-31) | firebase_bridge (listen), customer_status (read), admin_monitor |
| `/recipes` | recipe_seeder (시드), admin_monitor (편집) | kiosk (메뉴 매칭), backend (`create_order` lookup) |
| `/robot_status` | firebase_bridge | kiosk, customer_status |
| `/robot_state` | coord_service, fk_worker, SafetyBridge (필드 분할 소유 — §6.1) | admin_monitor |
| `/telemetry/robot_status` | coord_service (raw 5Hz) | (진단) |
| `/telemetry/robot_state` | coord_service SafetyBridge | (진단) |
| `/commands/robot` | admin_monitor | coord_service SafetyBridge |
| `/commands/robot_ack` | coord_service SafetyBridge | admin_monitor |
| `/logs` | app.py | admin_monitor |
| `/dsr_log` | log_bridge | admin_monitor |
| `/phase` | **customer_status 자체 (self-control)** — confirmCancel 등 사용자 조작 reset 신호. main_side 는 발행 안 함 | customer_status, kiosk(예약) |

### 6.1 `/robot_state` 필드 ownership matrix

`/robot_state` 는 단일 노드를 다수 writer 가 merge-update. 필드별 단독 소유자를
고정해 race 와 silent overwrite 를 방지한다.

| 필드 | 단독 writer | 주기 | 출처 |
|---|---|---|---|
| `joint_positions` | `coord_service._robot_state_loop` | 2 Hz | `/joint_states` (ros2 control) |
| `mode` | `coord_service.SafetyBridge` | ack 시점 | `dsr_msgs2/GetRobotState` |
| `tcp_position` / `tcp_updated_at` | `fk_worker` (sub1_side backend) | 2 Hz polling, dedup | joint 변화 시 FK 재계산 |
| `last_updated` | (마지막 writer 의 시각) | — | ISO timestamp (UTC) |
| `robot_status` / `gripper_status` / `speed_scale` / `current_recipe` / `current_step` / `error_code` | `app.py startup()` 1 회 + 외부 API 호출 시 | sparse | backend `update_robot_state` |

writer 가 자기 소유 외 필드를 건드리면 안 됨. 추가 시 표를 먼저 갱신.

### 6.2 `/orders` 스키마 (2026-05-31 확정)

backend `firebase_client.create_order(items, total)` 가 단독 작성:
- `order_id` — `ORD_<YYYYMMDD_HHMMSS>_<6hex>` (timestamp prefix → 키 정렬 = 시간 정렬)
- `items[]` — `{recipe_id, qty, name, price, subtotal}` 그대로 전달
- `recipe_data` — backend 가 `/recipes/<id>.segments` lookup 해 채움 (`{<rid>: {segments, name}}`)
- `recipe_id` — 호환용 단일 대표 (multi-item 일 때 첫 번째)
- `total`, `status` (`pending`→`cooking`→`delivered`|`failed`), `order_time`, `started_at`, `completed_at`, `failed_reason`

`segments: []` 인 recipe 한 건이라도 있으면 `RecipeNotSeededError` → HTTP 400.
즉 ros2_move_recoder GUI 로 녹화 후 `recipe_seeder.py` 갱신이 없으면 주문 자체가
거부됨 — 단계 A(녹화) 와 단계 B(주문) 의 동기화 보장.

---

## 7. 잔존 이슈 / 정리 포인트

**2026-05-31 워크플로우 무결성 보완 (Record→Order→Cook→Status→Render 5 단계 audit) 완료 분:**

- ✅ **레시피 카탈로그 시드 누락** — `recipe_seeder.py` 가 RAMEN/KIMCHI_STEW/STEAK 3 종 정의. `segments` 는 GUI 녹화 전까지 빈 배열 + 명확한 TODO. backend 가 빈 segments 발견 시 HTTP 400 으로 거부 → 단계 A(녹화) ↔ 단계 B(주문) 동기화 강제.
- ✅ **kiosk ↔ backend schema drift** — kiosk 의 직접 RTDB write 폐기. `POST /api/orders {items, total}` 만 사용. backend 가 `/recipes` lookup + `recipe_data` 채움 + `ORD_` ID 단일 발급 + `prune_orders(keep=10)` 담당.
- ✅ **firebase_bridge crash recovery** — 노드 재기동 시 `_recover_orphans()` 가 status="cooking" 인데 in-memory `_cur_order==None` 인 orphan 을 `failed("bridge restart")` 로 정리.
- ✅ **`/recipe` publish 실패 가드** — `_try_dispatch_next` 가 `publish()` try/except. 예외 시 status→pending 복귀 + busy 해제.
- ✅ **sequence_runner pre-flight 검증** — `/recipe` 수신 즉시 모든 segment 의 `smooth.json` 존재 검사. 누락 시 모션 시작 전 `state=ERROR("missing segments: …")` 즉시 종료.
- ✅ **`/phase` 컨트랙트 명확화** — main_side 무관, customer_status self-control 신호임을 패널 코드 주석 + 본 문서 §6 에 명시.
- ✅ **`/robot_state` ownership matrix** — §6.1 표로 필드 단독 소유자 고정. 코드 주석에도 1 줄씩 박음.
- ✅ **`/robot_status` stale 검출** — `firebase_bridge._on_status` 가 `last_updated` ISO 시각 부여. admin_monitor 기존 stale 감지 로직 (3s) 와 즉시 동기.

**여전히 유효한 점검 항목 (워크플로우 무결성 외):**

1. **`/robo_chef/order_request` 토픽 leftover** — `firebase_bridge` 가 `/orders` 를
   직접 listen 하므로 발행자가 사라졌지만 `order_receiver` 노드와 토픽 정의는
   남아 있음. 진단 도구로 명시 라벨링 (위 §3 표) 또는 제거 검토.
2. **`RECORDS_DIR` 가 sequence_runner 에 절대경로로 박힘** — 다른 PC 에서는
   `~/cobot_ws/src/ros2_move_recoder/records` 가 없을 수 있음. `COBOT1_RECORDS_DIR`
   같은 환경변수 도입 고려.
3. **m0609 IP/PORT 가 `BringupManager.HOST_BY_MODE` 에 하드코딩** (192.168.1.100:12345).
   현장 IP 변동 시 두 줄 수정 필요. 환경변수화 가능.
4. **`app.py /api/ros2/status` 는 여전히 `ros2 topic list` subprocess** — 관리자
   진단용으로 호출 빈도 낮아 유지. 운영 경로에서는 영향 없음.
5. **`cooking_core` 가 `RECOVERY` 같은 중간 상태를 emit 하지 않음** — kiosk 의
   `_robotBusy` 해제는 `IDLE`/`DONE`/`RECOVERY*` 셋 다 받도록 보강 완료(2026-05-24).
   향후 자동 복구 시퀀스 도입 시 cooking_core 에 RECOVERY emit 추가 필요.
6. **통합 회귀 테스트 부족** — `cooking_core`/`order_core` 단위테스트는 있으나
   ERROR→unlock→다음 주문 자동 dispatch 같은 시나리오 회귀가 없음.
7. **records/ 폴더 cross-PC 동기화 메커니즘 부재** — recorder PC 에서 녹화된
   `smooth.json` 이 메인 PC 에 같은 경로로 있어야 sequence_runner 가 재생 가능.
   현재는 `rsync` 등 수동 동기화 전제. NFS/SSHFS 또는 RTDB blob 업로드 옵션 검토 가능.

---

## 8. 환경 변수 표

| 변수 | 사용처 | 기본값 / 우선순위 | 비고 |
|---|---|---|---|
| `ROS_DOMAIN_ID` | 전 노드 | **24** (`start_all.sh`, `run_*.sh`, `app.py` 가 강제 주입) | bashrc 의 cobot3 기본값(130) 과 분리. cross-PC 통신 전제 |
| `RMW_IMPLEMENTATION` | 전 노드 | `rmw_fastrtps_cpp` (~/.bashrc) | Cyclone 사용 시 수동 변경 |
| `FIREBASE_CRED_PATH` | firebase_bridge, coord_service, log_bridge, firebase_client, fk_worker | env → `~/.config/cobot1/firebase-key.json` → 워크스페이스 후보 경로 → backend/serviceAccountKey.json | 다른 PC 이관 시 env 로 일괄 override |
| `FIREBASE_DB_URL` | 위와 동일 | `https://robochef-5d9b6-default-rtdb...` | dev/staging RTDB 분리 시 |
| `GRIPPER_IP` | sequence_runner, ros2_move_recoder | `192.168.1.1` | OnRobot RG2/RG6 |
| `GRIPPER_PORT` | 위 | `502` | Modbus TCP |
| `GRIPPER_TYPE` | 위 | `rg2` | `rg2` / `rg6` |

---

*분석 범위: cobot1 내 robo_chef · realtime_scan · web · ros2_move_recoder
전 파일. 최종 갱신: 2026-05-24 (architecture 감사 + ROS_DOMAIN_ID 통일 +
robot_bridge rclpy 네이티브 전환 + Firebase 경로 환경변수화 작업 직후).*
