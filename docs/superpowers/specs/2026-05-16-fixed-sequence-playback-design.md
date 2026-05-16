# 설계 스펙 — 고정 시퀀스 재생 + /orders 기반 복합주문 (cobot1)

- 날짜: 2026-05-16
- 대상: `/home/hoon/cobot_ws/src/cobot1`
- 성격: 조리 로직을 **동적 액션 해석 → 사전 녹화 고정 시퀀스 재생**으로 전면 전환, 주문 인지를 **`/orders` 복합주문 기반**으로 전환, 불필요 노드/코드 과감 제거

---

## 1. 목적과 범위

### 1.1 전환 요지
- 기존: `firebase_bridge → recipe_parser → state_manager → executer → ActionManager(동적 플러그인)` 가 레시피를 런타임 해석.
- 신규: 메뉴별 **사전 녹화 세그먼트 매크로 체인**을 그대로 재생. 주문은 `/orders`의 실제 주문 엔트리(복합주문)에서 인지.
- 흐름: `키오스크 → RTDB /orders(pending) → firebase_bridge(주문 감지·전개) → sequence_runner(재생) → 로봇`, 상태는 RTDB 경유로 조리현황/디버깅 패널에 실시간.

### 1.2 확정 요구사항 (브레인스토밍 합의)
1. **메뉴 = 재사용 세그먼트 체인.** 한 메뉴 = 순서 있는 세그먼트 매크로 목록. 런너가 순차 재생.
2. **런너/트리거**: robo_chef에 `sequence_runner` 노드 신규. `firebase_bridge` 유지(클라우드 경계).
3. **재생 코어 = 접근 A**: `ros2_move_recoder/playback.py` 신설(헤드리스). `gui.py`·`player.py`는 이 코어 호출로 리팩터(단일 진실 소스).
4. **그리퍼 재생 필수**: GUI 그리퍼 타임라인 로직을 코어로 이식. 세그먼트는 GUI로 녹화(그리퍼 캡처 포함).
5. **세그먼트 정의는 RTDB에 보관**: 메뉴별 세그먼트 체인을 RTDB `recipes/<recipe_id>.segments`에 저장(디버깅/admin 패널에서 저작). 키오스크가 주문 생성 시 `recipes/<rid>` 스냅샷을 `orders/<key>.recipe_data[rid]`에 통째로 박으므로, **런너는 주문에 내장된 `recipe_data[rid].segments`를 사용**(주문 시점 스냅샷 일관성).
6. **주문 인지 = `/orders` 복합주문 기반** *(order_count 감지 폐기)*: firebase_bridge가 RTDB `/orders` child 이벤트에서 `status=="pending"` 주문을 `order_time` FIFO로 잡아, `items[]`(순서 있는 권위 목록)를 전개해 처리. **복합주문**: `items[]` 배열 순서대로, 각 항목을 `qty`회 반복.
7. **상태 보고 = 주문/항목/세그먼트 단위 + 패널 개편(Option 2)**: `/cooking_status` 스키마를 주문·항목·세그먼트 차원으로 정의하고 `customer_status` 패널을 개편.
8. **주문 상태 전이**: `/orders/<id>/status` `pending → cooking → delivered`, 실패 시 `failed`. firebase_bridge가 전이 담당.
9. **실패 처리 = 즉시 정지 + ERROR + 수동 복구**: 세그먼트 실패 시 `DR_SSTOP` 즉시 정지, `state=ERROR`, `unlock_system`(Trigger) 수동 호출로만 복귀. 자동 재시도 없음.
10. **과감 제거**: robo_chef 동적 엔진 일체, recipe_tester, robo_chef 중복 `src/` 트리, web 주문 퍼블리셔 + 죽은 코드.

### 1.3 범위 밖 / 의도적 보류
- **경로 참조 정합성**(패키지 위치 vs 코드 내 하드코딩): 사용자 지시로 범위 밖. 코어는 절대경로를 인자로 받게 설계해 회피.
- **realtime_scan**(`coord_service`, `log_bridge`), 키오스크 주문 적재, admin/디버깅 패널 출력: 동작 변경 없음.
- 키오스크의 `recipes/<rid>/order_count` transaction 증가: 본 설계에서 **소비자 없음**(레거시 dead write). 제거는 선택, 이번 범위 밖(노이즈만, 무해). §6에 기록.
- `/robot_status` vs `/robot_state` 전반 정리: 별건. 본 작업이 닿는 `robot_status` 경로만 정합화.

### 1.4 선행 조건 (코드 아님, 사용자 작업)
조리용 녹화 매크로 현재 0개. 실제 조리하려면 사용자가 ros2_move_recoder **GUI로 세그먼트 녹화→스무딩**해 `records/<name>/smooth.json` 생성 + RTDB `recipes/<recipe_id>.segments`에 세그먼트명을 순서대로 등록해야 함. 본 스펙은 이를 가능케 하는 코드 경로 구축이며, 매크로 콘텐츠 저작은 코드 산출물 아님.

---

## 2. 아키텍처

```
[키오스크] (불변)
  RTDB /orders.push({
     order_id, recipe_id, recipe_ids[], menu_counts{}, total, order_time,
     status:"pending",
     items:[ {name, qty, recipe_id, price, subtotal}, ... ],          ← 복합주문 권위 목록(순서O)
     recipe_data:{ "<rid>": <recipes/<rid> 스냅샷(segments 포함)> }    ← 세그먼트 출처
  })
        |
        v
[firebase_bridge] (robo_chef, 개편)
  - RTDB /orders child_added(+초기 스캔) → status=="pending" 필터, order_time FIFO
  - 동시 1건만: 현재 주문 종료(delivered/failed) 전 다음 주문 미착수
  - 착수 시 /orders/<id>/status = "cooking" (started_at)
  - items[] 전개 → /recipe 발행 (아래 잡 페이로드)
  - /cooking_status 구독 →
        · robot_status.set(<status 객체 그대로>)
        · DONE  → /orders/<id>/status="delivered"(delivered_at)
        · ERROR → /orders/<id>/status="failed"(error_msg) + error_logs.push
        |
        v
[sequence_runner] (robo_chef, 신규, DSR 소유)
  1. /recipe 수신: { order_id, jobs:[ {recipe_id, qty, segments:[...]}, ... ] }
  2. state!=IDLE 이면 무시(가드)
  3. for item i in jobs (1..M):
       for q in 1..item.qty:
         for seg k in item.segments (1..S):
            /cooking_status {state:"EXECUTING", order_id, recipe_id,
                             item_index:i,item_total:M, qty_index:q,qty_total:item.qty,
                             segment_name:seg, segment_index:k,segment_total:S, error_msg:""}
            play_segment(<RECORDS_DIR>/<seg>/smooth.json, gripper=onrobot.RG, ...)
            실패 → DR_SSTOP, state=ERROR, 루프 중단(§5)
  4. 전체 완료 → /cooking_status {state:"DONE", order_id, ...} → state=IDLE
  5. srv unlock_system(Trigger): ERROR→IDLE
        |
        v
   Doosan m0609 (dsr01) + OnRobot 그리퍼(Modbus TCP)

[realtime_scan] (불변)  coord_service: joint_states→RTDB robot_state ; log_bridge: /rosout→RTDB dsr_log
   ※ sequence_runner 로그는 rclpy logger → /rosout → log_bridge가 자동 포착(전 과정 log)

[web 패널]  kiosk(주문, /robot_status state 읽기) · customer_status(개편, 신 스키마) · admin(robot_state/dsr_log/logs, 불변)
```

---

## 3. 컴포넌트 설계

### 3.1 (신규) `ros2_move_recoder/ros2_move_recoder/playback.py`
헤드리스 재생 코어. PyQt·DualSense·`input()` 의존 0. gui/player/runner 공유.

```python
@dataclass
class PlayResult:
    ok: bool
    duration_sec: float
    error: str | None
    measured_duration_sec: float | None

def play_segment(
    smooth_path: str, *,
    gripper=None, require_autonomous: bool = True,
    on_progress=None, abort_event=None, logger=None,
) -> PlayResult
```
**내부(기존 코드 이식, 신규 모션 로직 없음)**: smooth.json 로드 → (require_autonomous면) `get_robot_mode()==ROBOT_MODE_AUTONOMOUS` 확인, 아니면 `ok=False` → `amovesj([posj(*w)..], vel, acc)` async + `check_motion` 폴링(player.py) → 그리퍼 이벤트 타임라인 스레드(gui.py `_start_gripper_play_timeline`/`_ensure_gripper_final_state`의 소요시간 추정 우선순위·`GRIP_MIN_GAP_S`·최종상태 강제 그대로 이식) → `abort_event` set 또는 오류 시 `DR_SSTOP`+스레드 정리 → `PlayResult`.

**리팩터(영향도 점검 대상)**: `player.py`를 play_segment 호출 얇은 CLI로 축소(+`--yes` 비대화, `ros2 run ... player <name> [--yes]` 인터페이스 유지).

> **설계 정정(Task 4 실행 중 발견):** gui.py는 모션(`DsrWorker` 워커 스레드)과 그리퍼 타임라인(`MainWindow` GUI 스레드, 시그널 구동, ops-slider 비율·`measured_at_play_vel` 학습 보유)이 **의도적으로 분리·결합된 구조**다. 단일 블로킹 호출인 `play_segment`(+단순화된 `_GripperTimeline`)로 강제 치환하면 그리퍼 타이밍 정확도·서비스 기반 abort/pause/resume·6개 시그널 UI 핸들러·measured-duration 학습 루프를 회귀시킨다. → **gui.py 전체 dedup은 descope.** gui.py는 기존 검증된 재생/그리퍼 경로를 유지하고, 안전·무회귀 부분집합만 반영: ① `playback.play_segment` import ② `DsrWorker._play_abort`(threading.Event) 추가로 MoveStop 서비스가 느릴 때의 보조 abort 보강. 신규 조리 경로(`player.py`·`sequence_runner`)가 공유 코어를 사용하므로 "단일 진실 소스" 핵심 목표는 충족(사용자 요구사항 불변). gui↔playback 완전 일원화는 `_GripperTimeline`을 ops-ratio/measured-vel 인지로 확장 + Qt 진행 시그널 연동이 필요한 별도 작업으로 분리(YAGNI — 현 pivot 범위 밖). `smoother.py`/`recorder.py`/`dualsense_worker.py`/`gripper_worker.py`/`onrobot.py` 무변경.

### 3.2 (신규) `robo_chef/nodes/sequence_runner.py`
DSR 소유 ROS2 노드. 기존 `executer.py`의 DSR 초기화 패턴 계승.
- 노드명 `sequence_runner` + DSR 헬퍼노드(ns `dsr01`, `DR_init.__dsr__node` 할당), `MultiThreadedExecutor(4)`.
- DSR init: `ROBOT_ID="dsr01"`, `ROBOT_MODEL="m0609"`, `set_tool("Tool Weight")`, `set_tcp("GripperDA_v1")`, MANUAL→AUTONOMOUS (executer 동일).
- 그리퍼: `onrobot.RG` 1회 생성(env `GRIPPER_IP`/`GRIPPER_PORT`/`GRIPPER_TYPE`). play_segment에 주입.
- sub `/recipe`(std_msgs/String): `{order_id, jobs:[{recipe_id, qty, segments[]}...]}` 파싱.
  - jobs 비었거나 segments 누락 → `state=ERROR`, error_msg.
- 상태 가드: `state != IDLE` 이면 새 `/recipe` 무시. **IDLE→EXECUTING 전이는 `threading.Lock` 으로 원자화**(단일 물리로봇 안전, 콜백그룹 무관), `cc.run_jobs` 는 락 밖 실행. **`cc.run_jobs` 예기치 못한 예외 시 ERROR `/cooking_status` 발행 + state=ERROR**(unlock_system 으로 복구 — EXECUTING 고착 dead-end 제거).
- 3중 루프(item→qty→segment) §2대로. 세그먼트마다 `/cooking_status` 발행 후 `play_segment` 호출.
  - `smooth_path = <RECORDS_DIR>/<seg>/smooth.json` (RECORDS_DIR=상수, 경로 정합 §1.3 보류; 코어는 절대경로 인자).
  - 파일 없음 → `state=ERROR`, error_msg="missing segment file: <seg>".
- 완료 → `/cooking_status {state:"DONE", order_id, ...}` → `state=IDLE`.
- srv `unlock_system`(std_srvs/srv/Trigger): `state==ERROR`→`IDLE`(success=True) **후 `/cooking_status {state:"IDLE", order_id:<직전>}` 1회 발행**(firebase_bridge 재개 신호); 그 외 success=False. (서비스명 유지 → 기존 web/coord_service Firebase 명령 경로 재사용.)
- 노드 종료 시 abort_event set → 모션 정지.

### 3.3 (개편) `robo_chef/nodes/firebase_bridge.py`
- **제거**: `recipes` `order_count` 리스너 및 `_on_recipe_change` (order_count 방식 폐기).
- **신규 주문 인지**: RTDB `/orders` 에 listener(+기동 시 1회 스캔). `status=="pending"` 인 주문을 `order_time` 오름차순 큐잉.
- **동시 1건**: 내부 `busy` 플래그. 현재 주문이 `delivered`/`failed`로 종료될 때까지 다음 주문 미착수.
- **착수**: 선택된 주문 → `update_order_status(order_id, "cooking", started_at=now)` (재선택 방지: 이제 status!=pending) → `/recipe` 발행:
  ```json
  { "order_id": "<key>",
    "jobs": [ { "recipe_id":"RAMEN", "qty":2,
                "segments": <order.recipe_data["RAMEN"].segments> }, ... ] }
  ```
  `jobs`는 `order.items[]` 순서대로. `segments`는 `order.recipe_data[item.recipe_id].segments`에서 추출(없으면 그 주문 `failed` 처리 + 로그).
- **`/cooking_status` 구독**(`_on_status_receive`). `busy` 해제 규칙은 **단일 기준: `state ∈ {DONE, IDLE}` 수신 시에만 해제** 후 다음 pending 처리. (`DONE`=정상 종료, `IDLE`=unlock 후 재개 신호.)
  - 항상: `robot_status.set(status_data)` (스키마 통과, customer_status가 읽음).
  - `state=="DONE"` → `update_order_status(order_id,"delivered", delivered_at=now)`, `busy=False` → 다음 pending 처리.
  - `state=="ERROR"` → `update_order_status(order_id,"failed", error_msg=...)` + `error_logs.push({timestamp, order_id, recipe_id, item_index, segment_name, message})`. **`busy` 유지**(주문은 `failed`로 종결됐으나 런너가 ERROR라 로봇 점유 중) → 수동 `unlock_system` 후 런너가 `state:"IDLE"` 발행 시 비로소 `busy=False`, 다음 pending 처리.
  - `state=="IDLE"`(unlock 재개 신호) → `busy=False` → 다음 pending 처리.
- Firebase 초기화/인증 경로는 기존 그대로(하드코딩 경로는 §1.3 보류).

### 3.4 (개편) `sub1_side/web/panel/customer_status/index.html`
RTDB `/robot_status` 신 스키마로 조리 진행 렌더링.
```json
{ "state":"IDLE|EXECUTING|DONE|ERROR", "order_id",
  "recipe_id","item_index","item_total","qty_index","qty_total",
  "segment_name","segment_index","segment_total","error_msg" }
```
- 기존 `current_step/total_steps` 6-step 하드코딩 파생 제거.
- 표시: 현재 메뉴(`recipe_id`) + 항목 진행 `item_index/item_total` + 수량 `qty_index/qty_total` + 세그먼트 진행 `segment_index/segment_total`(`segment_name` 라벨). `segment_total` 동적.
- `state==="EXECUTING"` 진행, `"DONE"` 완료 연출, `state.startsWith("ERROR")` → 기존 에러 오버레이(`_onPhase('error')`) 재사용.
- `/orders` 최신 1건, `/phase` 제어 신호 로직 유지.

### 3.5 (소수정) `sub1_side/web/backend/recipe_seeder.py`
시드 `recipes/<rid>` 스키마에 `segments:[...]` 필드 추가(신 모델). 세그먼트명은 사용자 녹화 산출물 의존 → **플레이스홀더 + 주석("GUI 녹화 후 실제 세그먼트명으로 교체")**. 운영 가이드 성격, 코드 동작 무관.

---

## 4. 데이터 / 인터페이스 계약

| 채널 | 타입 | 스키마 |
|---|---|---|
| RTDB `/orders/<key>` | RTDB JSON | 키오스크 작성(불변). `items[]`(순서·권위), `recipe_data[rid]`(스냅샷, `segments` 포함), `status`(pending→cooking→delivered/failed) |
| RTDB `recipes/<rid>` | RTDB JSON | `{ segments:[...], ...메타 }`. admin/디버깅 패널 저작. 키오스크가 주문 시 스냅샷 |
| `/recipe` | topic `std_msgs/String` | `{ order_id, jobs:[ {recipe_id, qty, segments[]} ... ] }` (firebase_bridge 발행) |
| `/cooking_status` | topic `std_msgs/String` | `{state, order_id, recipe_id, item_index, item_total, qty_index, qty_total, segment_name, segment_index, segment_total, error_msg}` |
| RTDB `robot_status` | RTDB JSON | `/cooking_status`와 동일 객체(firebase_bridge set) → customer_status/kiosk 구독 |
| RTDB `error_logs` | RTDB push | `{timestamp, order_id, recipe_id, item_index, segment_name, message}` |
| `unlock_system` | srv `std_srvs/srv/Trigger` | ERROR→IDLE |
| 그리퍼 | Modbus TCP | `onrobot.RG` (env `GRIPPER_IP`/`PORT`/`TYPE`) |
| RTDB `robot_state`,`dsr_log` | RTDB | realtime_scan 기록(불변) — 로봇팔 움직임/전 과정 로그 → 디버깅 패널 |

상태머신(sequence_runner): `IDLE → EXECUTING(item→qty→segment 3중 루프) → DONE → IDLE` / 실패 `EXECUTING → ERROR --(unlock_system)--> IDLE`.
주문 상태(firebase_bridge): `pending → cooking → delivered` / 실패 `failed`. 동시 1건 FIFO(order_time).

---

## 5. 에러 처리
- 트리거: `play_segment.ok=False`(amovesj 비정상/예외/abort/그리퍼 결함/AUTONOMOUS 미충족) 또는 세그먼트 파일·`segments` 누락.
- 처리: sequence_runner → `DR_SSTOP` 즉시 정지 → `/cooking_status{state:"ERROR", ... ,error_msg}` → firebase_bridge가 `robot_status.set` + `/orders/<id>/status="failed"` + `error_logs.push`.
- 로봇 그 자리 정지(홈복귀 없음). `state=ERROR` 유지, 새 `/recipe` 무시. **자동 재시도 없음.**
- 복구: `unlock_system`(Trigger) 수동 호출 → `ERROR→IDLE`. 이후 firebase_bridge가 다음 pending 처리.
- AUTONOMOUS 미충족 시 침묵 진행 금지 — 즉시 ERROR.
- customer_status: `state.startsWith("ERROR")` → 에러 오버레이. admin: `error_logs`/`dsr_log` 출력.

---

## 6. 영향도 점검 & 회귀 (CLAUDE.md "변경 시 영향도 체크")
- `recipe_msgs/Recipe.action` 삭제 → 참조처 executer/state_manager(둘 다 삭제). 외부 참조 없음(확인 완료).
- `/recipe`: 발행=firebase_bridge(개편)·삭제될 recipe_tester, 구독=삭제될 recipe_parser→신규 sequence_runner. **페이로드 스키마 변경**(메뉴 단건 → 주문 잡). 발행/구독 모두 본 작업 내 동시 변경 → 정합.
- `/cooking_status`: 발행 state_manager(삭제)→sequence_runner, 구독 firebase_bridge(개편). **스키마 변경** → firebase_bridge 매핑 + customer_status 개편으로 정합.
- `order_count` 리스너 제거: 소비자 firebase_bridge 한 곳뿐(개편으로 제거). 키오스크의 order_count 증가 write는 잔존하나 **소비자 0** → 무해 dead write(선택적 후속 정리, 범위 밖).
- `/orders` 신규 listener: 기존 `get_orders`(web backend, 읽기)·키오스크 pruning(>10 삭제)와 공존. status 값 집합(pending/cooking/delivered/failed)이 web `/api/orders` 표시·키오스크와 호환되는지 확인(표시 전용이라 안전, 검증 항목).
- `unlock_system` 서비스명 유지 → 이를 호출하던 경로 그대로 동작.
- `gui.py` 리팩터 → **회귀 필수**: (a) GUI 녹화→스무딩→재생 (b) DualSense 조그/매크로 (c) bringup/모드전환 (d) 그리퍼 타임라인 재생 (e) CLI `player --yes`. 재생 경로 play_segment 일원화 확인.
- 패키지 간 import(robo_chef→ros2_move_recoder): `robo_chef/package.xml`에 `<exec_depend>ros2_move_recoder</exec_depend>`, 동일 워크스페이스 빌드 전제, ament_python 모듈 import 가능 확인.
- kiosk `/robot_status` 읽기: `state` 문자열만 사용 여부 코드 확인 후 신 state 집합 호환 검증.
- customer_status 개편: `/orders`·`/phase` 기존 로직 유지되는지 확인.

---

## 7. 삭제 목록 (확정)
**robo_chef**: `nodes/{recipe_parser,state_manager,executer,recipe_tester}.py`; `core/` 전체(`action_manager.py`,`base_action.py`,`actions/*`); `data/*.json`; `src/interfaces/recipe_msgs/` 전체; 중복 `src/robo_chef/` 트리 전체; 위 삭제분 종속 `test/` 정리; `setup.py` entry_points → `firebase_bridge`,`sequence_runner`만(오타 `firbase_bridge` 교정); `package.xml` → `recipe_msgs` 제거, `ros2_move_recoder` exec_depend 추가, 누락 `rclpy`/`std_msgs`/`std_srvs` 명시.
**web(sub1_side)**: `backend/ros2_order_publisher.py`, `backend/ros2_srv_call.py`, `backend/run_ros2_publisher.sh`, `web/start_all.sh`의 해당 기동 라인, `backend/app.py`의 미사용 `_start_order_watcher`/`_process_order`/`_processing_orders`/`_ros2_publish_order` + 이에만 의존하는 `/api/ros2/publish_order` 라우트.
**영향 확인**: 라이브 주문 경로는 신규 firebase_bridge `/orders` listener. 키오스크가 `/orders.push(status:pending)` 하므로 정상. 삭제될 publisher는 별도 no-op 경로였음(검증 완료).

---

## 8. 테스트 전략
- `playback.py` 단위(무하드웨어): `amovesj`/`check_motion`/`get_robot_mode` monkeypatch + fake gripper — 소요시간 추정·그리퍼 스케줄링·`GRIP_MIN_GAP_S`·abort·AUTONOMOUS 미충족.
- `sequence_runner` 노드테스트(play_segment mock): 단일/복합 jobs → `/cooking_status` 시퀀스(item/qty/segment 인덱스) 검증, jobs/segments 누락→ERROR, 비IDLE 가드, `unlock_system`.
- `firebase_bridge`(Firebase mock): pending 2건 → FIFO 1건만 착수, status pending→cooking→delivered, ERROR→failed+error_logs, busy 가드, 복합주문 items[]→jobs 전개·recipe_data segments 추출.
- `customer_status`: 신 스키마 샘플로 진행/에러 렌더 수동 확인.
- 회귀 체크리스트(§6) 수동 수행. 구현은 superpowers TDD 준수.

---

## 9. 구현 순서(권장)
1. `playback.py` 추출 + 단위테스트(player/gui 미변경).
2. `player.py` 축소(+`--yes`), CLI 회귀.
3. `gui.py` 재생/그리퍼 경로 play_segment 치환, GUI 회귀(§6).
4. `sequence_runner.py` 신규 + 노드테스트. `setup.py`/`package.xml` 갱신.
5. `firebase_bridge.py` 개편(/orders listener·복합 전개·status 전이) + 테스트.
6. `customer_status` 패널 개편.
7. 삭제 목록(§7) 일괄 제거 + 영향도 grep 스윕 재검증.
8. `recipe_seeder` segments 필드 + 운영 문서.

---

## 10. 미해결/보류 (명시)
- **경로 정합성**: `RECORDS_DIR` 실제 위치 vs 코드 하드코딩 불일치 보류(사용자 지시). 코어는 절대경로 인자로 회피, sequence_runner `RECORDS_DIR` 상수값은 추후 확정.
- **조리 매크로 부재**: 실제 동작은 사용자가 GUI 녹화 + `recipes/<rid>.segments` 등록 후 가능(§1.4).
- **키오스크 order_count dead write**: 소비자 0, 무해. 후속 정리 권장(범위 밖).
- git: cobot1 루트는 git 저장소 아님 → 본 스펙 커밋 생략(요청 시 init).
