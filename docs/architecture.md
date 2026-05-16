# cobot1 — 라면/스테이크 조리 로봇 아키텍처

> 이 문서는 현재 코드 기준으로 **노드와 각 노드가 담당하는 기능**을 정리한 것이다.
> (경로 참조 문제는 의도적으로 무시 — 폴더 구조 재배치 진행 중)

---

## 1. 한눈에 보는 전체 그림

손님이 **키오스크**에서 메뉴를 주문 → **Firebase**(클라우드 DB)에 주문 적재 → **로봇 PC**가 주문을
받아 레시피를 단계별 동작으로 풀어 **Doosan m0609 팔**을 움직여 조리 → 진행 상태/로그가 다시
Firebase로 올라가 **고객 현황판 / 관리자 대시보드**에 표시된다.

```
[키오스크]──┐                                  ┌──[고객 현황판]
            ▼                                  │
        Firebase RTDB (robochef-5d9b6)  ◀──────┤
            ▲   │  ▲                            └──[관리자 대시보드]
   주문/레시피 │  │ 주문 watch                       ▲ 로봇상태/로그
            │  ▼  │                                 │
  [web backend]   └────────────┐         [realtime_scan: 텔레메트리/안전]
  (Flask, sub PC)              │                     ▲
                               ▼                     │ joint_states / 시스템 서비스
                        [robo_chef 엔진] ───────▶ Doosan m0609 (dsr01)
                        (레시피→동작, 메인 PC)
```

물리적으로 **3개의 위치(side)** 로 코드가 나뉜다.

| side | 위치/역할 | 핵심 패키지 |
|---|---|---|
| `main_side` | 로봇 팔이 붙은 메인 PC. 실제 조리 실행 + 로봇 텔레메트리 | `robo_chef`, `realtime_scan` |
| `sub1_side` | 웹/주문 처리 서브 PC. 키오스크·대시보드·Firebase 브리지 | `web` (Flask backend + 정적 패널) |
| `common` | 양쪽 공용 도구. 동작 티칭(녹화/재생) | `ros2_move_recoder` |

---

## 2. `robo_chef` — 조리 엔진 (핵심)

레시피(JSON)를 받아 단계별 로봇 동작으로 실행하는 ROS 2 패키지. **노드 5개 + 동작 플러그인 엔진**.

### 2.1 노드와 데이터 흐름

```
firebase_bridge ──/recipe──▶ recipe_parser ──/parsed_recipe──▶ state_manager
   ▲  │ (또는 recipe_tester가 /recipe 대체)                          │
   │  └────────────── /cooking_status ◀──────────────────────────────┘
   │                                                          action: execute_recipe
Firebase                                                              ▼
                                                                  executer ──▶ ActionManager ──▶ m0609
```

| 노드 | 노드명 | 담당 기능 | 주요 인터페이스 |
|---|---|---|---|
| `firebase_bridge` | `firebase_bridge` | Firebase 주문 감지 → 레시피 발행, 로봇 상태/에러 로그를 Firebase에 기록 | pub `/recipe`(String), sub `/cooking_status`(String), Firebase RTDB |
| `recipe_parser` | `recipe_orchestrator` | 원시 레시피를 **실행 가능한 평탄 step 리스트**로 정규화 (위치 이름→좌표 치환, step 번호 부여) | sub `/recipe`, pub `/parsed_recipe` |
| `state_manager` | `state_manager` | 전체 **상태머신/오케스트레이션**. step 시퀀스를 executer에 액션으로 전달, 실패 시 재시도·리셋·관리자개입 처리 | sub `/parsed_recipe`, action client `execute_recipe`, srv `unlock_system`(Trigger), pub `/cooking_status` |
| `executer` | `recipe_executer` | 액션 서버. step마다 `ActionManager.perform()` 호출로 **실제 로봇 구동**, 단계별 피드백 발행 | action server `execute_recipe`(recipe_msgs/Recipe), DSR_ROBOT2(m0609) |
| `recipe_tester` | `recipe_tester` | 개발용. Firebase 없이 로컬 JSON을 `/recipe`로 1회 발행 | pub `/recipe` |

- **액션 인터페이스** `recipe_msgs/action/Recipe`: Goal `recipe_sequence`(JSON 문자열) / Result `success,message` / Feedback `current_step,current_action`.
- **상태머신**(state_manager): `IDLE → EXECUTING →` 실패 시 `RECOVERING_RETRY`(최대 2회) `→ RECOVERING_RESET → RECOVERING_FAIL`. `unlock_system` 서비스로 관리자가 IDLE 복귀.

### 2.2 동작 플러그인 엔진 (`core/`)

레시피의 `action` 문자열 하나 = 동작 클래스 하나. 플러그인 구조.

- **`base_action.py / BaseAction`**: 모든 동작의 공통 베이스. `movel/movej/periodic/gripper_*/compliance_*/set_desired_force/reset/clear_alarm/stop` 등 **저수준 로봇 래퍼** 제공. 서브클래스는 `execute(**kwargs)`만 구현.
- **`action_manager.py / ActionManager`**: `core/actions/` 하위 모듈을 자동 import → `BaseAction` 서브클래스를 `action_name` 키로 등록. `perform(name, **params)`로 호출, 예외/False 시 `handle_critical_error`(compliance off + stop) 후 에러 플래그.

| 동작(verb) | 로봇 모션 요약 |
|---|---|
| `approach` | 안전 높이로 이동 후 툴 Z로 하강 |
| `pick` / `place` | 위치 이동 → 하강 → 그리퍼 닫기/열기 → 상승 |
| `pour` | approach 후 손목 Ry 기울여 붓기 → 복귀 |
| `flip` | 관절 시퀀스로 팬/스패출러 뒤집기 (J6 180° 롤 포함) |
| `stir` | 냄비 진입 + compliance + 하향력 + 원형 주기운동(periodic) |
| `press` | compliance로 Z 하향력 유지(누르기) |
| `push` | 하강→상승 (그리퍼 없이 누름/찌름) |
| `shake` | 그리퍼 닫고 J6 롤 + 주기 진동 |
| `spread` | compliance 하향력 + 원형 주기운동(펴 바르기) |
| `squeeze` | 손목 기울여 튜브/병 짜기 |
| `open_cap` / `close_cap` | compliance로 뚜껑 회전 열기/닫기 |

### 2.3 레시피 JSON 스키마 (`data/`)

최상위 키 = 메뉴명(`RAMEN`, `STEAK`...). 메뉴 객체:

- `locations`: `{이름: [x,y,z,rx,ry,rz]}` — 명명된 6축 포즈(mm/deg, base 좌표).
- `order_count`: 주문 카운터 (firebase_bridge가 증가 감지 트리거).
- `sequence`: step 리스트. 각 step = `action`(동사) + `params`(`pos`는 location 이름→좌표 치환) + `desc`(한글 라벨).

데이터 파일: `ramen.json`(라면 14스텝), `steak.json`(스테이크 16스텝), `full_steak.json`(긴 변형), `test*.json`(테스트).

---

## 3. `realtime_scan` — 로봇 텔레메트리 & 안전 브리지 (메인 PC)

`DSR_ROBOT2`를 import하지 않고 ROS 토픽/서비스만으로 동작(노드 충돌 회피). 노드 3개.

| 노드 | 노드명 | 담당 기능 | 주요 인터페이스 |
|---|---|---|---|
| `coord_service` | `robo_chef_coord_service` | 로봇 관절상태 캐시 → ROS/Firebase 미러링, Firebase 안전명령을 DSR 컨트롤러로 전달 | sub `/dsr01/joint_states`, srv `/robo_chef/get_coords`(Trigger), pub `/robo_chef/coords`(String,2Hz), dsr_msgs2 시스템/모션 서비스 클라이언트, Firebase `telemetry/*`,`robot_state`,`commands/robot` |
| `order_receiver` | `robo_chef_order_receiver` | (진단용) 주문/레시피 JSON 수신해 콘솔 출력만. 다운스트림 없음 | sub `/robo_chef/order_request`(String) |
| `log_bridge` | `dsr_log_bridge` | 모든 `/rosout` 로그(INFO+)를 배치로 Firebase에 전송 → 관리자 콘솔용 | sub `/rosout`(rcl_interfaces/Log), Firebase `dsr_log`(최근 300개 유지) |

- 안전명령 흐름: 웹 관리자 → Firebase `commands/robot` → coord_service → `/dsr01/system/*`,`/dsr01/motion/*` 서비스 → `commands/robot_ack`로 결과 회신.
- 두 run 스크립트 모두 `ROS_DOMAIN_ID=24` 고정.

---

## 4. `web` (sub1_side) — 키오스크 / 대시보드 / Firebase 브리지

**Flask 백엔드 + 정적 HTML 패널 3종.** Firebase RTDB(`robochef-5d9b6`)를 중심으로 동작. `ROS_DOMAIN_ID=25`.

### 4.1 백엔드 모듈

| 모듈 | 담당 기능 |
|---|---|
| `app.py` | Flask REST API. `/api/recipes`,`/api/orders`,`/api/robot/{state,coords,stop,home,mode}`,`/api/logs`,`/api/ros2/*` 등. Firebase ↔ ROS2 브리지 |
| `config.py` | 상수 (Firebase URL, 토픽명, 도메인 ID 25 등) |
| `firebase_client.py` | Firebase Admin SDK 초기화 + `/recipes`,`/orders`,`/robot_state`,`/logs` 헬퍼, 로그 200개 prune |
| `ros2_order_publisher.py` | **실주문 파이프라인**. Firebase `/orders` 폴링 → pending 주문을 ROS2로 발행 |
| `ros2_coord_client.py` | 상시 rclpy 노드. `/robo_chef/coords` 구독 캐시 → `/api/robot/coords` 응답 |
| `robot_bridge.py` | `ros2` CLI subprocess로 m0609 모션/모드 서비스 호출, 0.5s 주기 `/robot_state` 갱신 |
| `fk_m0609.py` / `fk_worker.py` | m0609 순기구학(URDF 기반 4×4 변환) / 2Hz로 관절→TCP 좌표 계산해 `/robot_state.tcp_position` 기록 |
| `recipe_seeder.py` | 초기 레시피 시드(1회용) |
| `ros2_pub_once.py` / `ros2_srv_call.py` | 토픽 1회 발행 / Trigger 서비스 호출 CLI 헬퍼 |

### 4.2 패널 3종 (정적, http.server)

| 패널 | 포트 | 역할 |
|---|---|---|
| `kiosk` | 3001 | 손님 주문 화면. `/recipes` 읽고 `/orders` 적재 + `order_count` 증가 |
| `customer_status` | 3002 | 공개 조리 현황판. 최신 주문/로봇상태 표시 |
| `admin_monitor` | 3003 | 운영자 대시보드. `/robot_state`,`/logs`,`/dsr_log` 표시, 안전명령·좌표저장 |

`start_all.sh`: 패널 3개 + Flask + fk_worker + ros2_order_publisher + cloudflared 터널 일괄 기동.

---

## 5. `ros2_move_recoder` (common) — 동작 티칭 도구

m0609 **수동 핸드가이딩 동작을 녹화 → 스무딩 → 등속 재생**하는 도구. 레시피 좌표/모션을 사람이 직접 잡아 만들 때 사용. (조리 런타임 경로와는 분리된 보조 도구)

| 모듈 | 담당 기능 |
|---|---|
| `recorder.py` | `macro_recorder` 노드. `/dsr01/joint_states`(BEST_EFFORT) 고속 수집 → `raw.json` |
| `smoother.py` | Savitzky-Golay + 정지구간 압축 + 변곡점 + 호길이 균등 다운샘플(≤100점) → `smooth.json` |
| `player.py` | `macro_player` 노드. `amovesj`로 등속 재생 (AUTONOMOUS 필요) |
| `gui.py` | PyQt5 통합 GUI. bringup 관리 + 녹화/재생 + DualSense + 그리퍼 + 모드전환 |
| `dualsense_worker.py` | PS5 듀얼센스 60Hz 폴링 → 조그/매크로 제어 |
| `gripper_worker.py` / `onrobot.py` | OnRobot RG2/RG6 그리퍼 Modbus TCP 제어 (단일 스레드 직렬화) |
| `run.py` | 단독 모션 스모크 테스트 |

- 파이프라인: `raw.json`(timestamps_ms, joints_deg, ~100Hz) → smooth → `smooth.json`(waypoints_deg, vel/acc, gripper_events).
- 듀얼센스: Create=녹화토글, ○=스무딩+재생, ×홀드=비상정지, △=홈, □=그리퍼토글, Options=관절/TCP조그 전환, L2/R2=속도, L3+R3=MANUAL/AUTO 전환.

---

## 6. 핵심 통신 채널 요약

| 채널 | 타입 | 송신 | 수신 |
|---|---|---|---|
| `/recipe` | topic String | firebase_bridge / recipe_tester | recipe_parser |
| `/parsed_recipe` | topic String | recipe_parser | state_manager |
| `/cooking_status` | topic String | state_manager | firebase_bridge |
| `execute_recipe` | action Recipe | state_manager(client) | executer(server) |
| `unlock_system` | service Trigger | 웹/관리자 | state_manager |
| `/dsr01/joint_states` | topic JointState | dsr_controller2 | coord_service, robot_bridge, recoder |
| `/robo_chef/coords` | topic String | coord_service | web backend(ros2_coord_client) |
| `/robo_chef/get_coords` | service Trigger | (요청자) | coord_service |
| `/robo_chef/order_request` | topic String | web(ros2_order_publisher) | order_receiver |
| `/rosout` | topic Log | 전 노드 | log_bridge |
| Firebase RTDB | 클라우드 | web/firebase_bridge/coord_service/log_bridge | 패널 3종 / web backend |

---

## 7. 재구조화 시 참고할 정리 포인트

> 폴더 재배치 진행 중이라 경로 문제는 제외하되, **로직상 모호/중복** 지점만 기록.

1. **robo_chef 코드 중복**: 최상위 `core/ nodes/ data/` 와 `src/robo_chef/{core,nodes,data}` 가 바이트 단위로 완전 동일(중첩본은 test/만 추가). 정본 하나만 남길지 결정 필요.
2. **`stop` 동작 미등록**: `ActionManager`가 `stop`을 `_action_map`에 등록하지 않음 → 에러/취소 경로의 정지가 사실상 무동작. (안전 관련 → 우선 검토)
3. **`state_manager.try_reset()` 이중 goal 발송**: 리셋 시퀀스를 두 번 보냄.
4. **웹 패널 키 불일치**: kiosk/customer_status는 Firebase `/robot_status`를 구독하나, 백엔드는 전부 `/robot_state`에 기록 → 해당 패널의 로봇상태 표시가 죽어 있음. (admin_monitor만 정상)
5. **ros2_order_publisher 확인 단계 비활성**: `/robo_chef/place_order` 서비스 호출이 전부 주석 처리되어, 메인노드 확인 없이 모든 주문을 즉시 `delivered` 처리. docstring과 불일치.
6. **죽은/레거시 코드**: app.py의 `_start_order_watcher/_process_order` (startup 미호출), `ros2_srv_call.py` (참조 없음).
7. **설정 분산/하드코딩**: Firebase 인증 경로(`/home/kibeom/...`), DB URL, `ROS_DOMAIN_ID`(24 vs 25)가 여러 곳에 하드코딩·중복.
8. **package.xml 런타임 의존성 누락**: robo_chef가 rclpy/std_msgs/recipe_msgs/firebase_admin 등을 선언하지 않음. setup.py 엔트리포인트 오타(`firbase_bridge`).
9. **레시피 데이터 결함**: `steak.json`/`full_steak.json`이 `locations`에 없는 위치명(`LAST_PLATING_WTH_GRIP`, `FLIPPING_POINT`)을 step에서 참조 → 파서가 미해결로 통과시켜 실행 시 실패 가능.

---

*분석 범위: cobot1 내 robo_chef·realtime_scan·web·ros2_move_recoder 전 파일 (2026-05-16 기준).*
