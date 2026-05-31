# cobot1 — 발표 outline (20장)

> 인사담당자에게 1회성으로 보여주는 자료. 슬라이드별 talking points 와
> 시각요소 키워드를 정리. 톤: 비기술 친화 스토리형. 화자: 전체 설계자.

---

## Slide 01 · 표지

**제목 / 한 줄 카피**
- 큰 제목: **"라면을 끓이는 협동로봇, cobot1"**
- 부제: 키오스크에서 주문하면 m0609 가 조리하고, 손님은 진행 상황을 화면으로 본다
- 발표자명 · 발표일 · 한 줄 역할 ("전체 시스템 설계 및 구현 — ThatsHoon")

**시각요소 키워드**
- m0609 협동로봇 + 라면 그릇 일러스트 / 사진
- 하단에 작은 아이콘 라인 (📱 키오스크 · 🤖 로봇 · 🔥 인덕션 · 📺 현황판)

**왜 이 슬라이드**
- 처음 0.5 초 안에 "이 사람은 뭘 만들었나" 가 전달돼야 함.

---

## Slide 02 · 한 줄 요약 · 데모 시나리오

**한 줄 요약**
> "주문에서 음식까지, 사람의 손이 닿지 않는 30 초."

**데모 시나리오 (그림 4 컷)**
1. 손님이 키오스크에서 "라면 1, 스테이크 1" 주문
2. 메인 PC 의 firebase_bridge 가 주문을 잡아 sequence_runner 에 전달
3. m0609 가 사전 녹화된 동작 시퀀스(세그먼트 체인)를 차례로 재생
4. 손님은 옆 화면에서 진행 단계를 실시간으로 확인

**시각요소 키워드**
- 화살표로 연결된 4 컷 픽토그램 (가로 일렬, 한 줄)
- 시간축 표시 (00:00 → 00:30) 로 "체감 속도" 강조

**왜 이 슬라이드**
- 추상 단어 없이 "이 시스템이 뭘 하는지" 비기술자에게 한 번에 박힘.

---

## Slide 03 · 시스템 아키텍처 (1 페이지)

**한 줄 요약**
- 물리적으로 **3 개의 PC** 가 Firebase RTDB 를 허브로 협력한다.

**다이어그램 (한 장)**
```
┌─────────────────┐      ┌─────────────────────────┐      ┌─────────────────┐
│  sub1_side PC   │      │   Firebase RTDB         │      │  main_side PC   │
│  (웹/주문)      │ ───▶ │   (robochef-5d9b6)      │ ◀──▶ │  (로봇 팔)      │
│  Flask + 패널 3 │ ◀─── │   orders · recipes      │      │  m0609 + DSR    │
│  키오스크/현황/ │      │   robot_status/_state   │      │  sequence_runner│
│  관리자 모니터  │      │   logs · commands       │      │  coord_service  │
└─────────────────┘      └─────────────────────────┘      └─────────────────┘
       │                                                            │
       └────────────── (선택) 같은 LAN: ROS 2 토픽 직결 ──────────────┘
                         ROS_DOMAIN_ID=24
```

**핵심 라벨**
- main_side: 로봇과 직접 연결 — 조리 실행 + 텔레메트리
- sub1_side: 키오스크·대시보드·웹 노출
- common: 동작 녹화/재생 도구 (양쪽에서 import)

**왜 이 슬라이드**
- 기술/비기술 모두에게 한 장으로 "큰 그림" 제공. 다음 슬라이드들이 어디에
  속하는지 매핑하는 기준점.

---

## Slide 04 · ROS2 노드 통신 구조 (주요 기능별 묶음, 1 페이지)

**기능별 그룹 4 개**

**① 조리 코어** (robo_chef)
```
firebase_bridge ──/recipe──▶ sequence_runner
       ▲                            │
       └──/cooking_status───────────┘
```
- `/recipe` (잡 디스패치), `/cooking_status` (진행 상태)
- 서비스: `unlock_system` (ERROR → IDLE 복귀)

**② 텔레메트리 / 안전** (realtime_scan)
```
dsr_controller2 ──/dsr01/joint_states──▶ coord_service ──/robo_chef/coords──▶ web
                                                │
                                                └── Firebase /robot_state, /commands/robot
```
- 2 Hz 좌표 발행, Firebase 미러링
- SafetyBridge: Firebase commands/robot → dsr_msgs2 motion service

**③ 로그 수집** (realtime_scan)
```
모든 노드 ──/rosout──▶ log_bridge ──batch 0.5 s──▶ Firebase /dsr_log (max 300)
```

**④ 웹 브리지** (web backend)
```
Flask ──[rclpy native]──▶ /dsr01/motion/move_stop, move_joint, set_robot_mode
   └──[지속 구독]──▶ /robo_chef/coords (캐시)
```

**왜 이 슬라이드**
- 노드가 많아 보이지만 "사실은 4 가지 일만 한다" 를 보여준다.

---

## Slide 05 · 주요 기능별 기술 스택 (1 페이지)

| 영역 | 핵심 기술 | 한 줄 설명 |
|---|---|---|
| 로봇 제어 | **Doosan m0609** + DSR_ROBOT2 (Python) + DRFL | 6 축 협동로봇, ROS 2 dsr_controller2 |
| 그리퍼 | **OnRobot RG2** Modbus TCP | 단일 worker thread 로 직렬화 제어 |
| 미들웨어 | **ROS 2 Humble** · Fast DDS · `ROS_DOMAIN_ID=24` | 토픽/서비스 + DDS 격리 |
| 동작 데이터 | Savitzky-Golay 평활화 + arc-length 균등 샘플 | raw.json → smooth.json (≤ 100 점) |
| 입력 장치 | **PS5 DualSense** (pygame) | 60 Hz 폴링 → Qt 시그널 |
| GUI | **PyQt5** | 녹화/재생/조그/모드 전환 |
| 백엔드 | **Flask** + flask-cors + `rclpy` | 명령 → dsr_msgs2 service |
| 클라우드 | **Firebase RTDB** + Admin SDK | 주문/상태/로그/명령 허브 |
| 외부 노출 | **Cloudflare Tunnel** 고정 도메인 | kiosk/status/admin.thatshoon.com |
| 패널 | 정적 HTML/JS (firebase-js-sdk) | http.server 3001/3002/3003 |
| 시뮬레이션 | dsr_bringup2 mode=virtual | RViz + 시뮬 컨트롤러 |

**왜 이 슬라이드**
- "어떤 기술을 다뤘는가" 한 페이지 체크리스트. 이력서/포트폴리오 키워드와 매칭.

---

## Slide 06 · 데이터 흐름 — 한 주문이 시스템을 가로지르는 30 초

**스토리보드 (시간축 그림)**

| 시각 | 위치 | 일어나는 일 |
|---|---|---|
| t = 0 s | 키오스크 (sub PC) | 손님이 "라면 1" 주문 → Firebase `/orders/ORD_xxxx` 적재 |
| t = 0.1 s | Firebase | listener 트리거 |
| t = 0.2 s | firebase_bridge (main PC) | FIFO 에서 pending 1 건 잡고 `/orders/ORD_xxxx.status = cooking` |
| t = 0.3 s | sequence_runner | `/recipe` 수신 → state IDLE → EXECUTING |
| t = 1 ~ 25 s | playback 엔진 | segment 1, 2, …, n 차례로 amovesj 재생 + 그리퍼 이벤트 |
| 각 segment 마다 | sequence_runner | `/cooking_status` emit → Firebase `/robot_status` 갱신 → customer_status 화면 STEP 진행 |
| t = 25 s | sequence_runner | DONE emit → firebase_bridge 가 `/orders/ORD_xxxx.status = delivered` |
| t = 25.1 s | customer_status | "요리가 완료되었습니다" 표시 |

**왜 이 슬라이드**
- 5 장에서 본 큰 그림을 "1 주문 = 30 초" 라는 시간축으로 다시 한 번. 비기술자에게 가장 잘 박힘.

---

## Slide 07 · 조리 엔진 — sequence_runner + cooking_core

**핵심 아이디어**
- 사람이 손으로 시연한 동작들이 `records/<seg>/smooth.json` 으로 저장돼 있고,
  주문이 들어오면 그 세그먼트들을 **레시피 순서대로 재생** 한다.
- 동적으로 "들어 올리고 ➔ 기울이고 ➔ 붓는다" 같은 verb 파싱은 없음.
  **녹화된 동작 = 재생 단위** 라는 단순한 모델.

**3 중 루프 (한 줄로)**
```
items  →  qty  →  segment  →  amovesj + 그리퍼 이벤트
```
- 메뉴 N 개 × 각 수량 Q × 각 메뉴의 세그먼트 K 개.

**안전 가드**
- `_lock` — IDLE → EXECUTING 전이를 원자화 (동시에 두 주문이 들어와도 안전)
- `_abort` Event — 외부에서 즉시 중단 가능
- 실패 시 ERROR — `unlock_system` Trigger 로만 수동 복귀

**시각요소 키워드**
- "녹화 = 레시피" 비유 그림 (요리사가 한 번 시연 → 로봇이 평생 반복)

---

## Slide 08 · 주문 게이트키퍼 — firebase_bridge

**역할 한 줄**
- Firebase `/orders` 의 pending 주문을 **시간 순(FIFO)** 으로 1 건씩 골라
  로봇에 디스패치하고, 끝나면 다음 건으로 넘어간다.

**중요한 결정 3 가지**
1. **busy 플래그** — 동시에 2 건이 절대 디스패치되지 않도록 lock 으로 보호
2. **FIFO 정렬** — `order_time` 기준. order_count 같은 카운터 트리거 폐기
3. **상태 전이 분리** — DONE → delivered, ERROR → failed, IDLE → busy 해제

**왜 단순해졌는가**
- 이전: 카운터가 증가하면 디스패치 (race condition 위험)
- 지금: pending 만 보고 끝낼 때까지 잡고 있음 (정확한 1:1)

**시각요소 키워드**
- 주문 줄(큐) 픽토그램 — pending 4 건 중 맨 앞 1 건만 cooking, 나머지는 대기

---

## Slide 09 · 동작 녹화 → 재생 파이프라인

**3 단계 + 시각 한 줄**
1. **Record** — 사람이 m0609 를 직접 손으로 잡고 동작 시연 → `/dsr01/joint_states` 100 Hz 수집 → `raw.json`
2. **Smooth** — Savitzky-Golay 필터 + 정지구간 압축 + arc-length 균등 다운샘플 → `smooth.json` (≤ 100 점)
3. **Play** — `amovesj` 비동기 모션 + 그리퍼 이벤트 타임라인 동시 재생

**의미**
- 복잡한 동작도 **"한 번 보여주면 영원히 반복"**
- 코드를 한 줄도 안 짜고 새 레시피를 추가할 수 있다 (녹화 → 폴더 복사)

**시각요소 키워드**
- 위/아래 그래프 비교: raw (100 Hz 떨림 많음) vs smooth (100 점, 부드러움)

---

## Slide 10 · PS5 듀얼센스로 로봇 가르치기

**왜 PS5 패드?**
- 펜던트보다 가벼움, 양손 jog 자연스러움, 사람들이 직관적으로 안다
- 60 Hz 폴링 + Qt 시그널 → GUI 와 매끄럽게 연결

**버튼 매핑 (요약)**
| 버튼 | 동작 |
|---|---|
| Create (짧게/길게) | Record 시작·정지 / 새 액션 자동 생성 |
| ○ | Smooth + Play |
| × (길게) | 비상 정지 + Home 자동 복귀 |
| △ | Home 복귀 |
| □ | 그리퍼 Open/Close |
| L3 + R3 | MANUAL ↔ AUTONOMOUS 전환 |
| 좌/우 스틱 | 조인트 jog · TCP jog (Options 키로 모드 전환) |

**시각요소 키워드**
- DualSense 패드 사진 + 버튼별 콜아웃

**왜 이 슬라이드**
- "엔지니어가 아니어도 시연 가능" 이라는 사용성 강조 + 의외성 어필.

---

## Slide 11 · 그리퍼 통합 (OnRobot RG2)

**한 줄**
- Modbus TCP 로 폭(width)/속도/힘을 제어. 단일 worker thread 로 호출 직렬화 → 경합 없음.

**왜 직렬화가 필요했는가**
- Modbus 클라이언트가 thread-safe 가 아님 → 동시에 호출하면 응답 섞임
- → Queue 기반 단일 worker 패턴 (gripper_worker.py)

**녹화 통합**
- 녹화 시 그리퍼 width 변화도 timestamps 와 함께 기록
- smoother 가 가까운 변화점을 묶어 `gripper_events: [{t_norm, action}, …]` 로 추출
- 재생 시 진행률(t_norm) 도달 순간에 open/close 이벤트 발사

**시각요소 키워드**
- 그리퍼가 컵을 잡는 정지 컷 + 옆에 "width=50mm" 라벨

---

## Slide 12 · 안전 제어 경로 (이중 안전망)

**경로 1 — 관리자 비상 정지 (Firebase)**
```
admin_monitor 패널 → Firebase /commands/robot → coord_service SafetyBridge
                  → /dsr01/motion/move_stop service → /commands/robot_ack
```
- 외부 인터넷 어디서나 가능 (cloudflared 고정 도메인)
- 6 초 타임아웃, ack 회신

**경로 2 — Flask REST 직접 호출**
```
/api/robot/stop → robot_bridge (rclpy native) → MoveStop service
```
- 사내 LAN 진단/조작 용

**핵심**
- 두 경로 모두 마지막 단계는 동일 dsr_msgs2 service. 한쪽이 죽어도 다른 쪽으로 정지 가능.

**시각요소 키워드**
- 빨간 비상정지 버튼 + 두 갈래 화살표

---

## Slide 13 · 세 개의 웹 패널 — 역할 분리

**한 화면씩 그리고 한 줄로**

**① 키오스크 (3001)**
- 메뉴 카드 → 장바구니 → 결제
- 로봇이 ERROR 상태면 결제 버튼이 "로봇 점검 중" 으로 잠김
- 보호색: 빨강(차단) / 초록(주문 가능)

**② 고객 현황판 (3002)**
- 현재 주문 번호 + STEP 진행 (segment 단위로 동적 렌더)
- "STEP 03 / 12 · 면 투입 중" 같은 진행 표시
- 완료 시 "요리가 완료되었습니다" 풀스크린

**③ 관리자 모니터 (3003)**
- 로봇 텔레메트리 (관절·TCP·모드)
- 비상 정지 / 좌표 저장 / 로그 (`/logs` + `/dsr_log`)
- safety command 상태 표시

**시각요소 키워드**
- 3 화면 모형(mockup) 가로 일렬 — 색상으로 역할 구분

---

## Slide 14 · 클라우드 노출 — Cloudflare Tunnel

**문제**
- 학교/회사 공유망에서 외부 접속 IP 불가능
- ngrok 무료 플랜은 매번 URL 바뀜

**해결**
- Cloudflared **Named Tunnel** 로 고정 도메인 운영
- `kiosk.thatshoon.com`, `status.thatshoon.com`, `admin.thatshoon.com`, `api.thatshoon.com`
- 시연 자리에서 QR 만 찍어도 손님이 바로 주문 가능

**왜 의미 있나**
- "로컬 전용" 데모를 "어디서나 가능한 서비스" 로 격상

**시각요소 키워드**
- QR 코드 + 도메인 4 개 표

---

## Slide 15 · 데이터 모델 — Firebase RTDB 한 페이지

**경로 트리 (간략)**
```
/recipes/{recipe_id}              ← 메뉴 정의 (kiosk 가 읽음)
/orders/{order_id}                ← 주문 (status: pending → cooking → delivered/failed)
/robot_status                     ← sequence_runner 진행 상태 (kiosk·customer_status 가 구독)
/robot_state                      ← 하드웨어 텔레메트리 (admin_monitor 가 구독)
/telemetry/robot_status           ← coord_service raw 5 Hz (진단)
/commands/robot · /robot_ack      ← 안전 명령 + ack
/logs                             ← 비즈니스 이벤트 로그
/dsr_log                          ← /rosout 미러 (300 개 제한)
```

**설계 원칙**
- 같은 이름처럼 보이는 `/robot_status` 와 `/robot_state` 는 **서로 다른 writer 가 다른 데이터를 담는 별개 채널**
- writer 와 reader 가 명확히 분리돼 있어 디버깅 쉬움

**시각요소 키워드**
- 트리 뷰 (들여쓰기) + 각 노드에 writer/reader 라벨

---

## Slide 16 · 동시성·안전성 한 페이지

**적용된 패턴 4 가지**

| 패턴 | 위치 | 막은 문제 |
|---|---|---|
| `threading.Lock` IDLE→EXECUTING 전이 원자화 | sequence_runner | 같은 순간 두 주문이 들어와 동시에 시작되는 race |
| `threading.Event` (`_abort`) | playback | 재생 중간에 외부 정지 신호 받기 |
| MultiThreadedExecutor (num_threads=4) | sequence_runner | long-running 모션이 service 콜백을 block 하지 않게 |
| Queue 기반 단일 worker thread | gripper_worker | Modbus 클라이언트 thread-safety 결여 |

**버그 사례 (회고용)**
- ERROR → unlock → IDLE 직행인데 kiosk 는 RECOVERY 만 해제 조건으로 두어 영구 잠금이 됐던 적이 있음. IDLE/DONE 도 해제 조건에 포함하도록 수정.

**시각요소 키워드**
- 자물쇠 아이콘 4 개 + 막은 문제 한 줄씩

---

## Slide 17 · 리팩터링 여정 — 5 노드 → 2 노드

**Before (~2026-05-15)**
```
firebase_bridge → recipe_parser → state_manager → executer
                                                    │
                                            ActionManager
                                                    │
                                      11 종 동작 verb (movel/movej/pour/flip/stir/…)
```
- 5 노드 + 동적 action plugin 11 종
- 문제: state_manager 재시도 race, ActionManager 가 stop 등록 누락,
  레시피 JSON 의 location 키 미해결 등 9 개 진단

**After (현재)**
```
firebase_bridge → sequence_runner → playback.play_segment (헤드리스 코어)
```
- 2 노드 + 사전 녹화된 세그먼트 체인 재생
- 동적 verb 폐기 → 녹화 자체가 verb
- 9 개 진단 중 5 개 자연 해소

**핵심 판단**
- "복잡한 것을 더 똑똑하게 다루기" → "단순한 것으로 바꾸기" 가 더 빠르고 안전

**시각요소 키워드**
- Before/After 다이어그램 좌우 비교 + 빨간 X / 초록 ✓

---

## Slide 18 · 단일 출처 환경 변수 (ROS_DOMAIN_ID 24)

**문제**
- 호스트 `~/.bashrc` 가 다른 프로젝트(cobot3) 용으로 `ROS_DOMAIN_ID=130` 을 export
- cobot1 셸을 새로 열면 두 PC 가 서로 못 봄

**해결**
- 진입점 3 곳이 명시적으로 24 강제: `start_all.sh`, `run_coord_service.sh`, `run_receiver.sh`
- Flask `app.py` 가 rclpy import 전에 `os.environ["ROS_DOMAIN_ID"] = str(config.ROS2_DOMAIN_ID)`
- `config.py` 가 단일 출처 (single source of truth)

**부가 환경 변수**
- `FIREBASE_CRED_PATH`, `FIREBASE_DB_URL` — PC 이관 시 한 줄 override
- `GRIPPER_IP/PORT/TYPE` — 그리퍼 기종/IP 변경

**왜 인사담당자에게 의미가 있나**
- "환경 설정 실수로 안 돌아가는 코드" 의 흔한 원인을 시스템적으로 차단했음을 보여주는 사례.

---

## Slide 19 · 잔존 이슈 / 향후 계획 (정직성 슬라이드)

**현재 알고 있는 잔존 이슈 (architecture.md §7 그대로)**
1. `/robo_chef/order_request` 토픽이 leftover — 진단 도구로 라벨만 남김
2. `RECORDS_DIR` 가 절대경로 하드코딩 — `COBOT1_RECORDS_DIR` 환경변수화 고려
3. m0609 IP `192.168.1.100` 가 BringupManager 에 하드코딩
4. cooking_core 가 `RECOVERY` 같은 중간 상태를 emit 하지 않음 (자동 복구 시퀀스 도입 시 필요)
5. 통합 회귀 테스트 부족 — `ERROR → unlock → 다음 주문` 시나리오

**향후 방향**
- 자동 복구 시퀀스 (충돌 감지 후 안전 위치 후퇴 → 자동 재시도)
- 레시피 마켓플레이스 (녹화 폴더만 공유하면 다른 매장이 즉시 사용)
- vision-guided pick (현재는 위치 고정 — 카메라로 동적 위치 추적)

**왜 이 슬라이드**
- "잘 안 된 것/남은 것" 을 정직하게 보여주는 슬라이드는 신뢰도를 올린다.
  엔지니어링 판단력을 평가받는 지점.

---

## Slide 20 · 회고 · 배운 점

**한 줄로**
> "복잡한 것을 우아하게 풀기보다, 우아하게 단순화할 줄 아는 게 더 어렵다는 걸 배웠다."

**구체 항목 3 가지**
1. **추상화의 비용** — 동적 액션 verb 엔진은 멋있어 보였지만 디버깅·테스트·운영 비용이 컸음. "녹화 = 재생" 단순 모델이 같은 결과를 더 안정적으로.
2. **단일 출처의 힘** — `ROS_DOMAIN_ID`, Firebase 키 경로처럼 "어디 박혀있는지 잊기 쉬운 값" 을 한 곳에 모아두는 것만으로 PC 이관/협업 비용이 급감.
3. **두 채널이 같은 이름이면 한 채널이 죽는다** — `/robot_status` vs `/robot_state` 의 의미 분리를 문서에 명시하지 않았다면 누가 와도 통일하려 했을 것. 이름이 곧 계약.

**닫는 말**
- m0609 와 PS5 패드, 라면 한 그릇으로 시작했지만 이 프로젝트에서 가장 많이 배운 건 결국 "사람이 읽기 쉬운 시스템" 을 만드는 일이었습니다.

**시각요소 키워드**
- 본인 사진 또는 m0609 와 라면 그릇 정면 컷 + 한 줄 카피 굵게

---

## 부록 · 슬라이드 구성 한눈에

| # | 제목 | 카테고리 |
|---|---|---|
| 01 | 표지 | 도입 |
| 02 | 한 줄 요약 + 데모 시나리오 | 도입 |
| 03 | 시스템 아키텍처 | 사용자 요청 고정 슬롯 |
| 04 | ROS 2 노드 통신 구조 (기능별) | 사용자 요청 고정 슬롯 |
| 05 | 기능별 기술 스택 | 사용자 요청 고정 슬롯 |
| 06 | 1 주문 30 초 데이터 흐름 | 큰 그림 마무리 |
| 07 | 조리 엔진 (sequence_runner + cooking_core) | 기능 디테일 |
| 08 | 주문 게이트키퍼 (firebase_bridge) | 기능 디테일 |
| 09 | 녹화 → 재생 파이프라인 | 흥미요소 |
| 10 | DualSense 컨트롤러 | 흥미요소 |
| 11 | 그리퍼 통합 (OnRobot RG2) | 기능 디테일 |
| 12 | 안전 제어 경로 (이중 안전망) | 엔지니어링 가치 |
| 13 | 세 개의 웹 패널 | UX |
| 14 | Cloudflare Tunnel | 흥미요소 |
| 15 | Firebase 데이터 모델 | 기능 디테일 |
| 16 | 동시성·안전성 | 엔지니어링 가치 |
| 17 | 리팩터링 여정 (5→2 노드) | 엔지니어링 판단 |
| 18 | 단일 출처 환경 변수 | 엔지니어링 위생 |
| 19 | 잔존 이슈 / 향후 계획 | 정직성 |
| 20 | 회고 / 배운 점 | 닫음 |
