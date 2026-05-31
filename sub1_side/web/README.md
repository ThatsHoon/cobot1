# sub1_side/web

서브 PC에서 동작하는 웹 레이어 전체. Flask REST API 백엔드와 세 개의 HTML 패널(정적 SPA)로 구성된다.

---

## 디렉토리 구조

```
sub1_side/web/
├── backend/           Flask 백엔드 + 헬퍼 모듈
│   ├── app.py         라우터 진입점 (모든 /api/* 엔드포인트)
│   ├── config.py      환경변수·상수 중앙 관리
│   ├── firebase_client.py  Firebase RTDB CRUD 헬퍼
│   ├── fk_m0609.py    Doosan m0609 순기구학(FK) 수식
│   ├── fk_worker.py   FK 계산 → Firebase /robot_state 2Hz 갱신 워커
│   ├── recipe_seeder.py    Firebase 레시피 초기 데이터 시드
│   ├── robot_bridge.py     Firebase ↔ 로봇 제어 명령 브리지
│   ├── ros2_coord_client.py  ROS 2 /robo_chef/get_coords 서비스 클라이언트
│   ├── ros2_pub_once.py    ROS 2 토픽 일회성 발행 유틸리티
│   └── requirements.txt    Python 의존성
├── panel/             정적 HTML 패널 3종
│   ├── kiosk/         고객 주문 UI (포트 3001)
│   ├── customer_status/   조리 현황 UI (포트 3002)
│   ├── admin_monitor/     관리자 대시보드 (포트 3003)
│   └── firebase_config.js  공용 Firebase 설정 (패널별 복사본 있음)
└── start_all.sh       전체 서비스 일괄 기동 스크립트
```

---

## 패널 역할

| 패널 | 포트 | Firebase 경로 | 주요 기능 |
|---|---|---|---|
| **Kiosk** | 3001 | `/recipes`, `/orders`, `/robot_status` | 메뉴 선택·주문 제출. 로봇 busy 시 결제 버튼 비활성화. |
| **Customer Status** | 3002 | `/orders`, `/robot_status` | 조리 세그먼트 단위 실시간 진행 표시. 완료 알림 팝업. |
| **Admin Monitor** | 3003 | `/robot_state`, `/logs`, `/dsr_log`, `/commands/robot` | 관절 포지션·TCP 좌표·로봇 모드·로그 스트림. 원격 pause/resume/stop. |

모든 패널은 Firebase JS SDK 로 직접 RTDB 를 구독한다. Flask를 거치지 않는다.

---

## Flask 백엔드

**포트**: 5000 (기본값, `config.FLASK_PORT` 변경 가능)  
**전체 API 레퍼런스**: [`docs/api.md`](../../docs/api.md)

### 주요 모듈 역할

#### `firebase_client.py`
Firebase Admin SDK 래퍼. 모든 RTDB 읽기/쓰기를 담당한다.

주요 함수:
- `create_order(items, total)` — 주문 생성 + `/orders/<id>` 기록
- `get_orders(limit)`, `get_order(order_id)` — 주문 조회
- `update_order_status(order_id, status)` — 상태 수동 변경
- `get_robot_state()` — `/robot_state` 조회
- `log_event(level, message, source)` — `/logs` 기록

#### `fk_worker.py`
2 Hz 주기로 ROS 2 `/dsr01/joint_states` 를 수신해 FK 계산 후 Firebase `/robot_state.tcp_position` 을 갱신한다. `fk_m0609.py` 의 순기구학 수식을 사용한다.

#### `robot_bridge.py`
Firebase `/commands/robot` 에 명령을 쓰면 `coord_service.SafetyBridge` 가 실제 DSR 서비스를 호출한다. `app.py` 가 `POST /api/robot/stop` 등을 받으면 이 브리지를 경유한다.

#### `config.py`
모든 환경변수와 상수를 중앙 관리한다. 다른 모듈은 `config.VAR` 형태로 참조한다.

```python
ROS_DOMAIN_ID = 24          # app.py 시작 시 os.environ에 강제 적용
FLASK_HOST    = "0.0.0.0"
FLASK_PORT    = 5000
MAX_LOG_ENTRIES = 200       # Firebase 로그 최대 보관 수
```

---

## 실행

### 전체 일괄 기동

```bash
cd ~/cobot_ws/src/cobot1/sub1_side/web
bash start_all.sh
```

시작되는 서비스:

| 서비스 | tmux 세션 | 로그 파일 |
|---|---|---|
| Kiosk UI (3001) | — | `/tmp/rc_kiosk.log` |
| Customer Status UI (3002) | — | `/tmp/rc_customer.log` |
| Admin Monitor UI (3003) | — | `/tmp/rc_admin.log` |
| Flask 백엔드 (5000) | `flask` | `/tmp/rc_flask.log` |
| FK Worker | `fk` | `/tmp/rc_fk.log` |
| Cloudflared 터널 | — | `/tmp/cf_named.log` |

로그 확인:
```bash
tmux attach -t flask
tail -f /tmp/rc_flask.log
```

### Flask만 단독 실행 (개발)

```bash
cd backend
python app.py
```

---

## 의존성 설치

```bash
pip install -r backend/requirements.txt
# firebase-admin, flask, flask-cors
```

---

## Firebase 설정

패널별 `firebase_config.js` 는 모두 동일한 Firebase 프로젝트를 가리킨다.  
`panel/firebase_config.js` 가 마스터 파일이며, `start_all.sh` 가 각 패널 디렉토리에 복사한다.  
변경 시 마스터 파일만 수정하면 된다.

---

## 환경변수

전체 목록은 [`docs/setup.md`](../../docs/setup.md) 참고.

핵심 변수:
- `FIREBASE_CRED_PATH` — Firebase 키 파일 경로 (미설정 시 자동 탐색)
- `FIREBASE_DB_URL` — Firebase RTDB URL
- `ROS_DOMAIN_ID=24` — `app.py` 시작 시 강제 주입 (`config.py:36`)
