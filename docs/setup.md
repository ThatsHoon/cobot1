# cobot1 초기 세팅 가이드

> 마지막 검증: 2026-05-31 · ROS 2 Humble · Ubuntu 22.04

---

## 전체 구성

| Side | 역할 | 핵심 패키지 |
|---|---|---|
| `main_side` (메인 PC) | 로봇 팔 연결, 조리 실행, 텔레메트리 | `robo_chef`, `realtime_scan` |
| `sub1_side` (서브 PC) | 키오스크·대시보드 웹 서버 | `sub1_side/web` (Flask + HTML 패널) |
| `common` (공용) | 동작 녹화/재생 엔진 | `ros2_move_recoder` |

**ROS_DOMAIN_ID=24** 로 양쪽 PC가 동일하게 고정된다. 진입점 스크립트가 강제 적용하므로 `~/.bashrc` 값은 덮어써진다.

---

## 1. 공통 전제

### ROS 2 Humble 설치

```bash
# ROS 2 Humble 설치 (공식 가이드 기준)
sudo apt update && sudo apt install ros-humble-desktop -y
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### colcon 빌드 도구

```bash
sudo apt install python3-colcon-common-extensions -y
```

### 워크스페이스 클론

```bash
mkdir -p ~/cobot_ws/src && cd ~/cobot_ws/src
git clone https://github.com/ThatsHoon/cobot1.git cobot1
```

---

## 2. 메인 PC 세팅 (main_side)

### 2-1. Doosan ROS 2 패키지 설치

```bash
cd ~/cobot_ws/src
git clone https://github.com/doosan-robotics/doosan-robot2.git

# 의존성 설치
cd ~/cobot_ws
rosdep install --from-paths src --ignore-src -r -y
```

> **주의:** `dsr_controller2` 가 실행 중이어야 `robo_chef` 노드가 DSR API를 사용할 수 있다.

### 2-2. Python 의존성

```bash
pip install firebase-admin pyserial
```

### 2-3. Firebase 자격증명

Firebase 서비스 계정 키 파일을 아래 경로 중 하나에 둔다 (우선순위 순):

```
1. $FIREBASE_CRED_PATH  (환경변수 직접 지정)
2. ~/.config/cobot1/firebase-key.json
3. ~/cobot_ws/src/cobot1/main_side/robo_chef/config/serviceAccountKey.json
```

```bash
mkdir -p ~/.config/cobot1
cp /path/to/serviceAccountKey.json ~/.config/cobot1/firebase-key.json
```

> `.gitignore`에 `*serviceAccount*.json`, `*.key` 가 등록되어 있으므로 **절대 커밋하지 말 것.**

### 2-4. 그리퍼 설정 (OnRobot RG2)

기본값(`192.168.1.1:502`)이 맞지 않으면 환경변수로 오버라이드:

```bash
export GRIPPER_IP=192.168.1.1
export GRIPPER_PORT=502
export GRIPPER_TYPE=rg2   # rg2 | rg6
```

`~/.bashrc`에 추가해두는 것을 권장한다.

### 2-5. 빌드

```bash
cd ~/cobot_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 3. 서브 PC 세팅 (sub1_side)

### 3-1. Python 의존성

```bash
cd ~/cobot_ws/src/cobot1/sub1_side/web/backend
pip install -r requirements.txt
# requirements.txt: firebase-admin, flask, flask-cors
```

### 3-2. Firebase 자격증명

메인 PC와 동일한 방식으로 설치한다 (`firebase_client.py` 가 동일 경로 탐색).

### 3-3. Cloudflare Tunnel (선택)

`start_all.sh`가 `cloudflared`로 외부 접속 터널을 열기 때문에 사전에 로그인이 필요하다:

```bash
cloudflared tunnel login
cloudflared tunnel create cobot1
# ~/.cloudflared/config.yml 에 Named tunnel 설정 추가
```

터널이 필요 없으면 `start_all.sh` 내 `cloudflared` 관련 줄을 주석 처리한다.

---

## 4. 환경변수 전체 목록

| 변수 | 기본값 | 필수 | 용도 |
|---|---|---|---|
| `FIREBASE_CRED_PATH` | (자동 탐색) | 선택 | Firebase 키 파일 경로 직접 지정 |
| `FIREBASE_DB_URL` | `https://robochef-5d9b6-default-rtdb.asia-southeast1.firebasedatabase.app` | 선택 | Firebase RTDB URL |
| `ROS_DOMAIN_ID` | `24` | **필수** | ROS 2 DDS 도메인 (main·sub 동일해야 함) |
| `GRIPPER_IP` | `192.168.1.1` | 선택 | OnRobot 그리퍼 IP |
| `GRIPPER_PORT` | `502` | 선택 | OnRobot Modbus TCP 포트 |
| `GRIPPER_TYPE` | `rg2` | 선택 | 그리퍼 모델 (`rg2` \| `rg6`) |
| `FLASK_HOST` | `0.0.0.0` | 선택 | Flask 바인드 주소 (config.py) |
| `FLASK_PORT` | `5000` | 선택 | Flask 포트 |
| `FLASK_DEBUG` | `False` | 선택 | Flask 디버그 모드 |
| `ROS2_NAMESPACE` | `dsr01` | 선택 | Doosan 로봇 ROS 2 네임스페이스 |
| `STATE_UPDATE_INTERVAL` | `0.5` | 선택 | 로봇 상태 업데이트 주기 (초) |
| `MAX_LOG_ENTRIES` | `200` | 선택 | Firebase에 유지할 최대 로그 수 |

> `FLASK_*` 등 설정파일 변수는 `sub1_side/web/backend/config.py`에서 관리된다.

---

## 5. 실행 순서

### 메인 PC

```bash
source ~/cobot_ws/install/setup.bash

# 터미널 1: firebase_bridge (주문 감시·파이프라인 진입점)
ros2 run robo_chef firebase_bridge

# 터미널 2: sequence_runner (조리 실행, DSR 초기화 포함)
ros2 run robo_chef sequence_runner

# 터미널 3: coord_service (텔레메트리 + 안전 브리지)
ros2 run realtime_scan coord_service
```

`firebase_bridge` → `sequence_runner` 순서로 기동해야 `/recipe` 토픽 구독이 연결된다. `coord_service`는 독립적으로 언제든 기동 가능하다.

### 서브 PC

```bash
cd ~/cobot_ws/src/cobot1/sub1_side/web
bash start_all.sh
```

자동으로 아래 프로세스가 tmux 세션에서 시작된다:

| 서비스 | 포트 | tmux 세션 | 로그 |
|---|---|---|---|
| Kiosk UI | 3001 | — | `/tmp/rc_kiosk.log` |
| Customer Status UI | 3002 | — | `/tmp/rc_customer.log` |
| Admin Monitor UI | 3003 | — | `/tmp/rc_admin.log` |
| Flask 백엔드 | 5000 | `flask` | `/tmp/rc_flask.log` |
| FK Worker | — | `fk` | `/tmp/rc_fk.log` |
| Cloudflared 터널 | — | — | `/tmp/cf_named.log` |

로그 확인:
```bash
tmux attach -t flask   # Flask
tmux attach -t fk      # FK Worker
tail -f /tmp/rc_flask.log
```

---

## 6. 레시피(세그먼트) 사전 등록

조리 실행 전에 반드시 세그먼트를 녹화하고 Firebase에 등록해야 한다.

### 세그먼트 녹화

```bash
# ros2_move_recoder GUI 실행 (메인 PC에서)
ros2 run ros2_move_recoder main
```

녹화 결과는 `~/cobot_ws/src/ros2_move_recoder/records/<seg_name>/smooth.json` 에 저장된다.

### Firebase 레시피 등록

```bash
# recipe_seeder.py 실행 또는 API로 직접 등록
curl -X POST http://localhost:5000/api/recipes \
  -H "Content-Type: application/json" \
  -d '{"recipe_id":"RAMEN","recipe_name":"라면","segments":["pour_water","add_noodles","stir","serve"]}'
```

`segments` 배열이 비어 있으면 주문 시 `RecipeNotSeededError` 가 발생한다.

---

## 7. 빠른 동작 확인

```bash
# 1. 주문 생성
curl -X POST http://localhost:5000/api/orders \
  -H "Content-Type: application/json" \
  -d '{"items":[{"recipe_id":"RAMEN","qty":1,"name":"라면","price":8000,"subtotal":8000}],"total":8000}'

# 2. 주문 상태 확인
curl http://localhost:5000/api/orders

# 3. 로봇 상태 확인
curl http://localhost:5000/api/robot/state

# 4. 헬스 체크
curl http://localhost:5000/api/health
```
