# realtime_scan

ROS 2 Humble (`ament_python`) 패키지. Doosan m0609 의 관절 상태를 실시간으로 수집해 FK(순기구학) 좌표를 계산하고, Firebase RTDB 에 텔레메트리를 기록한다. 로봇 안전 제어(`SafetyBridge`)와 주문 수신 미러(`order_receiver`) 도 포함한다.

---

## 노드 구성

| 노드명 (진입점) | 파일 | 역할 |
|---|---|---|
| `coord_service` | `realtime_scan/coord_service.py` | 관절 상태 수집 → FK 계산 → Firebase 텔레메트리 기록 + `SafetyBridge` 내장 |
| `log_bridge` | `realtime_scan/log_bridge.py` | `/rosout` 로그 → Firebase `/dsr_log` 미러 |
| `order_receiver` | `realtime_scan/order_receiver.py` | Firebase `/orders` 수신 (실험용 미러, 운영에선 미사용) |

---

## 인터페이스

### 구독 토픽

| 이름 | 타입 | 설명 |
|---|---|---|
| `/dsr01/joint_states` | `sensor_msgs/JointState` | 관절 각도(rad)·속도·토크, 6축. Doosan dsr_controller2 발행. |

### 발행 토픽

| 이름 | 타입 | 주기 | 설명 |
|---|---|---|---|
| `/robo_chef/coords` | `std_msgs/String` | 2 Hz | `{joint:[j1..j6 deg], velocity:[...], effort:[...]}` JSON |

### 서비스

| 이름 | 타입 | 설명 |
|---|---|---|
| `/robo_chef/get_coords` | `std_srvs/Trigger` | 현재 관절·TCP 좌표 일회성 조회. Flask 백엔드(`GET /api/robot/coords`)가 호출. |

### DSR 서비스 클라이언트 (SafetyBridge → dsr_controller2)

| 이름 | 타입 | 설명 |
|---|---|---|
| `/{ns}/motion/move_pause` | `dsr_msgs2/MovePause` | 모션 일시정지 |
| `/{ns}/motion/move_resume` | `dsr_msgs2/MoveResume` | 모션 재개 |
| `/{ns}/motion/stop` | `dsr_msgs2/MoveStop` | 모션 정지 (mode=2 slow stop) |
| `/{ns}/system/set_robot_control` | `dsr_msgs2/SetRobotControl` | 제어 모드 전환 |
| `/{ns}/system/servo_off` | `dsr_msgs2/ServoOff` | 서보 Off |
| `/{ns}/system/get_robot_state` | `dsr_msgs2/GetRobotState` | 로봇 상태 코드 조회 (2 Hz 폴링) |

`{ns}` 기본값: `dsr01` (`config.ROS2_NAMESPACE`)

---

## Firebase RTDB 쓰기 경로

| 경로 | 주기 | 데이터 |
|---|---|---|
| `/telemetry/robot_status` | 5 Hz | `{joint, velocity, effort, model, source, ingested_at}` |
| `/robot_state` | 2 Hz (SafetyBridge) | `{mode, joint_positions, tcp_position, last_updated}` |
| `/dsr_log/<id>` | 이벤트 (log_bridge) | `/rosout` 로그 항목 |
| `/commands/robot_ack` | 이벤트 (SafetyBridge) | 명령 실행 결과 `{cmd, ok, detail, elapsed_sec, processed_at}` |

### Firebase 명령 수신 경로

| 경로 | 방향 | 명령 목록 |
|---|---|---|
| `/commands/robot` | 읽기 (SafetyBridge listen) | `pause`, `resume`, `stop`, `reset_safe_stop`, `servo_on`, `servo_off`, `servo_off_quick`, `recovery_enter_safe_stop`, `recovery_enter_safe_off`, `recovery_exit` |

---

## 로봇 상태 코드 (DSR get_robot_state)

| 코드 | 이름 | 의미 |
|---|---|---|
| 0 | `INITIALIZING` | 초기화 중 |
| 1 | `STANDBY` | 대기 (정상) |
| 2 | `MOVING` | 모션 실행 중 |
| 3 | `SAFE_OFF` | 서보 Off (안전) |
| 4 | `TEACH` | 티치 모드 |
| 5 | `SAFE_STOP` | 안전 정지 |
| 6 | `EMERGENCY_STOP` | 비상 정지 |
| 7 | `HOMMING` | 홈 복귀 중 |
| 8 | `RECOVERY` | 복구 중 |
| 9 | `SAFE_STOP2` | 안전 정지 2 |
| 10 | `SAFE_OFF2` | 서보 Off 2 |

---

## 실행

```bash
source ~/cobot_ws/install/setup.bash
ros2 run realtime_scan coord_service
```

`dsr_controller2` 가 먼저 실행 중이어야 `/dsr01/joint_states` 를 수신할 수 있다.  
`coord_service` 는 `robo_chef` 노드와 독립적으로 기동 가능하다.

---

## 설치 스크립트

```bash
# 의존성 자동 설치 (ROS 2 humble + Python 패키지)
bash main_side/realtime_scan/install.sh
```

---

## 의존성

| 패키지 | 용도 |
|---|---|
| `firebase-admin` | Firebase RTDB 텔레메트리 기록 |
| `sensor_msgs` | `JointState` 구독 |
| `dsr_msgs2` | Doosan 서비스 타입 |
| `std_srvs`, `std_msgs` | ROS 2 기본 타입 |

전체 세팅은 [`docs/setup.md`](../../docs/setup.md) 참고.
