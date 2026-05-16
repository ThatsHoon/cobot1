# Architecture — 데이터 플로우 / 스레드 / ROS 2 매핑

## 전체 그림

```
┌──────────────────────────────────────────────────────────────────────┐
│                           PC (외부 호스트)                            │
│                                                                       │
│  ┌──────────────────────────┐   subprocess.Popen                      │
│  │  ros2_move_recoder GUI (PyQt5)   │ ───────────────► dsr_bringup2 launch   │
│  │   - MainWindow (Qt thr)  │                  ├─ dsr_control2 노드   │
│  │   - RosSpinThread        │   ROS 2 그래프    ├─ dsr_hardware2     │
│  │   - DsrWorker (Qt thr)   │ ◄────────────►   ├─ robot_state_pub    │
│  │   - BringupManager       │                  └─ rviz2              │
│  └──────────────────────────┘                                         │
│         ▲              │                                              │
│         │ JointState   │ Service / Topic                              │
│         │              ▼                                              │
│  topic /dsr01/joint_states          service /dsr01/motion/move_*     │
│  (sensor_msgs/JointState)           (dsr_msgs2/srv/...)               │
│                                                                       │
└──────────────────────────────────────────────────────────────────────┘
                              │
                              ▼ TCP 12345 (DRFL)
                         ┌────────────┐
                         │ Doosan     │
                         │ Controller │  (real: 192.168.1.100, virtual: 127.0.0.1)
                         │  + DRCF    │
                         └────────────┘
```

## 4계층 데이터 플로우

```
   [티칭펜던트 손 조작]
          │
          ▼
   ┌──────────────────┐
   │ /dsr01/joint_states │ ← dsr_control2가 1kHz로 컨트롤러에서 받아 보통 60-100Hz로 publish
   └──────────────────┘
          │ subscribe (RELIABLE, depth=10)
          ▼
   ┌──────────────────┐
   │ recorder / GUI   │ raw.json: timestamps_ms[], joints_deg[N][6]
   └──────────────────┘
          │
          ▼
   ┌──────────────────┐
   │ smoother         │ Savgol → 정지 제거 → 전환점 추출 → arc-length 다운샘플
   │                  │ smooth.json: waypoints_deg[K][6], vel, acc  (K ≤ 100)
   └──────────────────┘
          │
          ▼
   ┌──────────────────┐
   │ player / GUI     │ AUTONOMOUS 전환 → movesj(pts, vel, acc)
   └──────────────────┘
          │ DSR service /dsr01/motion/move_spline_joint
          ▼
       [로봇 동작]
```

## 노드 / 토픽 / 서비스 매핑

### Subscribe (입력)

| 토픽 | 타입 | 용도 | QoS |
|---|---|---|---|
| `/dsr01/joint_states` | `sensor_msgs/JointState` | 관절 상태 기록 — `joint_1`..`joint_6` 라디안 → degree 변환 후 저장 | RELIABLE, KEEP_LAST 10 |

### Service Client (출력)

DSR_ROBOT2 래퍼가 내부적으로 호출. 모두 `/dsr01/` 네임스페이스.

| 함수 | 서비스 경로 | 용도 |
|---|---|---|
| `get_robot_mode()` | `system/get_robot_mode` | MANUAL/AUTONOMOUS 폴링 (3초 주기) |
| `set_robot_mode(MODE)` | `system/set_robot_mode` | 재생 직전 AUTONOMOUS 전환 |
| `movej(posj, vel, acc)` | `motion/move_joint` | 홈 복귀 |
| `movesj([posj], vel, acc)` | `motion/move_spline_joint` | 매크로 재생 |
| `stop(DR_SSTOP)` | `motion/move_stop` | E-STOP |

### 자동 의존 (시스템)

| 토픽 | 출처 | 용도 |
|---|---|---|
| `/rosout` | 모든 노드 | 로그 수집 (rqt_console로 모니터링) |
| `/parameter_events` | 모든 노드 | 파라미터 변경 추적 |
| `/dsr01/dsr_robot2_msg` | dsr_control2 | 컨트롤러 알람/로그 (rosout과 별개) |
| `/tf`, `/tf_static` | robot_state_publisher | RViz 시각화 |

## 스레드 모델 (gui.py)

```
┌───────────────────────────────────────────────────────────────┐
│                   Main Thread (Qt event loop)                 │
│                                                                │
│   QApplication.exec_()                                        │
│     ├─ MainWindow 위젯 이벤트                                  │
│     ├─ Timer: mode_timer (3s) → request_mode.emit()           │
│     └─ Timer: hz_timer (1s) → /joint_states 주기 표시         │
└───────────────┬───────────────────────────────────────────────┘
                │ pyqtSignal (Qt cross-thread)
                ▼
┌───────────────────────────────────────────────────────────────┐
│            DsrWorker Thread (QThread, worker_thread)          │
│                                                                │
│   - DSR_ROBOT2 호출 전담 (모두 블로킹 함수)                     │
│   - _busy 플래그로 동시 호출 방지                              │
│   - 슬롯: query_mode / go_home / play / emergency_stop        │
│   - 시그널: log / play_started / play_finished / mode_updated │
└───────────────┬───────────────────────────────────────────────┘
                │ ROS service call
                ▼
┌───────────────────────────────────────────────────────────────┐
│         RosSpinThread (QThread) — ROS 2 콜백 처리             │
│                                                                │
│   - rclpy.init()                                              │
│   - MacroNode 생성 → DR_init.__dsr__node 등록                  │
│   - MultiThreadedExecutor (4 threads)                         │
│   - rclpy.__executor 로 등록 (★ DSR 호환성 핵심)               │
│   - spin_once 루프                                            │
│   - /dsr01/joint_states 콜백 → joint_received.emit()          │
└───────────────────────────────────────────────────────────────┘
                │ Qt cross-thread signal
                ▼
        Main thread의 _on_joint() 슬롯 → UI 업데이트
```

### 왜 3개 스레드인가

| 스레드 | 이유 |
|---|---|
| Main (Qt) | Qt 위젯은 메인 스레드에서만 안전하게 그릴 수 있다. |
| RosSpinThread | `rclpy.spin()`은 블로킹. Qt 이벤트 루프와 동시에 못 돌린다. |
| DsrWorker | `movesj()`는 수십초 블로킹. RosSpinThread에 두면 콜백 멈춤 → `/joint_states` 끊김 → executor가 service response를 못 받음 → 데드락. |

### Qt 시그널/슬롯 그래프

```
MainWindow                          DsrWorker
   │                                   │
   ├── request_mode  ───────────────►  query_mode()
   ├── request_home  ───────────────►  go_home()
   ├── request_play  ───────────────►  play(path, vel, acc)
   ├── request_estop ───────────────►  emergency_stop()
   │                                   │
   │  ◄────────────── log              │
   │  ◄────────────── play_started     │
   │  ◄────────────── play_finished    │
   │  ◄────────────── mode_updated     │

RosSpinThread                       MainWindow
   │                                   │
   │  ─── joint_received ────────────►  _on_joint(joints)
   │  ─── ready ─────────────────────►  _on_ros_ready()
```

## 프로세스 구성

```
[GUI 프로세스]                      [bringup 프로세스 (subprocess)]
─────────────                       ────────────────────────────────
ros2_move_recoder.gui                        ros2 launch dsr_bringup2 ...
  ├─ Qt event loop                     ├─ dsr_control_node
  ├─ rclpy node (macro_gui_node)       ├─ dsr_hardware2 (HW interface)
  ├─ DSR_ROBOT2 client                 ├─ controller_manager
  └─ subprocess.Popen ──────────►      ├─ joint_state_broadcaster
                                       ├─ robot_state_publisher
                                       └─ rviz2
```

GUI 종료 시 `BringupManager.shutdown()`이 process group에 SIGINT 전달.
SIGINT 5초 timeout 후 SIGKILL.

## 두산 ROS 2 함정과의 매핑

| 함정 (`doosan-robotics` 스킬) | ros2_move_recoder 대응 |
|---|---|
| #1 dsr_controller2는 service-driven, JTC 거동 X | `movesj` service 직접 호출 (controller bypass) |
| #3 mode는 부팅 시 결정, 동적 전환 X | `BringupDialog`로 시작 시 real/virtual 선택 |
| #5 DRFL 콜백 50ms 안에 반환 | 모션은 `DsrWorker` 스레드에서만 호출 |
| #6 multi-robot 미지원 | `dsr01` 하드코딩 |
| RT 채널 real 모드 전용 | RT 미사용 (servoj/servol 안 씀) |
| name mangling `__dsr__node` | `_dr_set_node()` 모듈 헬퍼 사용 (gui.py:49) |
| global executor 호환성 | `rclpy.__executor` 로 우리 executor 등록 (gui.py:430) |
