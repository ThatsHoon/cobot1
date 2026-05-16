# gui.py — PyQt5 통합 GUI 상세

`ros2_move_recoder/gui.py` (1170 lines) — bringup launch + record + smooth + play + home + estop을 단일 윈도우에서 처리.

## 윈도우 레이아웃

```
┌── menuBar ──── 파일 / Ctrl+O 폴더 열기 / Ctrl+Q 종료 ─────────────────┐
├── BringupDialog (시작 시 modal) ──────────────────────────────────────┤
│  ● Real (192.168.1.100:12345)                                         │
│  ● Virtual (127.0.0.1)                                                │
│  ○ Skip (이미 실행 중)                                                 │
└────────────────────────────────────────────────────────────────────────┘

┌── 좌 사이드바 ──┬── 우 본체 ────────────────────────────────────────┐
│ records/ 목록   │ 액션: <name>           [MODE: AUTONOMOUS] ● 상태   │
│  [name] [tags]  │ ──────────────────────────────────────────────────│
│  ...            │  ┌── 현재 좌표 ──┐  ┌── 기록/시스템 로그 ──────┐  │
│                 │  │ J1: +12.345    │  │ [hh:mm:ss] [record] ... │  │
│  새로고침        │  │ ...            │  │                         │  │
│  폴더열기        │  │ J6: +90.000    │  │                         │  │
│  삭제           │  │ 샘플 수: 1234   │  │                         │  │
│                 │  └────────────────┘  │                         │  │
│                 │  ┌── 평활화 파라미터 ──┐ │                       │  │
│                 │  │ window  [51]        │ │                       │  │
│                 │  │ poly    [3]         │ │                       │  │
│                 │  │ max_pts [80]        │ │                       │  │
│                 │  │ eps     [0.5]       │ │                       │  │
│                 │  │ prom    [2.0]       │ │                       │  │
│                 │  └─────────────────────┘ │                       │  │
│                 │  ┌── 재생 속도 ────────┐ │                       │  │
│                 │  │ vel  [60 °/s]       │ │                       │  │
│                 │  │ acc  [120 °/s²]     │ │                       │  │
│                 │  │ [⏪Slow][▶Norm][⏩Fast][⏩⏩Max]              │  │
│                 │  │ [📊 시연 속도 자동 추천]                     │  │
│                 │  │ Operation Speed [────●────] 100%             │  │
│                 │  │ 실효 vel = 60 × 100% = 60 °/s                │  │
│                 │  └─────────────────────┘ └───────────────────────┘  │
│                 │  [progress bar  v / m waypoint]                   │
│                 │  [● Record] [■ Stop] [∿ Smooth] [▶ Play] [🏠] [🛑]│
└─────────────────┴────────────────────────────────────────────────────┘
└── statusBar ── Node ✓ │ /dsr01/joint_states: 60Hz │ raw 1234 / 12s ──┘
```

## 클래스 책임 분리

| 클래스 | 라인 | 책임 |
|---|---|---|
| `MacroNode(Node)` | 78–105 | rclpy 노드 — `/dsr01/joint_states` 구독만 담당 |
| `BringupDialog(QDialog)` | 111–138 | 시작 시 real/virtual/skip 선택 모달 |
| `BringupManager` | 144–186 | `subprocess.Popen`으로 `dsr_bringup2` launch, SIGINT 종료 |
| `DsrWorker(QObject)` | 192–401 | DSR_ROBOT2 호출 전담 worker (별 스레드) |
| `RosSpinThread(QThread)` | 407–451 | rclpy executor spin loop |
| `MainWindow(QMainWindow)` | 598–1157 | UI 위젯 + 시그널 라우팅 + 파일 I/O |
| `smooth_and_save()` | 457–592 | 인라인 평활화 (smoother.py와 동일 로직) |

## Qt 시그널/슬롯 전체 그래프

```
MainWindow.request_mode          ──► DsrWorker.query_mode  (3s 주기)
MainWindow.request_home          ──► DsrWorker.go_home
MainWindow.request_play          ──► DsrWorker.play(path, vel, acc)
MainWindow.request_estop         ──► DsrWorker.emergency_stop
MainWindow.request_set_ops_speed ──► DsrWorker.set_operation_speed(ratio)

DsrWorker.log            ──► MainWindow._log
DsrWorker.play_started   ──► MainWindow._on_play_started
DsrWorker.play_finished  ──► MainWindow._on_play_finished
DsrWorker.mode_updated   ──► MainWindow._on_mode_updated

RosSpinThread.joint_received ──► MainWindow._on_joint
RosSpinThread.ready          ──► MainWindow._on_ros_ready

QTimer mode_timer (3s) ──► request_mode.emit()
QTimer hz_timer   (1s) ──► _update_hz (statusbar)

slider_ops.valueChanged    ──► _on_ops_slider_changed (라벨 갱신만)
slider_ops.sliderReleased  ──► _on_ops_slider_released → request_set_ops_speed
```

## 재생 속도 컨트롤 (3-layer)

```
실효 속도 = (사용자 vel) × (Operation Speed %) × (movesj 내부 가감속 마진)
                ▲                  ▲                       ▲
              spin_vel          slider_ops          DSR 컨트롤러 결정
              (직접 입력)        (전역 배율)             (조정 불가)
```

| 컨트롤 | 효과 | 값 범위 |
|---|---|---|
| `spin_vel` | `movesj` 의 vel 인자 — 6축 합성 속도 | 1–360 °/s |
| `spin_acc` | `movesj` 의 acc 인자 | 1–720 °/s² |
| 프리셋 4개 | spin_vel/acc를 한 번에 (Slow 15/30 → Max 240/480) | 클릭 |
| 📊 자동 추천 | raw.json 분석 → p90 속도 × 1.2 → spin_vel 자동 설정 | 클릭 |
| `slider_ops` | `change_operation_speed(ratio)` 컨트롤러 전역 배율 | 1–100 % |

### Operation Speed 슬라이더 동작

```
사용자가 슬라이더 드래그 중:
  → valueChanged.emit(v)  — 매 픽셀마다 발생
  → _on_ops_slider_changed: lbl_ops 텍스트만 갱신, 서비스 호출 X

사용자가 손 뗌:
  → sliderReleased.emit()
  → _on_ops_slider_released → request_set_ops_speed.emit(value)
  → DsrWorker.set_operation_speed(ratio)
  → DSR change_operation_speed(ratio) 1회 호출
```

드래그 중 호출 폭주를 막기 위한 패턴. valueChanged 마다 service call 보내면 컨트롤러 큐가 터짐.

### 시연 속도 자동 추천 (`_suggest_speed_from_raw`)

```python
ts = raw["timestamps_ms"]
js = raw["joints_deg"]
dts    = np.diff(ts) / 1000.0
dqs    = np.linalg.norm(np.diff(js, axis=0), axis=1)   # 6축 norm
speeds = dqs / dts                                      # [°/s] 순간
peak   = np.percentile(speeds, 90)                      # 정지/평균 영향 회피
sug_vel = round(peak * 1.2 / 5) * 5                    # 5 단위 반올림
sug_acc = sug_vel * 2
```

평균이 아닌 **90% 분위수** 사용 — 펜던트 시연 중 정지/생각 구간이 평균을 끌어내려 너무 느린 추천이 나오는 것을 방지. ×1.2 마진은 컨트롤러 가감속 손실 보정.

### smooth.json ↔ GUI 양방향 동기화

- **저장 시**: `_on_smooth()` 가 현재 `spin_vel/acc` 를 smooth.json 에 쓴다
- **로드 시**: `_load_action()` 이 smooth.json 의 vel/acc 를 spin_vel/acc 로 복원

→ 액션마다 다른 속도를 따로 보존. 다른 액션 로드해도 그 액션의 속도가 즉시 반영.

## 시작 시퀀스

```
1. main() → QApplication 생성
2. MainWindow.__init__()
   ├─ UI 빌드
   ├─ BringupDialog.exec_()  ◄── modal, 사용자 선택 대기
   ├─ BringupManager.launch(mode)  ◄── subprocess 시작 (real/virtual)
   ├─ RosSpinThread.start()
   │   └─ rclpy.init() → MacroNode 생성
   │       ├─ _dr_set_node(node)  ◄── DR_init 등록
   │       ├─ MultiThreadedExecutor 생성
   │       ├─ rclpy.__executor = self.executor  ◄★ 핵심
   │       └─ ready.emit()
   ├─ DsrWorker → worker_thread (QThread)
   └─ mode_timer 시작 (3000ms)
3. _on_ros_ready()
   ├─ statusbar 업데이트
   └─ QTimer.singleShot(2000) → request_mode.emit()
       └─ 첫 모드 폴링 (controller service ready 대기)
```

## DsrWorker 동시성 가드

`_busy` 플래그로 한 번에 하나만 실행:

```python
@QtCore.pyqtSlot()
def query_mode(self):
    if self._busy:
        return                # 다른 작업 진행 중이면 skip
    ...
    self._busy = True
    try:
        mode = self._fns["get_robot_mode"]()
        ...
    finally:
        self._busy = False
```

이유:
- `get_robot_mode()`는 service call (보통 빠름)
- `movesj()`는 수십초 블로킹
- 둘이 겹치면 `spin_until_future_complete`가 다른 future와 경합 → 데드락 위험
- `_busy=True`인 동안 mode 폴링은 그냥 skip → 재생 끝난 뒤 다음 폴링에서 재개

## DSR_ROBOT2 lazy import (`_ensure_dsr`)

```python
def _ensure_dsr(self):
    if self._dsr_loaded:
        return
    if _dr_get_node() is None:
        raise RuntimeError("ROS 노드 미등록")     # ★ 1차 가드
    import DSR_ROBOT2                            # 이 시점에 g_node = DR_init.__dsr__node
    ...
```

**왜 lazy?** DSR_ROBOT2는 모듈 import 시점에 `g_node`를 캡처하고 모든 service client를 module-level에서 생성한다. ROS 노드가 등록되기 전에 import하면 `g_node = None`으로 영구 고정되어 모든 호출이 `AttributeError`로 실패. 따라서 import는 노드 ready 이후로 미룬다.

## `_service_ready()` — 데드락 회피용 사전 체크

```python
def _service_ready(self, srv_path, timeout=0.5):
    c = getattr(DSR_ROBOT2, "_ros2_get_robot_mode", None)
    return c.wait_for_service(timeout_sec=timeout)
```

DSR 함수는 service ready 확인 없이 바로 `call()` → controller가 죽어있으면 영원히 블로킹. 호출 전에 빠르게 wait_for_service로 체크.

## Bringup launch 옵션

`BringupManager.HOST_BY_MODE`:

```python
"real":    "192.168.1.100"      # ★ 실제 m0609 컨트롤러 IP
"virtual": "127.0.0.1"          # ★ 시뮬레이터는 반드시 localhost
```

⚠️ virtual 모드에 real IP를 주면 `dsr_hw_interface2`가 외부로 connect 시도 → spawner timeout → controller 활성화 실패. 실수 방지를 위해 `HOST_BY_MODE` dict 강제.

```python
cmd = ["ros2", "launch", "dsr_bringup2", "dsr_bringup2_rviz.launch.py",
       f"mode:={mode}", f"host:={host}", f"port:=12345",
       f"model:={ROBOT_MODEL}"]
```

stdout/stderr는 부모 터미널로 그대로 흘림 — DEVNULL로 막으면 spawner 실패 디버깅 불가.

## 종료 처리 (`closeEvent`)

```
1. mode_timer.stop() / hz_timer.stop()
2. worker_thread.quit() + wait(1000)
3. ros.stop() + wait(2000)              ← spin loop 탈출
4. bringup.shutdown()                    ← subprocess SIGINT (5s timeout → SIGKILL)
```

순서가 중요:
- worker → ros 순으로 정리해야 worker가 죽은 노드를 호출하지 않음
- bringup은 마지막 — 우리가 dsr_control_node를 죽이기 전에 ROS 노드부터 destroy

## 주요 함정 (코드 주석에 표시된 것들)

### 1. Name mangling — `__dsr__node` (라인 44–56)

```python
# 클래스 본문 안에서 DR_init.__dsr__node = node 하면
# Python이 _ClassName__node 로 변형 → 엉뚱한 속성에 set
def _dr_set_node(node):
    setattr(DR_init, "__dsr__node", node)
```

모듈 레벨 헬퍼로 통일. 클래스 안에서 직접 `DR_init.__dsr__node = ...` 절대 금지.

### 2. global executor 호환성 (라인 423–431)

```python
self.executor = MultiThreadedExecutor(num_threads=4)
rclpy.__executor = self.executor   # ★ 우리 executor를 global로 등록
self.executor.add_node(self.node)
```

DSR_ROBOT2의 `spin_until_future_complete`는 `rclpy.get_global_executor()` 사용. 우리 노드가 다른 executor에 속하면 global executor의 `add_node`가 False 반환 → spin 안 함 → future 영원히 안 풀림. 우리 executor를 global로 등록해서 우회.

### 3. 모드 전환 검증 (라인 357–367)

```python
self._fns["set_robot_mode"](AUTONOMOUS)
ok = False
for _ in range(20):
    time.sleep(0.1)
    if self._fns["get_robot_mode"]() == AUTONOMOUS:
        ok = True
        break
if not ok:
    self.play_finished.emit(-1, "AUTONOMOUS 전환 실패 — 펜던트가 MANUAL 점유 중")
```

`set_robot_mode`는 요청 성공만 의미하지 실제 전환 보장 안 함. 펜던트가 MANUAL을 점유하면 set이 reject. 폴링으로 검증해야 함.

### 4. mode 라벨 색상 (라인 944–951)

```python
color = {"MANUAL": "#e67e22", "AUTONOMOUS": "#27ae60"}.get(name, "#7f8c8d")
```

MANUAL=주황 (기록 가능), AUTONOMOUS=초록 (재생 가능), 그 외=회색. 사용자가 한눈에 인식.

### 5. 첫 모드 조회 지연 (라인 917)

```python
QtCore.QTimer.singleShot(2000, lambda: self.request_mode.emit())
```

ros.ready 직후 바로 모드 조회하면 dsr_controller2가 아직 service 등록 전이라 실패. 2초 지연.

## 단축키

| 키 | 액션 |
|---|---|
| R | Record |
| S | Stop |
| M | Smooth |
| P | Play |
| Esc | E-STOP |
| Ctrl+O | 폴더 열기 |
| Ctrl+Q | 종료 |

## UI 확장 시 주의

- 새 위젯은 `_build_ui()`에 추가
- DSR 호출이 필요한 액션은 반드시 `DsrWorker`에 슬롯 추가 + `request_*` 시그널 연결
- 메인 스레드에서 직접 `movej/movesj` 호출 절대 금지 (UI 멈춤 + 50ms 콜백 위반)
- 새 토픽 구독은 `MacroNode`에 추가 후 callback에서 시그널 emit → MainWindow 슬롯에서 UI 업데이트
