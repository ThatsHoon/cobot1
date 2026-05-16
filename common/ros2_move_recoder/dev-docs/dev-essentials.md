# ros2_move_recoder — 개발 요점 (Dev Essentials)

업계 종사자가 5분 안에 "이게 뭐 하는 패키지인지, 각 기능이 어떻게 구현돼 있는지"
파악할 수 있도록 한 장에 정리한 문서. 상세한 함정/설계 결정은 옆 문서들 참고.

## 1. 한 줄 요약

> Doosan m0609 협동로봇을 **사람이 직접 시연(직접 교시) → 기록 → 평활화 → 균일 속도
> 자동 재생**하는 매크로 시스템. PyQt5 GUI + PS5 DualSense + OnRobot RG 그리퍼 통합.

## 2. 사용자가 할 수 있는 일

| 기능 | 어떻게 |
|---|---|
| 직접 교시 | 펜던트로 로봇 손으로 움직이며 GUI의 ● Record |
| 균일 속도 재생 | ▶ Play — 시연 시 빠르게/느리게 한 부분도 동일 속도로 |
| Mini-jog (관절 1개 / TCP 1축씩 점진 이동) | GUI 슬라이더 또는 DualSense 우측 스틱 |
| DualSense 컨트롤러로 모든 조작 | × pause, ○ smooth+play, △ home, □ gripper, Create rec, Options jog mode |
| 그리퍼 동기 재생 | Record 중 □ 토글 → Play에서 동일 시점에 자동 발사 |
| Home 복귀 / E-Stop | △ 또는 GUI 빨간 버튼 / × long-press |
| Bringup 모드 선택 | 시작 시 다이얼로그에서 real / virtual / 외부 launch 중 택 |

## 3. 전체 데이터 플로우

```
펜던트(MANUAL)        GUI ●Record       smoother           Play(AUTO)
   │                     │                  │                  │
   │ /dsr01/joint_states │   raw.json       │   smooth.json    │
   ├────────────────────►│ ────────────────►│ ────────────────►│
   │   60–100Hz          │   타임스탬프 +   │   waypoints +    │
   │   (BEST_EFFORT QoS) │   joints(°) +    │   gripper events │
   │                     │   gripper widths │                  │
                                                              ▼
                                                       amovesj(vel,acc)
                                                       + check_motion poll
```

핵심: **시간 정보는 smoother 단계에서 사실상 버린다**. `amovesj`가 시간 무시·균일
속도라 등간격 waypoint만 의미를 가진다. 사람이 멈춰있던 구간은 압축되고, 모션 굽는
지점(전환점)은 강제로 보존되어 곡선 형태를 잃지 않는다.

## 4. 5개 entry point

| 명령 | 파일 | 역할 |
|---|---|---|
| `gui` | `gui.py` (3.5 kLoC) | 통합 GUI — bringup 관리, record/smooth/play, mini-jog, DualSense, 그리퍼 |
| `recorder <name>` | `recorder.py` | `/dsr01/joint_states` 고주기 기록 → `records/<name>/raw.json` |
| `smoother <name>` | `smoother.py` | Savgol + 정지 압축 + 전환점 보존 + arc-length 다운샘플 → `smooth.json` |
| `player <name>` | `player.py` | `smooth.json` → `amovesj` 비동기 재생 |
| `run` | `run.py` | `amovej`/`amovel` 즉시 모션 테스트 (코드 내 `my_motion()` 편집 후 ros2 run) |

## 5. 각 기능 핵심 구현

### 5.1 기록 (recorder.py)
- 노드 namespace `/dsr01`, 주제 `/dsr01/joint_states` 구독.
- **QoS BEST_EFFORT 필수** — RELIABLE이면 DDS 백프레셔로 실측 0.3Hz까지 떨어진 사례
  (`recorder.py:44-50` 주석). KEEP_LAST depth=50.
- 콜백에서 `time.monotonic()` 캡처 → `_buffer_t/_buffer_q` 에 push. lock으로 보호.
- Enter 키 입력은 별도 stdin 스레드. 정지 시 monotonic 시작 시각 기준 `timestamps_ms`
  로 변환해 JSON 직렬화.
- 라디안 → 도, J1~J6 순서 보장 (관절 이름→위치 dict로 매핑).

### 5.2 평활화 (smoother.py — 핵심)
단일 함수 `smooth_and_save()` 가 source-of-truth. CLI와 GUI 모두 import.

5단계 파이프라인 (`smoother.py:30-229`):

1. **Savitzky-Golay 필터** — 펜던트 stepwise 노이즈 제거. window는 `n_raw/10` 자동
   확장(홀수 보정), polyorder는 default 3. 시작/끝 값은 강제 보존 (`smoothed[0]=joints[0]`).
2. **정지 구간 압축** — 인접 샘플 6축 L2 norm < `eps=0.5°` 면 해당 점을 drop.
   사람이 1초 멈춰있던 구간이 1샘플로 축소.
3. **방향 전환점 검출** — 각 관절별 `find_peaks(prominence=2°)` → peaks ∪ troughs.
   이 인덱스는 다운샘플에서도 강제 보존되어 곡선 곡률을 잃지 않음.
4. **Arc-length 등간격 다운샘플링** — `cumsum(‖Δq‖)` 누적 → `linspace(0, total, N)`
   에 가장 가까운 인덱스 추출 → 전환점과 union → unique. 최종 ≤ `max_pts`.
   **상한 100** — `movesj` 의 하드 limit (`smoother.py:191`).
5. **검증/저장** — 인접 점프 max 도(°)도 메타로 기록.

### 5.3 재생 (player.py / amovesj)
- `amovesj(pts, vel, acc)` — Doosan의 등간격 spline 모션. **시간 무시 균일 속도**.
- 비동기 호출이라 즉시 리턴 → `check_motion()` 폴링 loop으로 완료 대기.
- 시작 전 `get_robot_mode() == ROBOT_MODE_AUTONOMOUS` 검증, 실패 시 사용자에게 모드
  전환 요구.
- DSR_ROBOT2 import는 main 안에서 (rclpy.init 후) — `DR_init.__dsr__node` 주입 필요.

### 5.4 GUI (gui.py)
3491줄이지만 핵심 클래스만 보면 구조는 단순:

| 클래스 | 역할 |
|---|---|
| `BringupDialog` / `BringupManager` | 시작 시 모드 선택 → `dsr_bringup2_rviz.launch.py` 를 `subprocess.Popen` 으로 띄움. 종료 시 SIGINT |
| `MacroNode` / `JointStateNode` | 백그라운드 rclpy 노드 (1개는 jog/서비스, 1개는 joint_states 구독) |
| `RosSpinThread` / `JointStateThread` | `MultiThreadedExecutor` 를 별도 QThread에서 spin |
| `DsrWorker` | DSR_ROBOT2 모션 호출 전담 — Qt 메인 스레드를 절대 블로킹하지 않기 위함 |
| `DsrInterruptWorker` | E-Stop / pause 같은 즉시 인터럽트 콜 — DsrWorker와 분리 |
| `_JogDispatcher` | mini-jog rate-limit + 양자화 (DualSense, 슬라이더 공유) |
| `MainWindow` | 위젯 트리, 시그널 배선, 테마 |

**왜 모션을 별도 스레드에 두는가:**
DRFL은 50ms 안에 콜백 리턴하라는 규약이 있고 ROS 서비스 호출도 동기 대기가 잦다.
Qt 메인 스레드가 막히면 GUI가 즉시 멈춤 → DsrWorker가 명령 큐를 받아 처리.

### 5.5 DualSense (dualsense_worker.py)
- `pygame.joystick` (SDL2) — `/dev/input/js0`, 별도 권한 불필요.
- 60Hz 폴링 스레드 → Qt 시그널 emit.
- 핵심 정책 (`dualsense_worker.py:71-79`):
  - **jog vel 양자화 10°/s** — DSR jog는 호출마다 ramp-up 재시작. 미세 변화는 collapse.
  - 동일 axis/부호 유지 시 ≥ 10°/s 변화만 재emit. cooldown 200ms.
  - 부호 변경/axis 변경/정지는 즉시 emit (반응성 보장).
- 버튼: × pause/E-Stop, ○ smooth+play, △ home, □ gripper, Create record, Options jog mode toggle.
- Long-press 임계: × ≥1s = E-Stop, Create ≥2s = 새 프로파일 자동 생성.
- 분리 자동 감지 → `stop_jog` 즉시 발사 (안전).

### 5.6 그리퍼 (gripper_worker.py + onrobot.py)
- OnRobot RG2/RG6, Modbus TCP 192.168.1.1:502 (env 로 override).
- **단일 워커 스레드 정책** (`gripper_worker.py:8-15`):
  pymodbus 2.5.x ModbusTcpClient는 thread-safe 아님. 여러 스레드에서 read/write 시
  socket corrupt → segfault (실측: dsr_controller2 의 get_robot_mode_cb 와 race).
  → 모든 IO는 `_run` 워커 스레드 단독 수행. 외부는 `queue.Queue` 에 명령만 push.
- 1Hz width polling + reconnect 자동.
- 단위: 펌웨어는 1/10 mm 정수 → UI/저장은 mm.

### 5.7 그리퍼 record/play 동기화 (smoother.py:69-140)
- record 중 width 값을 raw.json `gripper_widths_mm` 에 저장.
- smoother가 width 변화 ≥5mm 이면 **의도된 open/close** 로 분류해 event 추출.
- **t_norm 은 사람 시간이 아닌 arc-length 누적 진행률** — 정지 구간 압축으로 인해
  amovesj 실행 시간이 사람 시간보다 짧아져도 event 발사 시점이 어긋나지 않음.
- 인접 events 사이 최소 gap 강제 (RG2 모터 풀 동작 ~2s) → t_norm > 1.0 허용.

## 6. 데이터 형식 (요약)

`records/<name>/raw.json`:
```json
{
  "timestamps_ms": [0, 12, 25, ...],
  "joints_deg":    [[J1..J6], ...],
  "rate_hz_avg":   78.4,
  "duration_sec":  12.345,
  "samples":       968,
  "recorded_at":   "...",
  "gripper_widths_mm": [110.0, 109.5, ...]   // 선택
}
```

`records/<name>/smooth.json`:
```json
{
  "action_name": "...",
  "waypoints_deg": [[J1..J6], ...],   // ≤100점
  "vel": 30.0, "acc": 60.0,
  "record_duration_sec": 12.345,
  "n_waypoints": 78, "n_raw": 968,
  "n_after_stationary_filter": 412,
  "n_turning_points": 14,
  "max_adjacent_jump_deg": 3.21,
  "smoothing": { "window": 97, "polyorder": 3, ... },
  "gripper_events": [
    { "t_norm": 0.0,   "kind": "open",  "width_mm": 110.0 },
    { "t_norm": 0.42,  "kind": "close", "width_mm": 18.5  }
  ]
}
```
정확한 필드는 `data-formats.md`.

## 7. 결정적인 설계 제약 (왜 이렇게 했는가)

| 제약 | 이유 |
|---|---|
| `dsr01` namespace 하드코딩 | DSR_ROBOT2 multi-robot 미지원 |
| MANUAL ↔ AUTONOMOUS 모드 분리 | record는 MANUAL, play는 AUTO 필수 (DRFL 규약) |
| Bringup을 별도 프로세스로 | 시작 모드 (real/virtual/외부) 동적 선택 + 깨끗한 SIGINT 종료 |
| amovesj 사용 (movesj가 아닌) | 비동기 + 균일 속도 + GUI thread 비블로킹 |
| max 100 waypoints | movesj/amovesj API 한계 — smoother가 강제 다운샘플 |
| ROBOT_MODEL = "m0609" 고정 | 다른 모델 사용 시 `gui.py:42`, `run.py:12`, `player.py:18` 동시 변경 |
| BEST_EFFORT joint_states QoS | RELIABLE 시 DDS backpressure로 0.3Hz 사례 |
| pymodbus 단일 스레드 | thread-safe 아니어서 segfault 위험 |
| 그리퍼 t_norm = arc-length 진행률 | 정지 구간 압축 후에도 발사 시점 일관 |

## 8. 디렉토리 한눈에

```
ros2_move_recoder/
├── ros2_move_recoder/
│   ├── recorder.py          # 163줄 — 기록
│   ├── smoother.py          # 302줄 — 평활/다운샘플 (핵심 로직)
│   ├── player.py            # 104줄 — amovesj 재생
│   ├── run.py               #  87줄 — 즉시 모션 테스트
│   ├── gui.py               # 3491줄 — PyQt5 통합 GUI
│   ├── dualsense_worker.py  # 616줄 — PS5 컨트롤러
│   ├── gripper_worker.py    # 238줄 — OnRobot Modbus 워커
│   └── onrobot.py           # 184줄 — RG2/RG6 wrapper
├── records/<name>/          # raw.json + smooth.json
├── dev-docs/                # 이 문서 포함 13개
└── package.xml / setup.py   # entry_points 5개
```

## 9. 빠른 빌드/실행

```bash
cd ~/cobot_ws
colcon build --packages-select ros2_move_recoder --symlink-install
source install/setup.bash

ros2 run ros2_move_recoder gui                  # 통합 GUI (권장)
# 또는 CLI 파이프라인:
ros2 run ros2_move_recoder recorder my_first    # Enter → 시연 → Enter
ros2 run ros2_move_recoder smoother my_first    # raw → smooth
ros2 run ros2_move_recoder player   my_first    # AUTO 모드에서 재생
```

자세한 옵션/디버깅: `build-and-run.md`, `troubleshooting.md`.

## 10. 더 깊게 들어갈 때 읽을 문서

- 알고리즘 자세히 → `pipeline.md`
- GUI 위젯 트리 / 시그널 그래프 → `gui.md`
- DualSense 매핑 / 디버깅 → `dualsense-mapping.md`
- 그리퍼 통합 자세히 → `gripper.md`
- 함정 모음 → `troubleshooting.md`
- 기능 추가 가이드 → `extending.md`
- 변경 이력 → `CHANGELOG.md`
