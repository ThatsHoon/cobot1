# Troubleshooting — 알려진 함정 모음

이 패키지를 만들면서 실제로 부딪힌 함정과 해결법. 두산 ROS 2의 일반 함정은 `~/.claude/skills/doosan-robotics/references/multi-robot-and-quirks.md` 참고.

## 1. `AttributeError: 'NoneType' object has no attribute 'create_client'`

**원인**: `DR_init.__dsr__node` 등록 **전**에 `from DSR_ROBOT2 import ...` 했음. DSR_ROBOT2가 import 시점에 `g_node = DR_init.__dsr__node` 캡처 → `None` 박힘 → 모든 service client 생성 실패.

**해결**:
```python
# 잘못된 순서
from DSR_ROBOT2 import movej   # ❌ g_node = None 고정
DR_init.__dsr__node = node

# 올바른 순서
node = rclpy.create_node(...)
DR_init.__dsr__node = node     # ✓ 등록 먼저
from DSR_ROBOT2 import movej   # ✓ 이제 import
```

GUI는 `_ensure_dsr()` 함수로 lazy import 처리 (gui.py:207).

## 2. Name mangling — `_ClassName__dsr__node` 가 됨

**원인**: Python의 name mangling 규칙. 클래스 본문 안에서 `__name` 패턴(앞 `__`, 끝 비-`__`)을 쓰면 자동으로 `_ClassName__name`으로 변형.

```python
class Foo:
    def setup(self):
        DR_init.__dsr__node = node   # ❌ 실제로는 DR_init._Foo__dsr__node 에 set
```

DSR_ROBOT2는 `DR_init.__dsr__node` 를 읽음 → 영원히 `None` → 모든 호출 실패.

**해결**: 모듈 레벨 헬퍼 (gui.py:49–56):
```python
def _dr_set_node(node):
    setattr(DR_init, "__dsr__node", node)
def _dr_get_node():
    return getattr(DR_init, "__dsr__node", None)
```

클래스 안에서는 절대 `DR_init.__dsr__node = ...` 직접 안 씀.

## 3. DSR 호출이 영원히 안 끝남 (future complete 안 됨)

**원인**: DSR_ROBOT2의 `spin_until_future_complete`는 `rclpy.get_global_executor()` 사용. 우리가 별도 `MultiThreadedExecutor` 만들고 노드를 거기 등록하면, global executor가 `add_node`를 시도하지만 노드가 이미 다른 executor에 속해 있어 False 반환 → global executor는 노드를 spin 못 함 → future 영원히 안 풀림.

**해결** (gui.py:430):
```python
self.executor = MultiThreadedExecutor(num_threads=4)
rclpy.__executor = self.executor   # ★ 우리 executor를 global로 등록
self.executor.add_node(self.node)
```

## 4. virtual 모드인데 controller spawner timeout

**원인**: virtual 시뮬레이터는 로컬에서 동작. real IP(`192.168.1.100`)를 그대로 launch에 넘기면 `dsr_hw_interface2`가 외부 IP로 connect 시도 → 응답 없음 → spawner timeout → controller 활성화 실패.

**해결**: virtual 모드는 반드시 `127.0.0.1` (gui.py:148):
```python
HOST_BY_MODE = {"real": "192.168.1.100", "virtual": "127.0.0.1"}
```

## 5. `set_robot_mode(AUTONOMOUS)` 했는데 모드 안 바뀜

**원인**: `set_robot_mode` 반환은 "요청 접수 성공"만 의미. 펜던트가 MANUAL을 점유 중이면 컨트롤러가 silently reject. 다음 `get_robot_mode`는 여전히 MANUAL.

**해결** (gui.py:357):
```python
self._fns["set_robot_mode"](AUTONOMOUS)
ok = False
for _ in range(20):
    time.sleep(0.1)
    if self._fns["get_robot_mode"]() == AUTONOMOUS:
        ok = True; break
if not ok:
    self.play_finished.emit(-1, "AUTONOMOUS 전환 실패 — 펜던트 제어권 확인")
```

펜던트의 AUTO 버튼/제어권을 ROS 측으로 넘기라고 사용자에게 안내.

## 6. GUI 시작 직후 모드 조회 실패

**원인**: `RosSpinThread.ready` emit 직후 바로 `get_robot_mode` 호출하면 `dsr_controller2`가 아직 service 등록 전이라 `wait_for_service` 실패.

**해결** (gui.py:917):
```python
QtCore.QTimer.singleShot(2000, lambda: self.request_mode.emit())
```

2초 지연 후 첫 폴링.

## 7. GUI 메인 스레드에서 `movesj` 호출 → UI freeze + DRFL 50ms 위반

**원인**: `movesj`는 수십 초 블로킹. Qt 메인 스레드에서 호출하면 이벤트 루프 멈춤 → 클릭/그리기 모두 정지. + DRFL 콜백 규약(50ms 안에 반환) 위반.

**해결**: `DsrWorker(QObject)` + `worker_thread = QThread()` + `moveToThread`. 시그널/슬롯으로 트리거.

## 8. 동시 DSR 호출 → 데드락

**원인**: mode 폴링(3초 주기)과 사용자 트리거(play/home) 가 겹치면, 두 future가 동일 executor의 spin 자원을 경쟁.

**해결** (gui.py:204):
```python
self._busy = False   # 한 번에 하나만 실행
def query_mode(self):
    if self._busy: return
    self._busy = True
    try: ...
    finally: self._busy = False
```

`play` 도중 mode 폴링은 그냥 skip. play 끝나면 다음 폴링에서 재개.

## 9. `movesj` rc=非0, "인접 점프 과대"

**원인**: smoother가 다운샘플링하면서 인접 waypoint 6축 L2 norm이 과도하게 큼 (>30° 정도).

**진단**: smooth.json의 `max_adjacent_jump_deg` 확인.

**해결**:
- `max_pts` 늘리기 (waypoint 더 많이)
- `eps` (정지 임계) 줄이기 (정지 압축 약하게)
- `prom` (전환점) 줄이기 (전환점 더 많이 보존)
- 너무 빠른 시연 (펜던트 휙 휙 움직임) → 천천히 다시 시연

## 10. `/joint_states` 0 Hz

**원인 후보**:
1. bringup 안 떴음 → `ros2 node list` 확인
2. controller 죽음 → bringup 터미널 로그 확인
3. QoS mismatch → `ros2 topic info -v /dsr01/joint_states`로 양쪽 비교
4. namespace 오타 → 우리 노드는 `/dsr01/joint_states` 구독 (namespace 슬래시 포함)

## 11. 종료 시 `context already invalid` 또는 hang

**원인**: 종료 순서 잘못. ROS 노드 destroy 전에 worker가 아직 호출 중.

**해결** (gui.py:1139):
```python
def closeEvent(self, event):
    self.mode_timer.stop()           # 1. 폴링 정지
    self.worker_thread.quit(); self.worker_thread.wait(1000)  # 2. worker 종료
    self.ros.stop(); self.ros.wait(2000)                       # 3. ROS spin 정지
    self.bringup.shutdown()           # 4. subprocess SIGINT (마지막)
```

## 12. records/ 안에 빈 폴더만 생기고 raw.json 없음

**원인**: 기록 시작 후 `/joint_states` 메시지가 안 옴 (대부분 bringup 미실행 또는 JointState `name` 필드가 `joint_1` 형식 아님).

**진단**:
```bash
ros2 topic echo /dsr01/joint_states --once   # name 필드 확인
```

기대: `name: ['joint_1', 'joint_2', ..., 'joint_6']`. 다르면 recorder.py:57의 `f"joint_{i}"` 매핑 수정 필요.

## 13. PyQt5 import 실패

```
ImportError: No module named 'PyQt5'
```

**해결**:
```bash
sudo apt install python3-pyqt5
```

apt 패키지를 쓰는 이유: pip install pyqt5는 시스템 Qt 라이브러리와 ABI 충돌 가능. ROS 2 다른 노드(rqt 등)와 conflict 회피.

## 14. 재생이 시연보다 너무 느림

**원인 후보** (큰 영향 → 작은 영향):

1. **`vel`/`acc` 가 너무 낮음** — 기본 30/60은 보수적. m0609 max 약 360°/s. → **▶ Normal 프리셋(60/120)** 또는 **📊 자동 추천** 사용
2. **Operation Speed 슬라이더가 100% 미만** — 컨트롤러 전역 배율 확인. 펜던트에서 50%로 잡혀 있을 수도 있음
3. **`movesj` 시간 정보 폐기** — smoother가 timestamps 무시하고 균일 속도 재생. 시연 시 빠른 구간이 균일 평준화로 느려짐
4. **`max_pts` 가 너무 적음** — 인접 waypoint 거리가 멀면 spline 가감속 마진 커짐. max_pts 60→100 시도
5. **`max_adjacent_jump_deg` > 15°** — 점프 크면 컨트롤러가 안전 가감속 → 체감 속도 ↓. 평활화 파라미터 조정

**진단**:
```bash
# 적용된 vel/acc 확인
cat ~/cobot_ws/src/ros2_move_recoder/records/<name>/smooth.json | grep -E "vel|acc"

# 펜던트의 operation speed 확인 (펜던트 화면) — ROS GUI 슬라이더와 둘 중 작은 값 적용
```

근본 해결안 (시연 속도 정확 재현)은 `extending.md` 의 "시간 기반 재생" 섹션 참고.

## 15. Operation Speed 슬라이더 효과 없음

**원인**: `change_operation_speed` 가 펌웨어/DSR_ROBOT2 버전에 따라 미지원.

**진단**: GUI 로그에 "[dsr] ⚠️ change_operation_speed 미지원 펌웨어" 뜨면 해당.

**우회**: spin_vel/acc를 직접 줄이기. 또는 펜던트의 operation speed 슬라이더 사용.

## 16. 기존 macros 폴더 (사용 안 함)

`macros/` 디렉토리는 historical leftover. 현재 모든 데이터는 `records/<name>/`. 정리하고 싶으면 그냥 삭제 가능.

## 17. `dsr_bringup2` 종료 후 좀비 프로세스 잔존

**증상**: GUI 종료 후 `ps auxf | grep dsr_bringup2` 에 `<defunct>` 가 남음. 다음 launch 시 포트 12345 점유 충돌.

**원인**: GUI 의 `BringupManager.shutdown()` 이 SIGKILL 후 `proc.wait()` 으로 reap 하지 않아 부모 종료까지 좀비 유지.

**해결**: 이미 적용됨 — `shutdown()` 은 SIGINT → wait(5s) → SIGKILL → wait(2s) 3단계로 reap 까지 수행 (gui.py:246+). 만약 그래도 좀비가 보이면 외부에서:
```bash
pkill -INT -f dsr_bringup2_rviz   # graceful
sleep 5
pkill -KILL -f dsr_bringup2_rviz  # fallback
```

