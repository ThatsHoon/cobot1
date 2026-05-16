# 고정 시퀀스 재생 + /orders 복합주문 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 조리 로직을 동적 액션 해석에서 사전 녹화 세그먼트 체인 재생으로 전환하고, 주문 인지를 `/orders` 복합주문 기반으로 바꾼다.

**Architecture:** ros2_move_recoder에 헤드리스 재생 코어(`playback.py`)를 추출해 gui/player/robo_chef가 공유한다. robo_chef는 순수 로직(`cooking_core.py`, `order_core.py`) + 얇은 노드(`sequence_runner.py`, 개편된 `firebase_bridge.py`)로 분리해 ROS/Firebase/DSR 없이 단위테스트 가능하게 만든다. 불필요한 동적 엔진·중복 트리·web 퍼블리셔는 제거한다.

**Tech Stack:** ROS 2 Humble (rclpy, ament_python), Doosan DSR_ROBOT2, firebase_admin (RTDB), pytest, PyQt5(기존 GUI).

**Spec:** `docs/superpowers/specs/2026-05-16-fixed-sequence-playback-design.md`

---

## File Structure

| 파일 | 책임 | 신규/수정/삭제 |
|---|---|---|
| `common/ros2_move_recoder/ros2_move_recoder/playback.py` | 헤드리스 재생 코어 (amovesj + 그리퍼 타임라인) | 신규 |
| `common/ros2_move_recoder/ros2_move_recoder/player.py` | playback 호출 얇은 CLI(+`--yes`) | 수정 |
| `common/ros2_move_recoder/ros2_move_recoder/gui.py` | 재생/그리퍼 경로 → playback 호출 | 수정 |
| `common/ros2_move_recoder/test/test_playback.py` | playback 순수 로직 단위테스트 | 신규 |
| `main_side/robo_chef/nodes/cooking_core.py` | 순수: jobs 실행 루프·상태 dict 빌더 (ROS/DSR 무관) | 신규 |
| `main_side/robo_chef/nodes/sequence_runner.py` | DSR 소유 노드 (cooking_core + playback 와이어링) | 신규 |
| `main_side/robo_chef/nodes/order_core.py` | 순수: pending 선택·items→jobs 전개·상태 전이 매핑 | 신규 |
| `main_side/robo_chef/nodes/firebase_bridge.py` | `/orders` 감지·전개 노드 (order_core 와이어링) | 재작성 |
| `main_side/robo_chef/test/test_cooking_core.py` | cooking_core 단위테스트 | 신규 |
| `main_side/robo_chef/test/test_order_core.py` | order_core 단위테스트 | 신규 |
| `main_side/robo_chef/setup.py` | entry_points 갱신 | 수정 |
| `main_side/robo_chef/package.xml` | 의존성 갱신 | 수정 |
| `sub1_side/web/panel/customer_status/index.html` | 신 `/robot_status` 스키마 렌더 | 수정 |
| `sub1_side/web/backend/recipe_seeder.py` | 시드에 `segments` 필드 | 수정 |
| `sub1_side/web/backend/app.py` | 죽은 주문 코드·라우트 제거 | 수정 |
| `sub1_side/web/start_all.sh` | ros2_order_publisher 기동 라인 제거 | 수정 |
| 삭제: robo_chef `nodes/{recipe_parser,state_manager,executer,recipe_tester}.py`, `core/`, `data/`, `src/interfaces/recipe_msgs/`, 중복 `src/robo_chef/`; web `backend/ros2_order_publisher.py`,`backend/ros2_srv_call.py`,`backend/run_ros2_publisher.sh` | — | 삭제 |

**테스트 실행 전제:** `cd ~/cobot_ws && source /opt/ros/humble/setup.bash`. 순수 모듈 테스트는 ROS 빌드 없이 `pytest` 직접 실행 가능(아래 각 Task의 `PYTHONPATH` 지정 사용). 커밋은 cobot1 루트가 git 저장소가 아니므로 **각 하위 git 저장소**(`common/ros2_move_recoder`, `main_side/robo_chef`)에서 수행. web(`sub1_side`)은 git 저장소가 아니므로 커밋 생략(변경만).

---

## Phase 1 — 재생 코어 추출 (`playback.py`)

### Task 1: playback.py 공개 API 골격 + 실패 경로 테스트

**Files:**
- Create: `common/ros2_move_recoder/ros2_move_recoder/playback.py`
- Test: `common/ros2_move_recoder/test/test_playback.py`

- [ ] **Step 1: 실패 테스트 작성**

`common/ros2_move_recoder/test/test_playback.py`:
```python
import json, os, tempfile
import ros2_move_recoder.playback as pb


def _write_smooth(d):
    path = os.path.join(tempfile.mkdtemp(), "smooth.json")
    with open(path, "w") as f:
        json.dump(d, f)
    return path


def test_missing_file_returns_failure():
    res = pb.play_segment("/no/such/smooth.json")
    assert res.ok is False
    assert "missing" in (res.error or "").lower()


def test_not_autonomous_returns_failure(monkeypatch):
    path = _write_smooth({"waypoints_deg": [[0, 0, 90, 0, 90, 0]], "vel": 30, "acc": 60})
    monkeypatch.setattr(pb, "_dsr", lambda: pb._DSR(
        amovesj=lambda *a, **k: 0, check_motion=lambda: False,
        posj=lambda *w: list(w), get_robot_mode=lambda: 0,
        ROBOT_MODE_AUTONOMOUS=1, sstop=lambda: None))
    res = pb.play_segment(path, require_autonomous=True)
    assert res.ok is False
    assert "autonomous" in (res.error or "").lower()
```

- [ ] **Step 2: 실패 확인**

Run: `cd ~/cobot_ws/src/cobot1/common/ros2_move_recoder && PYTHONPATH=. python -m pytest test/test_playback.py -v`
Expected: FAIL — `ModuleNotFoundError: ros2_move_recoder.playback`

- [ ] **Step 3: playback.py 골격 구현**

`common/ros2_move_recoder/ros2_move_recoder/playback.py`:
```python
"""ros2_move_recoder.playback — 헤드리스 재생 코어.

PyQt/DualSense/input() 의존 없음. gui.py·player.py·robo_chef.sequence_runner 가 공유.
amovesj 비동기 모션 + smooth.json gripper_events 타임라인 재생.
"""
from __future__ import annotations
import json
import os
import threading
import time
from dataclasses import dataclass


@dataclass
class PlayResult:
    ok: bool
    duration_sec: float
    error: str | None = None
    measured_duration_sec: float | None = None


@dataclass
class _DSR:
    amovesj: object
    check_motion: object
    posj: object
    get_robot_mode: object
    ROBOT_MODE_AUTONOMOUS: object
    sstop: object


def _dsr() -> _DSR:
    """실로봇 DSR_ROBOT2 바인딩. 테스트는 monkeypatch 로 대체."""
    from DSR_ROBOT2 import (amovesj, check_motion, posj, get_robot_mode,
                            ROBOT_MODE_AUTONOMOUS)
    try:
        from DSR_ROBOT2 import stop as _stop, DR_SSTOP
        sstop = lambda: _stop(DR_SSTOP)
    except Exception:
        sstop = lambda: None
    return _DSR(amovesj, check_motion, posj, get_robot_mode,
                ROBOT_MODE_AUTONOMOUS, sstop)


def _log(logger, msg):
    (logger.info if logger else print)(msg)


def play_segment(smooth_path: str, *, gripper=None,
                  require_autonomous: bool = True, on_progress=None,
                  abort_event: "threading.Event | None" = None,
                  logger=None) -> PlayResult:
    if not os.path.isfile(smooth_path):
        return PlayResult(False, 0.0, f"missing segment file: {smooth_path}")
    with open(smooth_path) as f:
        sm = json.load(f)

    dsr = _dsr()
    if require_autonomous:
        try:
            if dsr.get_robot_mode() != dsr.ROBOT_MODE_AUTONOMOUS:
                return PlayResult(False, 0.0, "robot not in AUTONOMOUS mode")
        except Exception as e:  # noqa: BLE001
            return PlayResult(False, 0.0, f"get_robot_mode failed: {e}")

    waypoints = sm.get("waypoints_deg") or []
    vel = float(sm.get("vel", 30.0))
    acc = float(sm.get("acc", 60.0))
    if not waypoints:
        return PlayResult(False, 0.0, "no waypoints_deg in smooth.json")

    if on_progress:
        on_progress("amovesj_start")
    pts = [dsr.posj(*w) for w in waypoints]
    t0 = time.monotonic()
    rc = dsr.amovesj(pts, vel=vel, acc=acc)
    if rc != 0:
        dsr.sstop()
        return PlayResult(False, time.monotonic() - t0, f"amovesj rc={rc}")

    grip = _GripperTimeline(sm, gripper, vel, logger) if gripper else None
    if grip:
        grip.start()
    try:
        while dsr.check_motion():
            if abort_event is not None and abort_event.is_set():
                dsr.sstop()
                if grip:
                    grip.stop()
                return PlayResult(False, time.monotonic() - t0, "aborted")
            time.sleep(0.05)
    finally:
        if grip:
            grip.finish()
    dt = time.monotonic() - t0
    if on_progress:
        on_progress("done")
    return PlayResult(True, dt, None, grip.measured_duration if grip else None)


class _GripperTimeline:
    """gui.py 의 _start_gripper_play_timeline / _ensure_gripper_final_state 로직을
    self.* 의존 없이 이식. PORTING TASK: Task 2 에서 본문을 채운다."""
    def __init__(self, sm, gripper, play_vel, logger):
        self.sm = sm
        self.gripper = gripper
        self.play_vel = play_vel
        self.logger = logger
        self.measured_duration = None

    def start(self):  # pragma: no cover - Task 2 에서 구현
        raise NotImplementedError

    def stop(self):  # pragma: no cover
        raise NotImplementedError

    def finish(self):  # pragma: no cover
        raise NotImplementedError
```

- [ ] **Step 4: 통과 확인**

Run: `cd ~/cobot_ws/src/cobot1/common/ros2_move_recoder && PYTHONPATH=. python -m pytest test/test_playback.py -v`
Expected: PASS (2 passed) — 그리퍼 없는 경로만 검증하므로 `_GripperTimeline` 미구현 무관.

- [ ] **Step 5: 커밋**

```bash
cd ~/cobot_ws/src/cobot1/common/ros2_move_recoder
git add ros2_move_recoder/playback.py test/test_playback.py
git commit -m "feat(playback): headless play_segment core skeleton + failure-path tests"
```

### Task 2: 그리퍼 타임라인 이식 (`_GripperTimeline`)

**Files:**
- Modify: `common/ros2_move_recoder/ros2_move_recoder/playback.py` (`_GripperTimeline`)
- Reference (읽고 그대로 이식): `common/ros2_move_recoder/ros2_move_recoder/gui.py:2721-2930`
- Test: `common/ros2_move_recoder/test/test_playback.py`

- [ ] **Step 1: 그리퍼 스케줄 테스트 작성** (append)

```python
class FakeGripper:
    def __init__(self):
        self.calls = []
        self._w = 100.0
    def open(self):  self.calls.append(("open", time.monotonic()))
    def close(self): self.calls.append(("close", time.monotonic()))
    def move(self, w): self.calls.append(("move", w))
    def last_state(self): return "open"
    def width_mm(self): return self._w


def test_gripper_events_fire_in_order(monkeypatch):
    sm = {"waypoints_deg": [[0]*6, [10]*6], "vel": 30, "acc": 60,
          "record_duration_sec": 2.0,
          "gripper_events": [
              {"t_norm": 0.0, "kind": "open", "width_mm": 100},
              {"t_norm": 0.9, "kind": "close", "width_mm": 20}]}
    path = _write_smooth(sm)
    motion = {"n": 0}
    def fake_check():
        motion["n"] += 1
        return motion["n"] < 6   # ~0.3s 모션
    monkeypatch.setattr(pb, "_dsr", lambda: pb._DSR(
        amovesj=lambda *a, **k: 0, check_motion=fake_check,
        posj=lambda *w: list(w), get_robot_mode=lambda: 1,
        ROBOT_MODE_AUTONOMOUS=1, sstop=lambda: None))
    g = FakeGripper()
    res = pb.play_segment(path, gripper=g, require_autonomous=False)
    assert res.ok is True
    kinds = [c[0] for c in g.calls if c[0] in ("open", "close")]
    assert kinds == ["open", "close"]   # 순서 보존
```

- [ ] **Step 2: 실패 확인**

Run: `cd ~/cobot_ws/src/cobot1/common/ros2_move_recoder && PYTHONPATH=. python -m pytest test/test_playback.py::test_gripper_events_fire_in_order -v`
Expected: FAIL — `NotImplementedError` (`_GripperTimeline.start`)

- [ ] **Step 3: gui.py 로직 이식**

`gui.py:2721-2930` 의 `_start_gripper_play_timeline` 와 `_ensure_gripper_final_state` 본문을 읽어 `_GripperTimeline` 으로 옮긴다. 치환 규칙(자기상태→인자):
- `self._sm` → `self.sm` · `self.gripper` → `self.gripper` · `self._log(x)` → `_log(self.logger, x)`
- 현재 play 속도: gui의 `current_play_vel`/`get_operation_speed_ratio` 의존 → 생성자 인자 `self.play_vel` 사용(스무딩 vel 기준 단순화; 비율 보정 코드는 1.0 로 대체).
- 스레드: gui가 `QTimer`/시그널이면 `threading.Thread(daemon=True)` + `time.monotonic()` 절대시각 스케줄로 대체. `GRIP_MIN_GAP_S = 2.0` 와 `target_t = max(target_t, last_emit_t + GRIP_MIN_GAP_S)` 간격 규칙, 소요시간 추정 우선순위(`measured_duration_sec × (...)` → `record_duration_sec × (...)` → `arc_len/vel × 1.10`)를 그대로 보존.
- `kind`→그리퍼 호출 매핑(gui 동일): `open`→`gripper.open()`, `close`→`gripper.close()`, `move`→`gripper.move(width_mm)`.
- `start()`: 타임라인 스레드 기동. `stop()`: abort 시 스레드 즉시 종료(이벤트 set). `finish()`: 정상 종료 — `_ensure_gripper_final_state` 로직(마지막 이벤트 상태 강제) + `measured_duration` 계산해 `self.measured_duration` 에 저장.

**설계 정정(중요):** 초안은 정상 종료 경로에서 타임라인 스레드가 마지막 이벤트를 방출한 뒤 `finish()` 가 같은 이벤트를 한 번 더 강제 적용해 **중복**되고, 모션이 녹화보다 빨리 끝나도 스레드가 원 스케줄까지 sleep 해 `finish()` join 이 **지연**되는 결함이 있었다. 정정안: abort용 `_stop` 과 정상종료용 `_flush` 를 **분리**한다. 정상 종료 시 `_flush` 를 set → 스레드가 남은 이벤트를 **즉시 순서대로 1회씩** 방출하고 종료(중복·지연 없음). abort 시 `_stop` → 잔여 이벤트 폐기(로봇 정지했으므로). gui 원본의 `_ensure_gripper_final_state` 강제-적용은 "미방출 이벤트 fallback" 의도였으므로, 모든 이벤트가 정확히 1회 방출되면 마지막 이벤트가 곧 최종상태가 되어 별도 강제-적용이 불필요하다.

구현(이식 결과, 정정 반영):
```python
class _GripperTimeline:
    GRIP_MIN_GAP_S = 2.0

    def __init__(self, sm, gripper, play_vel, logger):
        self.sm = sm
        self.gripper = gripper
        self.play_vel = play_vel
        self.logger = logger
        self.measured_duration = None
        self._events = list(sm.get("gripper_events") or [])
        self._stop = threading.Event()    # abort: 잔여 이벤트 폐기
        self._flush = threading.Event()   # 정상 종료: 잔여 이벤트 즉시 방출
        self._thr = None
        self._t0 = None

    def _est_duration(self):
        sm = self.sm
        smooth_vel = float(sm.get("vel", self.play_vel) or self.play_vel)
        ratio = (smooth_vel / self.play_vel) if self.play_vel else 1.0
        md = float(sm.get("measured_duration_sec", 0.0) or 0.0)
        if md > 0:
            return md * ratio
        rd = float(sm.get("record_duration_sec", 0.0) or 0.0)
        if rd > 0:
            return rd * ratio
        return 1.0  # 최후 보정(아크길이 추정 불가시)

    def _apply(self, kind, width_mm):
        if kind == "open":
            self.gripper.open()
        elif kind == "close":
            self.gripper.close()
        else:
            self.gripper.move(width_mm)

    def _run(self):
        if not self._events:
            _log(self.logger, "[grip][play] skip — gripper_events 없음")
            return
        dur = self._est_duration()
        self._t0 = time.monotonic()
        last_emit_t = -1e9
        for ev in self._events:
            if self._stop.is_set():
                return
            target_t = float(ev.get("t_norm", 0.0)) * dur
            target_t = max(target_t, last_emit_t + self.GRIP_MIN_GAP_S)
            # 스케줄 시각까지 대기. abort(_stop) 또는 정상종료(_flush) 시 즉시 방출.
            while not self._stop.is_set() and not self._flush.is_set():
                now = time.monotonic() - self._t0
                if now >= target_t:
                    break
                time.sleep(min(0.02, target_t - now))
            if self._stop.is_set():
                return
            self._apply(ev.get("kind"), ev.get("width_mm"))
            last_emit_t = target_t

    def start(self):
        self._thr = threading.Thread(target=self._run, daemon=True)
        self._thr.start()

    def stop(self):
        """abort: 잔여 이벤트 폐기, 최종상태 강제 안 함."""
        self._stop.set()
        if self._thr:
            self._thr.join(timeout=1.0)

    def finish(self):
        """정상 종료: 남은 이벤트를 즉시 순서대로 방출(중복 없음)."""
        self._flush.set()
        if self._thr:
            self._thr.join(timeout=5.0)
        if self._t0 is not None:
            self.measured_duration = round(time.monotonic() - self._t0, 3)
```

- [ ] **Step 4: 통과 확인**

Run: `cd ~/cobot_ws/src/cobot1/common/ros2_move_recoder && PYTHONPATH=. python -m pytest test/test_playback.py -v`
Expected: PASS (4 passed)

- [ ] **Step 5: 커밋**

```bash
cd ~/cobot_ws/src/cobot1/common/ros2_move_recoder
git add ros2_move_recoder/playback.py test/test_playback.py
git commit -m "feat(playback): port gui gripper-event timeline into _GripperTimeline"
```

### Task 3: player.py 를 playback 호출로 축소 (+`--yes`)

**Files:**
- Modify: `common/ros2_move_recoder/ros2_move_recoder/player.py` (전체 교체)

- [ ] **Step 1: player.py 교체**

```python
"""ros2_move_recoder.player — smooth.json 재생 CLI (playback 코어 위임).

  ros2 run ros2_move_recoder player <name> [--yes]
"""
import os
import sys
import rclpy
import DR_init
from ros2_move_recoder.playback import play_segment

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
MACROS_DIR = os.path.expanduser("~/cobot_ws/src/ros2_move_recoder/records")


def main(args=None):
    argv = [a for a in sys.argv[1:] if a]
    yes = "--yes" in argv
    names = [a for a in argv if not a.startswith("-")]
    if not names:
        print("사용법: ros2 run ros2_move_recoder player <name> [--yes]")
        sys.exit(1)
    name = names[0]
    smooth_path = os.path.join(MACROS_DIR, name, "smooth.json")
    if not os.path.isfile(smooth_path):
        print(f"❌ 평활화 파일 없음: {smooth_path}")
        sys.exit(1)
    if not yes:
        try:
            if input("[player] 재생 시작? 작업 공간 안전한가요? (y/N) ").strip().lower() != "y":
                print("[player] 취소됨")
                return
        except EOFError:
            print("[player] 비대화 환경 — --yes 필요")
            return

    rclpy.init(args=args)
    node = rclpy.create_node("macro_player", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    try:
        res = play_segment(smooth_path, require_autonomous=True)
        print(f"[player] {'✅ 완료' if res.ok else '❌ 실패'}: "
              f"{res.error or ''} ({res.duration_sec:.2f}s)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: 임포트 스모크 확인**

Run: `cd ~/cobot_ws/src/cobot1/common/ros2_move_recoder && PYTHONPATH=. python -c "import ast; ast.parse(open('ros2_move_recoder/player.py').read()); print('OK')"`
Expected: `OK`

- [ ] **Step 3: 커밋**

```bash
cd ~/cobot_ws/src/cobot1/common/ros2_move_recoder
git add ros2_move_recoder/player.py
git commit -m "refactor(player): delegate to playback core, add --yes non-interactive flag"
```

### Task 4: gui.py 재생/그리퍼 경로를 playback 로 일원화

**Files:**
- Modify: `common/ros2_move_recoder/ros2_move_recoder/gui.py` (`_play_impl` ~505-577, `_do_play_no_confirm` ~2448, `_start_gripper_play_timeline`/`_ensure_gripper_final_state` ~2721-2930 호출부)

- [ ] **Step 1: 재생 경로 치환**

`_play_impl`(gui.py:505)의 `amovesj` 직접 호출 + check_motion 폴링 블록과, 그리퍼 타임라인 메서드(`_start_gripper_play_timeline`/`_ensure_gripper_final_state`) 호출을 다음으로 대체한다. 기존 메서드 본문은 **삭제하지 말고** 내부에서 `play_segment` 를 호출하도록 바꿔 외부 호출자(시그널 연결)를 깨지 않는다:

```python
from ros2_move_recoder.playback import play_segment   # gui.py 상단 import 블록에 추가

# _play_impl 내부의 amovesj+폴링+그리퍼 타임라인 블록을 치환:
res = play_segment(
    smooth_path,
    gripper=getattr(self, "gripper", None),
    require_autonomous=False,            # GUI는 자체 모드전환 후 호출
    abort_event=self._play_abort,        # 기존 abort 신호가 threading.Event 가 아니면 새로 생성해 연결
    logger=None,
)
self.log.emit(f"[play] {'완료' if res.ok else '실패'}: {res.error or ''} ({res.duration_sec:.2f}s)")
```
`self._play_abort` 가 없으면 `MainWindow.__init__` 에 `self._play_abort = threading.Event()` 추가하고, 기존 중단 버튼이 이 이벤트를 `set()`/`clear()` 하도록 연결한다. `_start_gripper_play_timeline`/`_ensure_gripper_final_state` 는 이제 사용처가 없으면 본문을 `pass` 로 비우고 docstring 에 "기능은 playback._GripperTimeline 으로 이전됨" 명시(중복 제거).

- [ ] **Step 2: GUI 회귀 점검(수동, 하드웨어/시뮬)**

가상 모드로 실행해 다음 확인. Run:
```bash
cd ~/cobot_ws && source install/setup.bash
ros2 run ros2_move_recoder gui
```
체크리스트(스펙 §6): (a) 녹화→스무딩→재생 정상 (b) DualSense 조그/매크로 (c) bringup/모드전환 (d) 그리퍼 타임라인 재생 (e) abort 버튼이 모션 즉시 정지. 이상 시 Step 1 수정.

- [ ] **Step 3: 커밋**

```bash
cd ~/cobot_ws/src/cobot1/common/ros2_move_recoder
git add ros2_move_recoder/gui.py
git commit -m "refactor(gui): route play + gripper timeline through playback core (dedupe)"
```

---

## Phase 2 — robo_chef 순수 코어

### Task 5: `order_core.py` — pending 선택·items→jobs 전개·상태 전이

**Files:**
- Create: `main_side/robo_chef/nodes/order_core.py`
- Test: `main_side/robo_chef/test/test_order_core.py`

- [ ] **Step 1: 실패 테스트 작성**

`main_side/robo_chef/test/test_order_core.py`:
```python
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "nodes"))
import order_core as oc


def test_select_next_pending_fifo_by_order_time():
    orders = {
        "B": {"status": "pending", "order_time": "2026-04-26T04:34:10Z"},
        "A": {"status": "pending", "order_time": "2026-04-26T04:34:00Z"},
        "C": {"status": "delivered", "order_time": "2026-04-26T04:33:00Z"},
    }
    oid, order = oc.select_next_pending(orders)
    assert oid == "A"


def test_select_next_pending_none_when_no_pending():
    assert oc.select_next_pending({"A": {"status": "delivered"}}) == (None, None)


def test_build_jobs_expands_items_in_order_with_segments():
    order = {
        "items": [
            {"recipe_id": "RAMEN", "qty": 2},
            {"recipe_id": "STEAK", "qty": 1},
        ],
        "recipe_data": {
            "RAMEN": {"segments": ["r1", "r2"]},
            "STEAK": {"segments": ["s1"]},
        },
    }
    jobs = oc.build_jobs(order)
    assert jobs == [
        {"recipe_id": "RAMEN", "qty": 2, "segments": ["r1", "r2"]},
        {"recipe_id": "STEAK", "qty": 1, "segments": ["s1"]},
    ]


def test_build_jobs_raises_when_segments_missing():
    order = {"items": [{"recipe_id": "X", "qty": 1}], "recipe_data": {"X": {}}}
    try:
        oc.build_jobs(order)
        assert False, "should raise"
    except oc.OrderError as e:
        assert "segments" in str(e)


def test_status_transition_for_cooking_status():
    assert oc.order_transition("DONE") == ("delivered", True)
    assert oc.order_transition("ERROR") == ("failed", False)
    assert oc.order_transition("EXECUTING") == (None, None)
    assert oc.order_transition("IDLE") == (None, True)
```

- [ ] **Step 2: 실패 확인**

Run: `cd ~/cobot_ws/src/cobot1/main_side/robo_chef && python -m pytest test/test_order_core.py -v`
Expected: FAIL — `ModuleNotFoundError: order_core`

- [ ] **Step 3: order_core.py 구현**

`main_side/robo_chef/nodes/order_core.py`:
```python
"""순수 주문 로직 — ROS/Firebase 무관, 단위테스트 대상.

select_next_pending : FIFO(order_time) 로 다음 pending 주문 선택
build_jobs          : order.items[] 를 jobs 로 전개 (recipe_data.segments 결합)
order_transition    : /cooking_status.state → (/orders status, busy 해제 여부)
"""
from __future__ import annotations


class OrderError(Exception):
    pass


def select_next_pending(orders: dict):
    pend = [(oid, o) for oid, o in (orders or {}).items()
            if isinstance(o, dict) and o.get("status") == "pending"]
    if not pend:
        return (None, None)
    pend.sort(key=lambda x: x[1].get("order_time", ""))
    return pend[0]


def build_jobs(order: dict) -> list:
    items = order.get("items") or []
    rdata = order.get("recipe_data") or {}
    jobs = []
    for it in items:
        rid = it.get("recipe_id")
        qty = int(it.get("qty", 1))
        segs = ((rdata.get(rid) or {}).get("segments")) or []
        if not segs:
            raise OrderError(f"no segments for recipe_id={rid}")
        jobs.append({"recipe_id": rid, "qty": qty, "segments": list(segs)})
    if not jobs:
        raise OrderError("order has no items")
    return jobs


def order_transition(state: str):
    """반환: (orders status 새 값 | None, busy 해제 | None)
    None busy = '변화 없음'. True = 해제(다음 pending 진행). False = 유지."""
    if state == "DONE":
        return ("delivered", True)
    if state == "ERROR":
        return ("failed", False)   # 주문은 failed 종결, 그러나 런너 ERROR → busy 유지
    if state == "IDLE":
        return (None, True)        # unlock 재개 신호
    return (None, None)            # EXECUTING 등
```

- [ ] **Step 4: 통과 확인**

Run: `cd ~/cobot_ws/src/cobot1/main_side/robo_chef && python -m pytest test/test_order_core.py -v`
Expected: PASS (5 passed)

- [ ] **Step 5: 커밋**

```bash
cd ~/cobot_ws/src/cobot1/main_side/robo_chef
git add nodes/order_core.py test/test_order_core.py
git commit -m "feat(order_core): pure pending-select / jobs-expand / status-transition"
```

### Task 6: `cooking_core.py` — jobs 실행 루프 + 상태 dict 빌더

**Files:**
- Create: `main_side/robo_chef/nodes/cooking_core.py`
- Test: `main_side/robo_chef/test/test_cooking_core.py`

- [ ] **Step 1: 실패 테스트 작성**

`main_side/robo_chef/test/test_cooking_core.py`:
```python
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "nodes"))
import cooking_core as cc


def test_run_jobs_emits_status_sequence_and_completes():
    jobs = [
        {"recipe_id": "RAMEN", "qty": 2, "segments": ["r1"]},
        {"recipe_id": "STEAK", "qty": 1, "segments": ["s1", "s2"]},
    ]
    emitted = []
    played = []

    def play(seg):
        played.append(seg)
        return True  # 성공

    final = cc.run_jobs("ORD1", jobs, play_fn=play,
                         emit_fn=lambda s: emitted.append(s),
                         seg_path=lambda s: s)
    # 재생 호출: r1,r1, s1,s2  (qty/segment 전개)
    assert played == ["r1", "r1", "s1", "s2"]
    # 마지막 emit 은 DONE
    assert emitted[-1]["state"] == "DONE"
    assert final["state"] == "DONE"
    # EXECUTING 한 건 스키마 확인
    ex = next(s for s in emitted if s["state"] == "EXECUTING")
    assert set(ex) == {"state", "order_id", "recipe_id", "item_index",
                       "item_total", "qty_index", "qty_total",
                       "segment_name", "segment_index", "segment_total",
                       "error_msg"}
    assert ex["order_id"] == "ORD1"


def test_run_jobs_stops_on_failure_with_error_state():
    jobs = [{"recipe_id": "RAMEN", "qty": 1, "segments": ["r1", "r2"]}]
    emitted = []

    def play(seg):
        return seg != "r2"   # r2 실패

    final = cc.run_jobs("ORD2", jobs, play_fn=play,
                         emit_fn=lambda s: emitted.append(s),
                         seg_path=lambda s: s)
    assert final["state"] == "ERROR"
    assert final["segment_name"] == "r2"
    assert emitted[-1]["state"] == "ERROR"
    assert "r2" in emitted[-1]["error_msg"]
```

- [ ] **Step 2: 실패 확인**

Run: `cd ~/cobot_ws/src/cobot1/main_side/robo_chef && python -m pytest test/test_cooking_core.py -v`
Expected: FAIL — `ModuleNotFoundError: cooking_core`

- [ ] **Step 3: cooking_core.py 구현**

`main_side/robo_chef/nodes/cooking_core.py`:
```python
"""순수 조리 실행 로직 — ROS/DSR 무관, 단위테스트 대상.

run_jobs(order_id, jobs, play_fn, emit_fn, seg_path) :
  jobs 를 item→qty→segment 3중 루프로 재생. 세그먼트마다 emit_fn 으로
  /cooking_status dict 발행. play_fn(seg_path(seg))->bool. 실패 시 ERROR.
"""
from __future__ import annotations


def _status(state, order_id, recipe_id="", item_index=0, item_total=0,
            qty_index=0, qty_total=0, segment_name="", segment_index=0,
            segment_total=0, error_msg=""):
    return {
        "state": state, "order_id": order_id, "recipe_id": recipe_id,
        "item_index": item_index, "item_total": item_total,
        "qty_index": qty_index, "qty_total": qty_total,
        "segment_name": segment_name, "segment_index": segment_index,
        "segment_total": segment_total, "error_msg": error_msg,
    }


def run_jobs(order_id, jobs, *, play_fn, emit_fn, seg_path):
    m = len(jobs)
    for i, job in enumerate(jobs, 1):
        rid = job["recipe_id"]
        segs = job["segments"]
        qn = int(job.get("qty", 1))
        s_total = len(segs)
        for q in range(1, qn + 1):
            for k, seg in enumerate(segs, 1):
                st = _status("EXECUTING", order_id, rid, i, m, q, qn,
                             seg, k, s_total)
                emit_fn(st)
                if not play_fn(seg_path(seg)):
                    err = _status("ERROR", order_id, rid, i, m, q, qn,
                                  seg, k, s_total,
                                  error_msg=f"segment failed: {seg}")
                    emit_fn(err)
                    return err
    done = _status("DONE", order_id)
    emit_fn(done)
    return done
```

- [ ] **Step 4: 통과 확인**

Run: `cd ~/cobot_ws/src/cobot1/main_side/robo_chef && python -m pytest test/test_cooking_core.py -v`
Expected: PASS (2 passed)

- [ ] **Step 5: 커밋**

```bash
cd ~/cobot_ws/src/cobot1/main_side/robo_chef
git add nodes/cooking_core.py test/test_cooking_core.py
git commit -m "feat(cooking_core): pure item/qty/segment run loop + status builder"
```

---

## Phase 3 — robo_chef 노드

### Task 7: `sequence_runner.py` 노드

**Files:**
- Create: `main_side/robo_chef/nodes/sequence_runner.py`

- [ ] **Step 1: 노드 구현**

`main_side/robo_chef/nodes/sequence_runner.py`:
```python
"""sequence_runner — /recipe(주문 잡) 수신 → 세그먼트 체인 재생.

DSR 소유 노드. 재생은 ros2_move_recoder.playback.play_segment 위임.
실패 시 즉시 정지 + ERROR, unlock_system(Trigger) 으로만 복귀.
"""
import os
import json
import threading

import rclpy
import DR_init
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from std_srvs.srv import Trigger

from ros2_move_recoder.playback import play_segment
import cooking_core as cc

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"
RECORDS_DIR = os.path.expanduser("~/cobot_ws/src/ros2_move_recoder/records")

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


class SequenceRunner(Node):
    def __init__(self):
        super().__init__("sequence_runner")
        self.state = "IDLE"
        self._abort = threading.Event()
        self._gripper = self._init_gripper()
        self.init_dsr()
        self.status_pub = self.create_publisher(String, "/cooking_status", 10)
        self.create_subscription(String, "/recipe", self._on_recipe, 10)
        self.create_service(Trigger, "unlock_system", self._on_unlock)
        self.get_logger().info("🍳 sequence_runner ready (state=IDLE)")

    def _init_gripper(self):
        try:
            from ros2_move_recoder.onrobot import RG
            ip = os.environ.get("GRIPPER_IP", "192.168.1.1")
            port = int(os.environ.get("GRIPPER_PORT", "502"))
            gtype = os.environ.get("GRIPPER_TYPE", "rg2")
            return RG(gtype, ip, port)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"그리퍼 init 실패(그리퍼 없이 진행): {e}")
            return None

    def init_dsr(self):
        from DSR_ROBOT2 import (set_tool, set_tcp, ROBOT_MODE_MANUAL,
                                ROBOT_MODE_AUTONOMOUS, set_robot_mode)
        set_robot_mode(ROBOT_MODE_MANUAL)
        set_tool(ROBOT_TOOL)
        set_tcp(ROBOT_TCP)
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    def _emit(self, status: dict):
        m = String()
        m.data = json.dumps(status)
        self.status_pub.publish(m)

    def _seg_path(self, seg: str) -> str:
        return os.path.join(RECORDS_DIR, seg, "smooth.json")

    def _play(self, smooth_path: str) -> bool:
        res = play_segment(smooth_path, gripper=self._gripper,
                           require_autonomous=True, abort_event=self._abort,
                           logger=self.get_logger())
        return res.ok

    def _on_recipe(self, msg: String):
        if self.state != "IDLE":
            self.get_logger().warn("state!=IDLE — /recipe 무시")
            return
        try:
            job = json.loads(msg.data)
            order_id = job["order_id"]
            jobs = job["jobs"]
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"잘못된 /recipe: {e}")
            return
        self.state = "EXECUTING"
        self._abort.clear()
        final = cc.run_jobs(order_id, jobs, play_fn=self._play,
                            emit_fn=self._emit, seg_path=self._seg_path)
        self.state = "ERROR" if final["state"] == "ERROR" else "IDLE"

    def _on_unlock(self, request, response):
        if self.state == "ERROR":
            self.state = "IDLE"
            self._emit({"state": "IDLE", "order_id": "", "recipe_id": "",
                        "item_index": 0, "item_total": 0, "qty_index": 0,
                        "qty_total": 0, "segment_name": "",
                        "segment_index": 0, "segment_total": 0,
                        "error_msg": ""})
            response.success = True
            response.message = "unlocked → IDLE"
        else:
            response.success = False
            response.message = f"state={self.state} (ERROR 아님)"
        return response


def main(args=None):
    rclpy.init(args=args)
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    dsr_node = Node("dsr_helper_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = dsr_node
    try:
        import DSR_ROBOT2  # noqa: F401
    except Exception as e:  # noqa: BLE001
        print(f"DSR_ROBOT2 Load Error: {e}")
    executor = MultiThreadedExecutor(num_threads=4)
    node = SequenceRunner()
    executor.add_node(dsr_node)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._abort.set()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: 문법/구조 스모크**

Run: `cd ~/cobot_ws/src/cobot1/main_side/robo_chef && python -c "import ast; ast.parse(open('nodes/sequence_runner.py').read()); print('OK')"`
Expected: `OK`

- [ ] **Step 3: 커밋**

```bash
cd ~/cobot_ws/src/cobot1/main_side/robo_chef
git add nodes/sequence_runner.py
git commit -m "feat(sequence_runner): DSR-owning node wiring cooking_core + playback"
```

### Task 8: `firebase_bridge.py` 재작성 (/orders 감지·전개·상태 전이)

**Files:**
- Modify (전체 교체): `main_side/robo_chef/nodes/firebase_bridge.py`

- [ ] **Step 1: 재작성**

`main_side/robo_chef/nodes/firebase_bridge.py`:
```python
"""firebase_bridge — RTDB /orders(pending) 감지 → /recipe 잡 발행,
/cooking_status → robot_status + /orders status 전이.

order_count 방식 폐기. 동시 1건(busy), order_time FIFO.
"""
import json
import datetime
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import firebase_admin
from firebase_admin import credentials, db

import order_core as oc

CRED_PATH = "/home/kibeom/cobot_ws/src/robo_chef/config/serviceAccountKey.json"
DB_URL = "https://robochef-5d9b6-default-rtdb.asia-southeast1.firebasedatabase.app"


class FirebaseBridge(Node):
    def __init__(self):
        super().__init__("firebase_bridge")
        self.recipe_pub = self.create_publisher(String, "/recipe", 10)
        self.create_subscription(String, "/cooking_status",
                                 self._on_status, 10)
        self._busy = False
        self._cur_order = None
        self._lock = threading.Lock()
        try:
            cred = credentials.Certificate(CRED_PATH)
            firebase_admin.initialize_app(cred, {"databaseURL": DB_URL})
            self.get_logger().info("✅ Firebase Connected")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"❌ Firebase init 실패: {e}")
        self.orders_ref = db.reference("orders")
        self.status_ref = db.reference("robot_status")
        self.log_ref = db.reference("error_logs")
        self._try_dispatch_next()                      # 기동 시 1회 스캔
        self.orders_ref.listen(self._on_orders_event)  # 이후 실시간

    # ---- 주문 인지/디스패치 ----
    def _on_orders_event(self, event):
        self._try_dispatch_next()

    def _try_dispatch_next(self):
        with self._lock:
            if self._busy:
                return
            orders = self.orders_ref.get() or {}
            oid, order = oc.select_next_pending(orders)
            if not oid:
                return
            try:
                jobs = oc.build_jobs(order)
            except oc.OrderError as e:
                self.get_logger().error(f"주문 {oid} 전개 실패: {e}")
                db.reference(f"orders/{oid}").update(
                    {"status": "failed", "error_msg": str(e)})
                return
            self._busy = True
            self._cur_order = oid
            db.reference(f"orders/{oid}").update(
                {"status": "cooking", "started_at": _now()})
            msg = String()
            msg.data = json.dumps({"order_id": oid, "jobs": jobs})
            self.recipe_pub.publish(msg)
            self.get_logger().info(f"▶️ 주문 {oid} 디스패치 ({len(jobs)} jobs)")

    # ---- 상태 수신 ----
    def _on_status(self, msg: String):
        try:
            st = json.loads(msg.data)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"잘못된 /cooking_status: {e}")
            return
        self.status_ref.set(st)
        new_status, release = oc.order_transition(st.get("state", ""))
        oid = st.get("order_id") or self._cur_order
        if new_status and oid:
            extra = {"delivered_at": _now()} if new_status == "delivered" \
                else {"error_msg": st.get("error_msg", "")}
            db.reference(f"orders/{oid}").update(
                {"status": new_status, **extra})
            if new_status == "failed":
                self.log_ref.push({
                    "timestamp": _now(), "order_id": oid,
                    "recipe_id": st.get("recipe_id", ""),
                    "item_index": st.get("item_index", 0),
                    "segment_name": st.get("segment_name", ""),
                    "message": st.get("error_msg", "")})
        if release:                       # DONE 또는 IDLE(unlock 재개)
            with self._lock:
                self._busy = False
                self._cur_order = None
            self._try_dispatch_next()


def _now():
    return datetime.datetime.utcnow().isoformat() + "Z"


def main(args=None):
    rclpy.init(args=args)
    node = FirebaseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 2: 문법 스모크**

Run: `cd ~/cobot_ws/src/cobot1/main_side/robo_chef && python -c "import ast; ast.parse(open('nodes/firebase_bridge.py').read()); print('OK')"`
Expected: `OK`

- [ ] **Step 3: 커밋**

```bash
cd ~/cobot_ws/src/cobot1/main_side/robo_chef
git add nodes/firebase_bridge.py
git commit -m "feat(firebase_bridge): /orders pending detection + composite expand + status transitions"
```

### Task 9: setup.py / package.xml 갱신

**Files:**
- Modify: `main_side/robo_chef/setup.py`, `main_side/robo_chef/package.xml`

- [ ] **Step 1: setup.py entry_points 교체**

`setup.py` 의 `entry_points` 블록을 교체:
```python
    entry_points={
        'console_scripts': [
            'firebase_bridge = nodes.firebase_bridge:main',
            'sequence_runner = nodes.sequence_runner:main',
        ],
    },
```

- [ ] **Step 2: package.xml 의존성 갱신**

`<test_depend>` 블록 뒤(`<export>` 앞)에 추가:
```xml
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>std_srvs</exec_depend>
  <exec_depend>ros2_move_recoder</exec_depend>
```
(기존 `recipe_msgs` 의존 태그는 원래 없으므로 제거할 것 없음 — 코드 임포트만 삭제로 해소됨.)

- [ ] **Step 3: 빌드 확인**

Run: `cd ~/cobot_ws && colcon build --packages-select ros2_move_recoder robo_chef --symlink-install 2>&1 | tail -5`
Expected: `Summary: 2 packages finished` (실패 시 누락 의존/임포트 경로 수정)

- [ ] **Step 4: 커밋**

```bash
cd ~/cobot_ws/src/cobot1/main_side/robo_chef
git add setup.py package.xml
git commit -m "chore(robo_chef): entry_points → firebase_bridge/sequence_runner, declare deps"
```

---

## Phase 4 — web 패널 & 정리

### Task 10: customer_status 패널 신 스키마 렌더

**Files:**
- Modify: `sub1_side/web/panel/customer_status/index.html`

- [ ] **Step 1: /robot_status 리스너 교체**

`index.html` 의 `db.ref('/robot_status').on('value', ...)` 콜백(스펙에서 확인된 ~2253행 블록)을 신 스키마 기반으로 교체. 기존 `current_step/total_steps` 6-step 하드코딩 파생 삭제, 아래 로직으로 대체:
```javascript
db.ref('/robot_status').on('value', snap => {
  const rs = snap.val(); if (!rs || typeof rs.state !== 'string') return;
  const state = rs.state;
  if (state.startsWith('ERROR')) { _onPhase('error'); return; }
  if (state === 'IDLE')  { _onPhase('ready'); return; }
  if (state === 'DONE')  { _renderDone(rs); return; }
  if (state === 'EXECUTING') {
    _renderProgress({
      menu:    rs.recipe_id || '',
      itemIdx: Number(rs.item_index) || 0,
      itemTot: Number(rs.item_total) || 0,
      qtyIdx:  Number(rs.qty_index) || 0,
      qtyTot:  Number(rs.qty_total) || 0,
      segName: rs.segment_name || '',
      segIdx:  Number(rs.segment_index) || 0,
      segTot:  Number(rs.segment_total) || 0,
    });
  }
});
```
`_renderProgress(p)`: 현재 메뉴(`p.menu`), 항목 `p.itemIdx/p.itemTot`, 수량 `p.qtyIdx/p.qtyTot`, 세그먼트 진행바 `p.segIdx/p.segTot` 와 `p.segName` 라벨을 표시(기존 step-panel DOM 재사용, 6 고정 배지 제거 → `p.segTot` 동적). `_renderDone(rs)`: 완료 연출. `_onPhase` 는 기존 함수 재사용. `/orders` 최신 1건·`/phase` 기존 리스너는 유지.

- [ ] **Step 2: 수동 렌더 확인**

Run: `cd ~/cobot_ws/src/cobot1/sub1_side/web/panel && python3 -m http.server 3002`
브라우저 `localhost:3002/customer_status/` → RTDB `robot_status` 에 샘플
`{"state":"EXECUTING","recipe_id":"RAMEN","item_index":1,"item_total":2,"qty_index":1,"qty_total":2,"segment_name":"ramen_stir","segment_index":3,"segment_total":4,"error_msg":""}`
주입 시 진행바/라벨 정상, `state:"ERROR..."` 시 에러 오버레이 확인.

- [ ] **Step 3: 변경 기록 (web은 git 비저장소)**

`sub1_side` 는 git 저장소가 아님 → 커밋 생략. 변경 요약을 작업 로그로 남기고 다음 Task 진행.

### Task 11: recipe_seeder segments 필드 + app.py/start_all 죽은코드 제거

**Files:**
- Modify: `sub1_side/web/backend/recipe_seeder.py`, `sub1_side/web/backend/app.py`, `sub1_side/web/start_all.sh`

- [ ] **Step 1: recipe_seeder.py — 시드 레시피에 segments 추가**

각 시드 레시피 dict(`KIMCHI_STEW_001`, `RAMEN_001`)에 다음 키 추가:
```python
    # TODO(운영): GUI 녹화 후 실제 세그먼트명으로 교체할 것. (코드 동작과 무관한 시드 값)
    "segments": [],
```

- [ ] **Step 2: app.py 죽은 코드/라우트 제거**

`app.py` 에서 다음을 삭제: 함수 `_ros2_publish_order`, `_start_order_watcher`, `_process_order`, 전역 `_processing_orders`, 그리고 `_ros2_publish_order` 에만 의존하는 `/api/ros2/publish_order` 라우트 핸들러. `startup()` 이 이들을 호출하지 않음(스펙 §7 확인됨)을 재확인.

- [ ] **Step 3: start_all.sh — ros2_order_publisher 기동 라인 제거**

`start_all.sh` 에서 `run_ros2_publisher.sh`/`ros2_order_publisher` 를 nohup 으로 기동하는 라인과, 종료부의 해당 프로세스 kill 라인을 삭제.

- [ ] **Step 4: 스모크**

Run: `cd ~/cobot_ws/src/cobot1/sub1_side/web/backend && python -c "import ast; [ast.parse(open(f).read()) for f in ('app.py','recipe_seeder.py')]; print('OK')"`
Expected: `OK`

- [ ] **Step 5: 변경 기록** (web git 비저장소 — 커밋 생략)

### Task 12: 삭제 일괄 수행 + 영향도 재검증

**Files (삭제):**
- robo_chef: `nodes/recipe_parser.py`, `nodes/state_manager.py`, `nodes/executer.py`, `nodes/recipe_tester.py`, `core/`(전체), `data/`(전체), `src/interfaces/recipe_msgs/`(전체), 중복 `src/robo_chef/`(전체), 삭제분 종속 `test/RoboChef.py` 등
- web: `backend/ros2_order_publisher.py`, `backend/ros2_srv_call.py`, `backend/run_ros2_publisher.sh`

- [ ] **Step 1: 잔존 참조 사전 스캔 (삭제 전 안전 확인)**

Run:
```bash
cd ~/cobot_ws/src/cobot1
grep -rEn "recipe_msgs|core\.action_manager|action_manager|recipe_parser|state_manager|RecipeExecuter|ros2_order_publisher|ros2_srv_call" \
  --include=*.py --include=*.xml --include=*.sh --include=*.html \
  main_side/robo_chef/nodes sub1_side/web 2>/dev/null | grep -v "/src/robo_chef/"
```
Expected: 신규 파일(`sequence_runner.py`,`firebase_bridge.py`,`*_core.py`)에서 매칭 **없음**. 매칭이 있으면 그 참조부터 정리.

- [ ] **Step 2: 삭제 실행**

```bash
cd ~/cobot_ws/src/cobot1/main_side/robo_chef
git rm -r nodes/recipe_parser.py nodes/state_manager.py nodes/executer.py \
         nodes/recipe_tester.py core data src/interfaces/recipe_msgs src/robo_chef
git rm test/RoboChef.py 2>/dev/null || true
cd ~/cobot_ws/src/cobot1/sub1_side/web
rm -f backend/ros2_order_publisher.py backend/ros2_srv_call.py backend/run_ros2_publisher.sh
```

- [ ] **Step 3: 빌드 + 전체 테스트 재검증**

```bash
cd ~/cobot_ws && colcon build --packages-select ros2_move_recoder robo_chef --symlink-install 2>&1 | tail -3
cd ~/cobot_ws/src/cobot1/common/ros2_move_recoder && PYTHONPATH=. python -m pytest test/ -q
cd ~/cobot_ws/src/cobot1/main_side/robo_chef && python -m pytest test/ -q
```
Expected: 빌드 2 packages finished, 모든 pytest PASS.

- [ ] **Step 4: 영향도 grep 스윕 (스펙 §6)**

Run:
```bash
cd ~/cobot_ws/src/cobot1
grep -rEn "/recipe|/cooking_status|order_count|unlock_system" \
  --include=*.py --include=*.html main_side sub1_side | grep -vE "/src/robo_chef/|test/"
```
기대: `/recipe`·`/cooking_status` 는 신규 노드와 firebase_bridge 에서만, `order_count` 는 키오스크 dead write(소비자 0)만, `unlock_system` 은 sequence_runner 에서만. 결과를 작업 로그에 기록.

- [ ] **Step 5: 커밋**

```bash
cd ~/cobot_ws/src/cobot1/main_side/robo_chef
git add -A && git commit -m "chore: remove dynamic engine, duplicate src tree, dead web order code"
```

---

## Self-Review (작성자 점검 결과)

- **스펙 커버리지**: §3.1→T1-2, §3.2 sequence_runner→T7, §3.3 firebase_bridge→T8, §3.4 customer_status→T10, §3.5 seeder→T11, player/gui 리팩터→T3-4, 순수 분리(테스트성)→T5-6, 삭제 §7→T12, setup/package §7→T9. 전 항목 매핑됨.
- **상태 전이 정합**: `order_core.order_transition` 의 `IDLE→(None,True)`/`DONE→("delivered",True)`/`ERROR→("failed",False)` 가 firebase_bridge `_on_status` 의 busy 해제 규칙(`release` True 시 해제)과, sequence_runner unlock 의 `IDLE` 발행과 일치(스펙 §3.2/§3.3 수정분 반영).
- **타입/이름 일관성**: `play_segment(...)->PlayResult.ok`, `cc.run_jobs(order_id, jobs, *, play_fn, emit_fn, seg_path)`, status dict 키 11개가 T6/T7/T10 전반 동일.
- **플레이스홀더**: 신규 코드 전량 실제 코드 제공. T2/T4 는 기존 `gui.py` 의 명시된 라인범위(2721-2930)를 이식하는 정밀 리팩터(공개 API·테스트 완비) — placeholder 아님. T11 `segments:[]` 는 의도된 운영 시드값(주석 명시).
- **보류 명시**: `RECORDS_DIR` 경로 정합은 스펙 §1.3/§10 지시대로 보류(코어는 절대경로 인자로 회피).

---

## 미커밋 영역 주의
`sub1_side/web` 은 git 저장소가 아님 → Task 10/11 변경은 커밋되지 않음. 작업 로그에 변경 파일·요약을 남길 것. cobot1 루트도 비저장소(스펙 §10). 하위 저장소(`common/ros2_move_recoder`, `main_side/robo_chef`)만 커밋 대상.
