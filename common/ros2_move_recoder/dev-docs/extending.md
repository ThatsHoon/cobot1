# Extending — 새 기능 추가 가이드

## 새 모션 함수 추가

예: `move_periodic` (진동 모션) 추가.

### 1. DsrWorker 에 슬롯 등록

```python
# gui.py — DsrWorker 클래스
@QtCore.pyqtSlot(list, float, int)   # amp, period, repeat
def vibrate(self, amp, period, repeat):
    if self._busy:
        self.log.emit("[dsr] ⚠️ busy")
        return
    self._busy = True
    try:
        self._ensure_dsr()
        from DSR_ROBOT2 import move_periodic, DR_TOOL
        self.log.emit(f"[dsr] vibrate amp={amp} period={period}")
        move_periodic(amp=amp, period=period, atime=0.2, repeat=repeat, ref=DR_TOOL)
        self.log.emit("[dsr] vibrate done")
    finally:
        self._busy = False
```

### 2. MainWindow 시그널 + 버튼

```python
class MainWindow(QtWidgets.QMainWindow):
    request_vibrate = QtCore.pyqtSignal(list, float, int)
    ...
    def __init__(self):
        ...
        self.request_vibrate.connect(self.dsr.vibrate)

    def _build_ui(self):
        ...
        self.btn_vibrate = self._mkbtn("∿ Vibrate", "#9b59b6", self._on_vibrate)
        btn_row.addWidget(self.btn_vibrate)

    def _on_vibrate(self):
        # 파라미터 입력 다이얼로그 또는 spinbox 값 사용
        amp = [10, 0, 0, 0, 30, 0]
        self.request_vibrate.emit(amp, 1.0, 5)
```

**규칙**:
- DSR 호출은 **반드시** `DsrWorker` 안에서. 메인 스레드 호출 금지.
- `_busy` 가드 필수.
- 시그널 인자는 PyQt5 지원 타입만 (list, str, int, float, dict 등 OK; numpy array는 list로 변환).

## 새 평활화 알고리즘 추가

`smoother.py` + `gui.py` 의 `smooth_and_save()` 두 곳 동기 유지 필요. 또는 공통 모듈로 분리.

### 분리 권장 구조

```
ros2_move_recoder/
├── ros2_move_recoder/
│   ├── smooth_lib.py          ← 새로 만듦, smooth_and_save() 옮김
│   ├── smoother.py            ← argparse + smooth_lib.smooth_and_save() 호출
│   └── gui.py                 ← from .smooth_lib import smooth_and_save
```

### 새 알고리즘 추가 패턴

```python
# smooth_lib.py
def smooth_and_save(name, *, algorithm="savgol_arclength", **kwargs):
    if algorithm == "savgol_arclength":
        return _savgol_arclength(name, **kwargs)
    elif algorithm == "bspline_uniform":
        return _bspline_uniform(name, **kwargs)
    else:
        raise ValueError(f"unknown algorithm: {algorithm}")
```

GUI 콤보박스 추가:
```python
self.cmb_algo = QtWidgets.QComboBox()
self.cmb_algo.addItems(["savgol_arclength", "bspline_uniform"])
plw.addRow("알고리즘:", self.cmb_algo)
```

## 새 토픽 구독

예: 외부 force/torque 센서 `/wrench` 구독해서 GUI에 표시.

### 1. MacroNode 에 subscriber 추가

```python
class MacroNode(Node):
    def __init__(self, on_joint, on_wrench):
        ...
        self.create_subscription(
            WrenchStamped, "/wrench",
            self._handle_wrench, qos, callback_group=self.cb_group)

    def _handle_wrench(self, msg):
        f = msg.wrench.force
        t = msg.wrench.torque
        self._on_wrench([f.x, f.y, f.z, t.x, t.y, t.z])
```

### 2. RosSpinThread 에 시그널 추가

```python
class RosSpinThread(QtCore.QThread):
    joint_received  = QtCore.pyqtSignal(list)
    wrench_received = QtCore.pyqtSignal(list)
    ready           = QtCore.pyqtSignal()

    def run(self):
        ...
        self.node = MacroNode(
            on_joint  = lambda j: self.joint_received.emit(j),
            on_wrench = lambda w: self.wrench_received.emit(w))
```

### 3. MainWindow 위젯 + 슬롯

```python
def __init__(self):
    ...
    self.ros.wrench_received.connect(self._on_wrench)

def _on_wrench(self, w):
    # UI 업데이트
    ...
```

## 새 entry point 추가

`setup.py`:
```python
entry_points={
    'console_scripts': [
        ...
        'analyzer = ros2_move_recoder.analyzer:main',   # 추가
    ],
},
```

`ros2_move_recoder/analyzer.py`:
```python
"""raw.json 분석 + 통계 출력"""
def main():
    ...

if __name__ == "__main__":
    main()
```

빌드:
```bash
colcon build --packages-select ros2_move_recoder --symlink-install
source install/setup.bash
ros2 run ros2_move_recoder analyzer my_first
```

## 새 파라미터를 GUI 에 노출

`MainWindow._build_ui()` 의 "파라미터" GroupBox 에 추가:
```python
self.spin_new_param = QtWidgets.QDoubleSpinBox()
self.spin_new_param.setRange(0.0, 100.0)
self.spin_new_param.setValue(10.0)
plw.addRow("새 파라미터:", self.spin_new_param)
```

`_on_smooth()` 에서 `smooth_and_save(..., new_param=self.spin_new_param.value())` 로 전달.

## 다른 로봇 모델 지원

현재 `m0609` 하드코딩. 다른 모델로 바꾸려면:

1. `gui.py:42`, `run.py:12`, `player.py:18` 의 `ROBOT_MODEL = "m0609"` 변경
2. `BringupManager.HOST_BY_MODE` 의 IP가 해당 로봇과 맞는지 확인
3. URDF 파일이 새 모델용인지 확인 (`dsr_description2`)

⚠️ multi-model 동시 지원은 DSR_ROBOT2가 multi-robot 미지원이라 어려움. 모델 전환은 GUI 재시작이 필요.

## 다른 namespace 사용

`ROBOT_ID = "dsr01"` 변경:
1. 모든 `.py` 파일의 `ROBOT_ID` 동시 변경
2. bringup launch도 동일하게 (`name:=dsr02` 등)
3. 토픽 경로 하드코딩(`/dsr01/joint_states`) 다 갱신 — `gui.py:93`, `gui.py:832`, `recorder.py:46`

또는 환경변수로 빼는 리팩터:
```python
ROBOT_ID = os.environ.get("DSR_ID", "dsr01")
JOINT_TOPIC = f"/{ROBOT_ID}/joint_states"
```

## 안전 가드 추가 패턴

새 모션 추가 시 사전 체크 권장:
```python
def my_new_motion(self, ...):
    if self._busy:
        self.log.emit("[dsr] busy"); return
    self._busy = True
    try:
        self._ensure_dsr()
        if not self._service_ready("get_robot_mode", timeout=1.0):
            self.log.emit("[dsr] controller not ready"); return
        if self._fns["get_robot_mode"]() != self._fns["MODE_AUTONOMOUS"]:
            self.log.emit("[dsr] not in AUTO mode"); return
        # ... 실제 모션
    finally:
        self._busy = False
```

이 4개 체크(`_busy`, `_ensure_dsr`, `_service_ready`, `mode==AUTO`)는 **모든** DSR 호출 슬롯의 표준.

## 테스트 작성

ament_python 프로젝트는 `test/` 폴더에 pytest 작성:
```
ros2_move_recoder/
└── test/
    └── test_smoother.py
```

```python
# test/test_smoother.py
import json
import tempfile
from pathlib import Path
import numpy as np
from ros2_move_recoder.gui import smooth_and_save

def test_smooth_basic(tmp_path, monkeypatch):
    # records 디렉토리 가짜로 만들기
    records = tmp_path / "records"
    monkeypatch.setattr(
        "ros2_move_recoder.gui.RECORDS_DIR", records)
    action_dir = records / "test_action"
    action_dir.mkdir(parents=True)

    raw = {
        "joints_deg": [[i, 0, 90, 0, 90, 0] for i in range(100)],
        "timestamps_ms": list(range(0, 1000, 10)),
        "samples": 100, "duration_sec": 1.0, "rate_hz_avg": 100.0,
        "recorded_at": "2026-01-01T00:00:00Z",
    }
    (action_dir / "raw.json").write_text(json.dumps(raw))

    r = smooth_and_save("test_action", window=21, polyorder=3,
                        max_pts=20, vel=30, acc=60)
    assert r["n_waypoints"] <= 20
    assert (action_dir / "smooth.json").exists()
```

실행:
```bash
cd ~/cobot_ws/src/ros2_move_recoder
python3 -m pytest test/
```

ROS 통합 테스트는 `launch_testing` 사용 (별도 — `~/.claude/skills/ros2-architect/references/testing.md`).
