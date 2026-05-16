# DualSense 듀얼 매핑 jog 컨트롤러 추가 — 설계 계획

> **상태**: 설계안 (구현 전)
> **작성일**: 2026-04-28
> **목적**: 펜던트 없이 DualSense 컨트롤러만으로 m0609 6축 jog (joint + TCP) 가능

---

## Context

`ros2_move_recoder` 는 펜던트 → record → smooth → play 파이프라인이지만 **시연 자체를 컨트롤러로 직접 할 방법이 없다**. 사용자는 펜던트를 들고 로봇 옆에서 조작해야 한다.

`manage` 프로젝트의 `Manage_jog.py` 는 이미 게임패드로 6축 jog 를 구현해 둔 검증된 패턴이 있다 (`_JogDispatcher` latest-wins 큐 + DSR `jog()` API + 4-모드 axis 매핑). 이 패턴을 채택해서 **DualSense 컨트롤러로 joint 6축 + TCP 6축을 직접 jog 할 수 있는 인터페이스**를 `ros2_move_recoder` 에 추가한다. 기존 record GUI 와 통합되어 있어 "컨트롤러로 자세 잡고 → 그 자리에서 record 시작" 흐름이 자연스러워진다.

목표: 펜던트 없이 컨트롤러만으로 시연 가능.

---

## 매핑 설계 (manage 4-모드 패턴 채택 + 듀얼센스 전용 확장)

### 왜 4-모드 × 3-축 패턴인가
- 6축 동시 제어는 안전상 위험 (스틱 4개 축으로는 6축 정확 분리 불가)
- manage 가 25공정 운영에서 검증한 패턴 — 사용자 학습 비용 최소
- 펜던트의 "joint mode / cartesian mode" 멘탈 모델과 일치
- 모드별 lightbar 색상으로 시각 피드백 가능 (DualSense 전용 강점)

### 4개 모드

| 모드 | 축 매핑 (LX / LY / RY) | Lightbar | 단축 |
|---|---|---|---|
| **J123** (기본) | J1 base / J2 shoulder / J3 elbow | 청색 | □ |
| **J456** | J4 wrist roll / J5 wrist pitch / J6 tool roll | 시안 | △ |
| **XYZ** | X / Y / Z (BASE 좌표) | 녹색 | ○ |
| **ABC** | A yaw / B pitch / C roll | 황색 | × |

- DSR `jog(axis, ref, vel)` axis 매핑: J123→0~2, J456→3~5, XYZ→6~8 (ref=DR_BASE), ABC→9~11
- 한 번에 1축만 활성 (latest-wins) — manage 와 동일한 안전 모델

### 보조 입력 (모든 모드 공통)

| 입력 | 기능 |
|---|---|
| L2 / R2 (트리거) | 속도 배율 0%~100% — 두 트리거 합산. 둘 다 0이면 default 50% |
| L1 / R1 | 모드 직접 사이클 (← / →) |
| D-Pad ↑ / ↓ | 속도 default 미세 조정 (± 5%) |
| D-Pad ← / → | (예약) — 향후 step-jog 용 |
| Touchpad press | **즉시 정지** (`stop(DR_SSTOP)`) — 비상정지 |
| Options | AUTONOMOUS 모드 토글 (현재 `_request_set_mode` 재사용) |
| Share | 현재 자세 → record 시작 트리거 (선택, 기존 record 버튼과 연결) |
| PS | DualSense 연결 테스트 (햅틱 1회 펄스) |
| Gyro / Accel | **사용 안 함** — 의도치 않은 jog 방지 |

### Deadzone 및 응답 곡선
- 스틱 radial deadzone: **8%** (raw |stick| < 0.08 → 0)
- 응답 곡선: `vel = sign(s) × |s|^1.5 × VEL_MAX[mode]` — 중심부 미세제어 향상
- VEL_MAX 기본값: joint 모드 30°/s, XYZ 50mm/s, ABC 30°/s

### Watchdog (안전)
- 입력 폴링 thread 가 100ms 주기로 동작
- 마지막 stick 입력 후 250ms 무신호 → `dispatcher.stop()` 자동 발사
- 컨트롤러 disconnect 감지 → 즉시 stop + GUI 상태바 "⚠ 컨트롤러 연결 끊김"

---

## 듀얼센스 처리 방식

### 라이브러리 선택: **`pydualsense`** (hidraw 기반)
- 이유: lightbar 색상 + 햅틱 피드백 = 모드 전환 인지성 ↑, manage 가 쓴 `inputs` 보다 듀얼센스 전용 기능 풍부
- 대안 `pygame` / `inputs` 는 디지털 버튼/축만 노출하고 lightbar 출력 불가
- `pip install pydualsense` (의존성 `hidapi` — apt 로 설치)

### 폴링 vs 이벤트 모델
- pydualsense 는 콜백 등록 방식 → 별도 polling 루프 없이 백그라운드 스레드가 이벤트 발생
- Qt 시그널로 메인 스레드에 전달 (`Qt.QueuedConnection`) — Qt thread 안전

### 연결 방식
- **USB 권장** (블루투스는 BlueZ 5.69+ 재연결 불안정)
- 유저 권한: `udev` rule 또는 `/dev/hidraw*` 권한 (ros2_move_recoder 설치 가이드에 추가)

### 햅틱 피드백 (DualSense 전용 UX 강점)
- 모드 변경 시: 짧은 펄스 1회
- 비상정지 발동 시: 강한 진동 0.5초
- 관절 한계 근접 (DSR 에러 코드 수신) 시: 좌측 모터 진동
- AUTONOMOUS 전환 실패 시: 우측 모터 진동

### Lightbar 색상 (시각 피드백)
- 모드별 색상 (위 표) + 비상정지 시 빨강 깜빡임
- 컨트롤러 disconnect → GUI 만으로는 모드 모호 → lightbar 가 "지금 어느 모드에 있는지" 항상 표시

---

## 구현 계획

### 새 파일 / 수정 대상

**새 파일:**
- `ros2_move_recoder/dualsense_worker.py` — DualSense 입력 처리 + JogDispatcher (manage 의 `_JogDispatcher` L59-120 패턴 포팅)
  - 클래스 `DualSenseWorker(QObject)`: pydualsense 콜백 → Qt 시그널 변환
  - 클래스 `_JogDispatcher`: latest-wins target + 워커 스레드 (manage 와 동일 구조)
  - 시그널: `jog_request(axis: int, ref: int, vel: float)`, `mode_changed(str)`, `connection_changed(bool)`, `estop_pressed()`

**수정 파일:**

1. **`ros2_move_recoder/gui.py`**
   - DsrWorker 에 `jog(axis, ref, vel)` 슬롯 추가 (현재 미사용 — `from DSR_ROBOT2 import jog` 추가, `_ensure_dsr` 의 옵션 심볼 목록에 등록)
   - `MainWindow.__init__` 에 `DualSenseWorker` 인스턴스화 + 별도 QThread 에 moveToThread
   - 시그널 연결:
     ```
     ds_worker.jog_request    → dsr_worker.jog (Qt.QueuedConnection)
     ds_worker.estop_pressed  → dsr_worker.emergency_stop
     ds_worker.mode_changed   → 새 GUI 라벨 lbl_mode 갱신
     ds_worker.connection_changed → 상태바 갱신
     ```
   - 새 GroupBox "DualSense Jog" — 현재 모드 / 속도 % / 연결 상태 표시
   - 기존 `_busy` 플래그 활용해 jog 와 play 가 동시에 안 돌게 (jog 도중 play 거부)

2. **`setup.py`**
   - 새 entry point: `dualsense_jog = ros2_move_recoder.dualsense_jog:main` (CLI 단독 실행용, GUI 없이도 사용 가능)
   - 새 install_requires: `pydualsense`

3. **`package.xml`**
   - `<exec_depend>python3-hid</exec_depend>` (hidapi 의존성)

**문서:**

4. **`dev-docs/dualsense.md`** (신규) — 매핑 표, lightbar 색상, 햅틱 패턴, udev 규칙 설치, troubleshooting (블루투스 vs USB)
5. **`dev-docs/architecture.md`** 업데이트 — Qt 시그널 그래프에 DualSense thread 추가
6. **`dev-docs/extending.md`** 업데이트 — "새 컨트롤러 추가" 섹션
7. **`dev-docs/troubleshooting.md`** 업데이트 — 17, 18번 항목 (controller not detected, lightbar 안 켜짐)

### 재사용 패턴 (이미 코드베이스에 존재)

| 패턴 | 위치 | 재사용 방식 |
|---|---|---|
| latest-wins JogDispatcher | `manage/Manage_jog.py:59-120` | dualsense_worker.py 로 포팅 |
| `_dr_set_node()` + name mangling 회피 | `gui.py:49-50` | 그대로 사용 |
| `rclpy.__executor` 글로벌 등록 | `gui.py:454` | 그대로 사용 |
| `_busy` 플래그 동시성 가드 | `gui.py:204` | jog 와 play 상호 배제 |
| `_ensure_dsr()` lazy import | `gui.py:207-253` | `jog` 심볼 추가만 |
| AUTONOMOUS 전환 검증 폴링 | `gui.py:357` | jog 시작 전에도 동일 적용 |
| QObject worker + QThread + moveToThread | `gui.py:430+` | DualSenseWorker 도 동일 패턴 |

---

## 검증 (end-to-end)

### 1. 빌드 + 실행
```bash
cd ~/cobot_ws
colcon build --packages-select ros2_move_recoder
source install/setup.bash
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=virtual host:=127.0.0.1 port:=12345 model:=m0609 &
ros2 run ros2_move_recoder gui
```

### 2. 수동 테스트 시나리오
- [ ] DualSense USB 연결 → GUI 상태바 "✓ DualSense 연결됨" + lightbar 청색 (J123 모드)
- [ ] LX 좌/우 → J1 회전 (RViz 에서 확인)
- [ ] △ 누름 → 모드 J456, lightbar 시안, 햅틱 펄스
- [ ] ○ 누름 → 모드 XYZ, lightbar 녹색, LX/LY/RY → X/Y/Z 직교 이동
- [ ] R2 트리거 100% 압박 → 속도 2배 (default 50% → 100%)
- [ ] 스틱 놓으면 250ms 내 정지 (watchdog)
- [ ] Touchpad 누름 → 즉시 정지 + lightbar 적색 깜빡임
- [ ] USB 뽑기 → 즉시 stop + 상태바 경고
- [ ] jog 도중 ▶ Play 클릭 → "busy" 거부 메시지

### 3. 통합 흐름 확인
- [ ] DualSense 로 자세 잡기 → ● Record → 기록 → ■ Stop → ∿ Smooth → ▶ Play 가 끝까지 동작
- [ ] DSR 에러 코드 시 좌측 햅틱 진동 (관절 한계 시뮬레이션 — 의도적으로 limit 근처로 jog)

### 4. 회귀 검증 (기존 기능 영향)
- [ ] 기존 record / smooth / play 모두 정상 동작 (DualSense 미연결 상태에서도)
- [ ] 기존 4개 entry point (`run`, `recorder`, `smoother`, `player`) 영향 없음
- [ ] `myros2-macro_gui` alias 그대로 동작

---

## 주의 / 위험

- **pydualsense 의 hidapi 의존성** — apt `libhidapi-hidraw0` 필요. `udev` rule 없으면 root 권한 필요할 수 있음 → 설치 가이드에 명시
- **manage 와 동시 실행 금지** — 둘 다 같은 컨트롤러를 잡으려 하면 충돌. 한 시점에 하나만
- **AUTONOMOUS 모드 미전환 시 jog 거부** — 기존 `set_robot_mode` 패턴 재사용 (gui.py:357)
- **6축 동시 jog 불허** — 4-모드 분리로 한 번에 3축만 가능 → `_busy` + latest-wins 가 자동으로 보장
