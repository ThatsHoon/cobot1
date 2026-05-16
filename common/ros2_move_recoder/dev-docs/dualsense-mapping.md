# DualSense × ros2_move_recoder — 실제 매핑 + 디버깅

> 설계안은 [dualsense-plan.md](dualsense-plan.md) 참조. 본 문서는 **실제 구현된** 매핑/동작/문제 해결책을 정리한다.

## 활성화

- 메뉴: `[컨트롤러] → DualSense 활성화` (Ctrl+D)
- 활성화 시 **OnRobot 그리퍼도 자동 connect** (메뉴 별도 옵션 없음)
- 비활성화 시 그리퍼도 같이 disconnect
- 상태바 `🎮 DualSense ✓ <name>` (회색 OFF / 주황 검색중 / 청색 연결됨)
- USB 또는 Bluetooth 모두 동작 (커널 `hid-playstation` 드라이버 자동 인식)

## 매핑 표

| 입력 | 동작 | 비고 |
|---|---|---|
| **Create** (short) | Record on/off 토글 | UP 시점에 long fired 안 됐으면 |
| **Create** (≥2s hold) | 새 프로파일 자동 시작 (`action_YYYYMMDD_HHMMSS`) | 진행 중인 record 자동 stop 후 |
| **○** | Smooth+Play (smooth.json 있으면 즉시 Play) | 다이얼로그 활성 시 → Yes/Ok |
| **× (short)** | Pause ↔ Resume 토글 | 다이얼로그 활성 시 → No/Cancel |
| **× (≥1s hold)** | 🛑 비상 정지 (E-Stop) | modal 떠있으면 무시 (안전) |
| **△** | Home 위치 복귀 | `_on_home()` — 재생 중이면 abort 후 자동 home |
| **□** | OnRobot 그리퍼 Open ↔ Close 토글 | `last_state` 기준 반전. close = 20mm |
| **Options** (터치패드 우측) | **🦾 JOINT ↔ 🌐 TCP·BASE jog 모드 토글** | 패널 좌표 표시도 J* ↔ X/Y/Z/A/B/C 자동 전환 |
| **L2** (hold) | 선택 joint 속도 −1°/s 지속 (10Hz) | hold 시 100ms 주기 자동 반복 |
| **R2** (hold) | 선택 joint 속도 +1°/s 지속 (10Hz) | 동일 |
| **L3 + R3** | MANUAL ↔ AUTONOMOUS 토글 | 둘 다 release 해야 다음 발사 |
| **D-Pad** | (예약, 미사용) | 이전: joint 선택 → 좌측 스틱으로 이동 |
| Touchpad / PS / Mute / L1 / R1 | (예약) | future |

### 좌·우 스틱 (mode 의존)

| 모드 | 좌측 스틱 | 우측 스틱 |
|---|---|---|
| **🦾 JOINT** | ↑/→ = 다음 joint, ↓/← = 이전 joint (J1→…→J6 wrap, 히스테리시스 0.6/0.3) | RX/RY 중 큰 쪽 → 선택 joint jog (응답곡선 `sign(s)·\|s\|^1.5 · 80°/s`) |
| **🌐 TCP·BASE** | LX 좌/우 → −X/+X (mm/s), LY 위/아래 → +Y/−Y | RY 위/아래 → +Z/−Z (mm/s, max 50) |

### TCP 좌표 패널 표시

mode 가 `'tcp'` 로 전환되면 통합 패널의 6 축 표시 자동 변환:
- 라벨: `J1~J6` → `X` `Y` `Z` `A` `B` `C` (단위 mm/° 함께 표시)
- 좌표 값: `_tcp_posx_timer` 가 1Hz 로 `request_posx` emit → `DsrWorker.query_posx()` 가 `get_current_posx()` 호출 → `posx_received(list)` 시그널 → 패널 갱신
- joint 콜백 (`_on_joint`) 은 TCP 모드면 좌표 갱신 skip (덮어씌움 방지)
- `'joint'` 로 복귀 시 라벨/타이머/콜백 자동 복원

### Joint 선택 즉시 전환 (조작감)

좌측 스틱 좌/우로 joint 변경 시:
1. 이전 joint 의 잔여 jog 즉시 정지 (`dispatcher.stop()`)
2. `_last_jog_target = None` + `_last_jog_emit_t = 0` (cooldown reset)
3. 다음 polling iteration (~16ms) 에 우측 스틱 raw 다시 읽어 → 새 joint 로 즉시 emit

**우측 스틱 hold 한 채로 좌측 스틱으로 joint 바꾸면** — 손 안 떼고 연속 jog 전환 가능.

## 라이브러리 / 커널 / 권한

- Python: **pygame 2.1.2** (`python3-pygame` apt)
- SDL: 2.0.20 (joystick subsystem 만 사용, `SDL_VIDEODRIVER=dummy` 헤드리스 안전)
- 커널 드라이버: **`hid-playstation`** (Linux 5.12+, USB/BT 통합)
- 입력 디바이스 (예시):
  - `/dev/input/js0` — 메인 컨트롤러 (사용)
  - `/dev/input/event15` — 동일 (evdev 경로, pygame 안 사용)
  - `/sys/class/leds/input21:rgb:indicator/` — RGB lightbar (sysfs, 권한 필요)
- 권한: `/dev/input/js0` 는 `crw-rw-r--` (누구나 read) → **추가 udev 불필요**

## Joydev axis/button 인덱스 (SDL 2.0.20 + hid-playstation)

```python
AXIS_LX = 0;  AXIS_LY = 1
AXIS_L2 = 2  # 안눌림 -1, 풀눌림 +1
AXIS_RX = 3
AXIS_RY = 4
AXIS_R2 = 5

BTN_CROSS = 0; BTN_CIRCLE = 1; BTN_TRIANGLE = 2; BTN_SQUARE = 3
BTN_CREATE = 8     # Share / Create
BTN_L3 = 11; BTN_R3 = 12
```

> 환경에 따라 다를 수 있다. 디버그 모드 (Ctrl+Shift+D) 켜고 버튼 누르면 GUI 로그에 `[ds] btn N ↓` 가 표시되어 실제 인덱스 확인 가능. 안 맞으면 `dualsense_worker.py` 의 `BTN_*` 상수 조정.

## Jog 안정화 정책

스틱 raw 값 jitter + 60Hz 폴링이 jog 명령 폭주 → DSR 컨트롤러의 ramp-up 곡선이 매번 재시작 → 평균 vel 이 명령보다 훨씬 낮아지는 문제가 있었다. 현재 정책:

| 항목 | 값 | 효과 |
|---|---|---|
| `VEL_QUANT_DEG_S` | 10°/s | vel 양자화 (미세 변화 collapse) |
| `VEL_CHANGE_TH_DEG_S` | 10°/s | 동일 axis/부호 시 이 미만 변화 무시 |
| `JOG_EMIT_MIN_INTERVAL_S` | 0.20s | 같은 axis 재emit 최소 간격 (DSR ramp 보호) |
| 즉시 emit | first / axis 변경 / 부호 변경 / 정지 | 반응성 보장 |

## 다이얼로그 ○/× 매핑

- `QApplication.activeModalWidget()` 검사
- 3단계 fallback (`MainWindow._try_dialog_button`):
  1. `QMessageBox.button(std_button)` — Yes/No 다이얼로그
  2. 내부 `QDialogButtonBox.button(std_button)` — 일반 QDialog 일부 환경
  3. `dialog.accept()` / `dialog.reject()` — `QInputDialog` 처럼 standard 매핑이 None 인 경우 (Enter/Esc 와 동일)
- 호출은 `QMetaObject.invokeMethod(..., Qt.QueuedConnection)` 으로 메인 thread 안전
- ○: `Yes` → `Ok` 폴백 / ×: `No` → `Cancel` 폴백
- modal 활성 시 × long-press (E-Stop) 는 무시

## Modal 떠있을 때 jog 일시정지

- GUI 100ms timer 가 `activeModalWidget()` 모니터링
- 변화 시 `worker.set_modal_active(True/False)` 통보
- 워커는 modal 활성 시 polling 루프에서 jog/D-Pad/L2R2/L3R3 처리 skip
- button event 는 그대로 dispatch (modal 응답 가능)
- modal 진입 시 잔여 jog 즉시 stop

## Cross-thread 시그널 dispatch (PyQt5 edge case)

DualSense 워커는 pure Python `threading.Thread` 라서, nested `exec_()` (예: `QInputDialog.getText()`) 안에서 cross-thread queued signal 이 dispatch 누락되는 케이스 발견.

**우회**: 모든 워커 emit 을 `QTimer.singleShot(0, signal.emit)` 으로 wrap (`_post_main` 헬퍼). caller thread 와 무관하게 메인 thread queue 에 timer event 로 등록 — nested loop 도 timer event 는 안정적으로 spin.

```python
def _post_main(self, callable_):
    QtCore.QTimer.singleShot(0, callable_)

# button down 처리:
if ev.button == BTN_CIRCLE:
    self._post_main(self.smooth_play_combo.emit)
```

## 디버깅 (Ctrl+Shift+D)

verbose 모드 ON 시 추가 출력:
- `[ds][debug] btn N ↑` — button up
- `[ds][debug] hat (x,y) → (x,y)` — D-Pad 변화
- `[ds][debug] RX/RY/L2/R2 ±0.00 → ±0.00` — 스틱/트리거 raw (≥0.05 변화)
- `[ds][debug] L3=1 R3=0 (armed=True)` — L3/R3 + 콤보 arm 상태
- `[ds][debug] jog J3 vel=+25°/s (raw use=+0.45)` — jog emit 시점
- `[ds][debug] 헬스 — poll 60.0 Hz (누적 300/5.0s), connected=True, ...` — 5초 주기
- `[ds][debug] 폴링 지연: 25.3ms (목표 16.7ms)` — cycle > 1.5x period 시
- `[ds][diag] modal=QInputDialog → accept() (queued)` — modal 응답 fallback 추적

## 매핑 cheat sheet (앱 내)

`[컨트롤러] → DualSense 매핑 보기…` 메뉴로 다이얼로그 표시.

## 향후 확장 (예약)

- L1 / R1 / Touchpad / PS / Mute 버튼
- 좌측 스틱 — TCP X/Y jog (모드 전환 옵션)
- Lightbar 색상 (모드별) — sysfs udev rule 필요
- 햅틱 진동 (`pygame.joystick.Joystick.rumble`) — 이벤트 피드백
