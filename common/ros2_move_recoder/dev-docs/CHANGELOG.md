# Changelog — 주요 변경 이력

> 시간순. 자세한 매핑/동작은 각 dev-doc 참조.

## 2026-05-03

### TCP jog 모드 (Options 버튼 토글)
- **Options 버튼** (터치패드 우측, `BTN_OPTIONS=9`) → `'joint'` ↔ `'tcp'` 모드 토글
- TCP 모드 매핑:
  - 좌측 스틱 LX 좌/우 → **−X / +X** jog (mm/s, ref=DR_BASE)
  - 좌측 스틱 LY 위/아래 → **+Y / −Y** jog
  - 우측 스틱 RY 위/아래 → **+Z / −Z** jog
  - 우선순위: 3 candidates 중 절댓값 큰 쪽 (latest-wins, dispatcher 와 호환)
- 워커 신규 시그널: `jog_mode_changed(str)` — 메인 thread queued
- 모드 전환 시: 잔여 jog 즉시 정지 + cooldown reset
- vel 단위: joint °/s (max 80) / TCP mm/s (max 50, `TCP_MAX_VEL_MM_S` 상수)
- DSR `jog(axis 6/7/8, ref=DR_BASE, vel)` API 그대로 사용 — dispatcher 변경 없음
- Cheat sheet 갱신: 두 모드 매핑 명시

### TCP 좌표 패널 표시 동기화
- mode 토글 시 통합 패널의 좌표 표시도 자동 전환:
  - `'joint'`: J1~J6 (deg) — `_on_joint` (60Hz `/joint_states`) 가 갱신
  - `'tcp'`: X/Y/Z (mm), A/B/C (deg) — 1Hz 폴링 (`_tcp_posx_timer`)
- 신규: `DsrWorker.query_posx` 슬롯 + `posx_received(list)` 시그널 (worker thread 안전)
- `MainWindow.request_posx` 시그널 + `_on_posx_received` 슬롯 (cross-thread queued)
- `axis_tags` 리스트로 tag 라벨 (J* ↔ X/Y/Z/A/B/C) 동적 변경 (단위 표시 포함)
- `_on_joint` 가드: TCP 모드면 joint_labels 덮어씌우지 않음
- closeEvent 에서 timer 정지

### E-STOP 통합 (모든 동작 취소 + HOME 자동 복귀)
- `_on_estop()` 재설계: ① record stop → ② DSR stop(DR_SSTOP) → ③ 그리퍼 timeline abort → ④ 800ms 후 HOME 자동
- DualSense × long-press 도 통합 핸들러 호출 → 동일 동작
- cheat sheet 갱신

### Joint 선택: D-Pad → 좌측 스틱
- D-Pad joint 선택 매핑 제거 (예약 상태로 전환)
- **좌측 스틱 좌/우** 로 joint −1 / +1 (히스테리시스 0.6/0.3)
- 우측 스틱 hold 한 채로 좌측 스틱 → 손 안 떼고 연속 joint 전환
- 비선택 joint 가시성: 테마 토큰 (`text`) 사용 → 다크/라이트 모두 가독성 보장
- cheat sheet + dev-docs (`dualsense-mapping.md`) 갱신

### 그리퍼 Play 정확도 개선
- `gripper_events` 에 **`kind` 필드** 추가 (`'close'` / `'open'` / `'move'`) — 변화 방향으로 사용자 의도 추론. raw 측정 width 정확도와 무관
- player timeline 이 kind 따라 **□ 와 동일 함수 호출** (`gripper.close()` / `gripper.open()` / `gripper.move()`)
- final state 통합 시 `t_norm = max(prev, 1.0)` 버그 fix — 직전 event 의 명령 발사 시점 보존
- `t_norm` arc-length 진행률 + 1.0 초과 허용 (raw 마지막 정지 구간 events 도 grace 안에 발사)
- min_gap 강제 (raw 시간 2.0s 환산) + player `GRIP_MIN_GAP_S=2.0` 안전망
- est_duration 우선순위: measured (학습) > record_duration > arc-length fallback
- final state flush + measured_duration 학습 (다음 play 부터 정확)
- dev-docs (`gripper.md`) 전체 갱신

### 코드베이스 정리
- **`smoother.py` ↔ `gui.py:smooth_and_save` 통합** — 두 코드가 분기되어 있던 것을 단일 source of truth (`smoother.smooth_and_save`) 로 정리. CLI (`ros2 run smoother`) 와 GUI 가 동일 함수 사용 → `gripper_events` / `record_duration_sec` 가 CLI 에서도 정상 추출
- gui.py 의 인라인 정의 207줄 제거 → `from .smoother import smooth_and_save`
- DualSense 진단 로그 (`[ds] btn N ↓`, `[ds][diag] modal=...`) 를 **verbose 모드 (Ctrl+Shift+D)** 일 때만 출력하도록 변경 — default 로그 깔끔
- 검토 후 그대로 유지: `macros/` 빈 디렉토리 (예약), `__init__.py`, 모든 imports (실 사용)

### Mini-Jog (개별 조인트 ± GUI 제어)
- `gui.py` 통합 패널: 현재 좌표 6행 + 각 행에 `−` / `+` 버튼 + 속도 입력 (default **80°/s**, 1~90 범위)
- AUTONOMOUS 모드 + `_busy=False` 일 때만 활성, 모드/play 변화 시 자동 토글
- `DsrWorker.jog(axis, ref, vel)` + `stop_jog()` 슬롯
- `_JogDispatcher` (manage 의 latest-wins 패턴 포팅) — 별도 thread, 동일 vel 재발사 방지, 1초 SAFETY_REFIRE
- 행 배치 순서: **J6 (위) → J1 (아래)** — 펜던트 mental model 과 일치

### DualSense PS5 컨트롤러 통합 (`dualsense_worker.py`)
- pygame.joystick 기반, `/dev/input/js0` (권한 추가 불필요)
- 메뉴 `[컨트롤러] → DualSense 활성화` (Ctrl+D) — 활성 시 OnRobot 그리퍼도 자동 connect
- 상태바 `🎮 DualSense ✓` pill (회색/주황/청색)
- 매핑 ([dualsense-mapping.md](dualsense-mapping.md) 참조):
  - **Create** short=Record toggle / long(≥2s)=새 프로파일 자동 생성
  - **○** Smooth+Play (smooth.json 있으면 즉시 Play, 없으면 Smooth 후 자동) / 다이얼로그 시 Yes/Ok
  - **×** short=Pause/Resume 토글 / long(≥1s)=E-Stop / 다이얼로그 시 No/Cancel
  - **△** Home 복귀
  - **□** OnRobot 그리퍼 Open ↔ Close 토글
  - **D-Pad ↑/→** 선택 joint +1 (wrap), **↓/←** −1
  - **우측 스틱** 현재 선택 joint 의 jog (응답곡선 `|s|^1.5 × max_vel`)
  - **L2/R2** hold 시 100ms 주기로 속도 ±1°/s
  - **L3+R3** MANUAL ↔ AUTONOMOUS 토글
- jog 안정화: 양자화 10°/s + 변화 임계 10°/s + cooldown 200ms (DSR ramp-up 보호)
- 디버그 모드 (Ctrl+Shift+D) — verbose 입력 로그, 폴링 헬스 통계
- 매핑 cheat sheet 다이얼로그 (`[컨트롤러] → DualSense 매핑 보기…`)

### OnRobot RG2/RG6 그리퍼 통합 (`gripper_worker.py`, `onrobot.py`)
- Modbus TCP wrapper (Calibration_Tutorial 의 `onrobot.py` 채택)
- 환경변수: `GRIPPER_IP` (default `192.168.1.1`), `GRIPPER_PORT` (`502`), `GRIPPER_TYPE` (`rg2`)
- DualSense 활성화 시 자동 connect (별도 메뉴 옵션 없음)
- 4Hz width polling → 상태바 `🦾 Gripper ✓ XX.Xmm` + 패널 표시
- Recorder: 매 joint sample 마다 width 함께 buffer → `raw.json: gripper_widths_mm`
- Smoother: width 변화 ≥5mm 시점만 events 추출 → `smooth.json: gripper_events`
- Player: amovesj 진행 시간 추정 (`arc_length / (vel × ops_ratio) × 1.10`) 으로 timeline 발사
- 호환성: 그리퍼 비활성/미연결 시 기존 raw/smooth 흐름 그대로

### 다이얼로그 ○/× 매핑
- `QApplication.activeModalWidget()` 검사 → `QMessageBox.Yes/No/Ok/Cancel` 클릭
- 3단계 fallback: `QMessageBox.button()` → `QDialogButtonBox.button()` → `dialog.accept()/reject()`
- `QInputDialog` (액션 이름 입력) 도 `accept()/reject()` 로 처리
- modal 떠있을 때 × long-press(E-Stop) 는 **무시** (안전)

### Cross-thread 시그널 dispatch
- DualSense 워커는 pure Python `threading.Thread` (Qt thread 아님)
- nested `exec_()` 에서 cross-thread queued signal 누락 edge case 발견
- 시도 1: `QTimer.singleShot(0, signal.emit)` wrap → **caller thread (daemon) 에 event loop 가 없어 timer fire 안 됨 → 모든 button 무반응** → **rollback**
- 현재: `signal.emit()` 직접 호출 (PyQt5 cross-thread queued 자동 dispatch)
- nested loop 누락 케이스는 `_try_dialog_button` 의 `QMetaObject.invokeMethod(..., Qt.QueuedConnection)` 로 modal 응답 보장

### Modal 활성 시 jog 일시정지
- GUI 100ms timer 가 `activeModalWidget()` 모니터링 → `worker.set_modal_active(True/False)` 통보
- modal 활성 시 워커가 jog/D-Pad/L2R2/L3R3 입력 차단 (button event 만 처리)
- 진입 시 잔여 jog 즉시 stop

### GUI 레이아웃
- 좌측 패널 `QScrollArea` 적용 — 화면이 작아도 모든 그룹박스 접근 가능
- 윈도우 최소 크기 900x600

## 2026-04-28

- DualSense 도입 설계안 작성 ([dualsense-plan.md](dualsense-plan.md))
- 음성/비전 비동기 워크플로 설계 ([voice-vision-async-workflow.md](voice-vision-async-workflow.md))

## 2026-04-27

- 초기 dev-docs 작성 (architecture / build-and-run / data-formats / extending / gui / pipeline / troubleshooting)
