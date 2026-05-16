# OnRobot RG2/RG6 그리퍼 통합

> 그리퍼 제어 + record/smooth/play 파이프라인 통합. 코드: `ros2_move_recoder/onrobot.py`, `gripper_worker.py`.

## 개요

- 하드웨어: OnRobot **RG2** (max 1100/10mm = 110mm, force 400/10N) / RG6 (max 1600/10mm, force 1200/10N)
- 통신: **Modbus TCP** (default `192.168.1.1:502`)
- 라이브러리: `pymodbus` 2.5.3 (apt `python3-pymodbus`)
- 코드 출처: `~/Downloads/corecode/Calibration_Tutorial/onrobot.py` 패키지에 복사

## 환경변수

| 변수 | default | 비고 |
|---|---|---|
| `GRIPPER_IP`   | `192.168.1.1` | Modbus TCP 호스트 |
| `GRIPPER_PORT` | `502` | Modbus TCP 포트 |
| `GRIPPER_TYPE` | `rg2` | `rg2` / `rg6` |

```bash
GRIPPER_IP=10.0.0.5 ros2 run ros2_move_recoder gui
```

## 활성화

DualSense 활성화 (Ctrl+D) 시 **자동 connect** — 별도 메뉴 옵션 없음.

상태바 `🦾 Gripper`:
- 회색 OFF
- 빨강 연결 실패 (5초 마다 재시도)
- 청색 ✓ 연결 + 현재 width [mm]

## API (`gripper_worker.py`)

### 외부 슬롯
- `start()` / `stop()` — polling thread 제어
- `open()` / `close()` / `move(width_mm: float)` — 사용자/패널/패드 입력
- `is_connected() -> bool`
- `last_width() -> float | None` — mm
- `last_state() -> str` — `'open' / 'closed' / 'mid' / 'unknown'` (□ 토글 결정용)

### 시그널
- `connected_changed(bool, str)` — 연결 상태 + 메시지
- `width_changed(float)` — 4Hz 폴링 결과 [mm]
- `log(str)`

### 내부
- `_lock: threading.Lock` — Modbus 호출 직렬화
- `_command(fn, new_state, log)` — connect 보장 + 예외 처리 + 상태 업데이트
- `_safe_disconnect()` — 통신 끊김 시 graceful 정리
- 4Hz polling loop — width 읽기 + 변화 ≥0.5mm 시만 emit (UI 부하 제어)

## GUI 통합

### 패널 (좌측 컬럼)
```
OnRobot 그리퍼 · DualSense 활성화 (Ctrl+D) 시 자동 연결
─────────────────────────────────────────────────────
현재   <width mm>   [Open] [Close]
목표   [50.00 mm]   [Move →     ]
준비 — Open/Close 버튼 또는 □ 버튼 (DualSense)
```

### DualSense □ 토글
- `_on_dualsense_gripper_toggle`:
  - `last_state == 'closed'` → `gripper.open()`
  - 그 외 (`'open' / 'mid' / 'unknown'`) → `gripper.close()`
- 비활성/미연결 시 무시 + 로그

## Recorder 통합

- 매 `_on_joint` 콜백마다 `self._gripper_width_mm` 함께 buffer:
  ```python
  self.buffer_w.append(
      self._gripper_width_mm if self._gripper_active else None)
  ```
- `_save_raw` 가 1개라도 None 아닌 width 가 있으면 `gripper_widths_mm` 키 추가
- 그리퍼 비활성/미연결 → 키 자체 미저장 (기존 raw.json 호환)

## Smoother 통합 (`smoother.smooth_and_save` — single source of truth)

GUI 와 CLI (`ros2 run smoother`) 모두 동일 함수 사용. `raw.json` 의 `gripper_widths_mm` → `smooth.json` 의 `gripper_events` 변환:

```python
g_widths = raw.get("gripper_widths_mm")
if g_widths and len(g_widths) == n_raw:
    # arc-length 진행률 기반 t_norm — 정지 구간 압축에 강건
    raw_cum = np.concatenate([[0.0], np.cumsum(joint_diffs)])
    raw_total = float(raw_cum[-1])
    last_w = None
    for i, w in enumerate(g_widths):
        if last_w is None:
            kind = 'close' if w <= 20 else ('open' if w >= 50 else 'move')
        elif abs(w - last_w) >= 5.0:
            kind = 'open' if w > last_w else 'close'   # 변화 방향 = 사용자 의도
        else:
            continue
        gripper_events.append({
            "t_norm": raw_cum[i] / raw_total,
            "kind": kind,
            "width_mm": round(float(w), 2),
        })
        last_w = w
    # final state 통합 + 인접 events 최소 gap 강제 (raw 시간 2.0초 환산)
```

### 핵심 정책

1. **`kind` 분류 = 사용자 의도** (raw 측정 width 의 정확도 무관)
   - 첫 event: width 절대값 임계 (≤20mm=close, ≥50mm=open, 그 외=move)
   - 그 외: width 변화 방향 (증가=open, 감소=close)
2. **`t_norm` = arc-length 누적 진행률** (사람 시연 시간 비례 X)
   - 사람이 그리퍼만 조작하느라 로봇 정지하면 정지 구간이 smoother 에서 압축돼 amovesj 시간이 훨씬 짧아짐
   - arc-length 진행률은 amovesj 의 균일 vel 진행과 정확히 일치
   - **1.0 초과 허용** — raw 마지막에 모션 없는 정지 구간의 events 도 player grace 안에 발사
3. **min_gap (raw 시간 2.0초 환산)**: events 사이 최소 t_norm gap — OnRobot RG2 모터 동작 시간 (~2s) 보장
4. **final state 통합**: 마지막 raw width 가 직전 event 와 차이 < 5mm → width 만 update (t_norm 보존)

## Player 통합

`MainWindow._start_gripper_play_timeline()` — `_on_play_started` 시점에 호출:

1. `smooth.json` 의 `gripper_events` 로드 (없으면 return)
2. **est_duration 결정 (우선순위)**:
   - `measured_duration_sec × (measured_play_vel / current_play_vel)` — 이전 play 측정값 (가장 정확)
   - `record_duration_sec × (smooth_vel / current_play_vel)` — 첫 play
   - `arc-length / vel × 1.10` — 옛 action fallback
3. 별도 daemon thread 가 wall clock 으로 진행:
   - target_t = `ev['t_norm'] × est_duration`
   - **`GRIP_MIN_GAP_S = 2.0` 안전망**: emit 간 최소 시간 보장 (smoother 가 못 분리한 케이스 안전망)
   - **`kind` 따라 □ 와 동일 함수 호출**:
     - `'close'` → `gripper.close()` (펌웨어 20mm grip)
     - `'open'` → `gripper.open()` (펌웨어 max_width 까지)
     - `'move'` → `gripper.move(width_mm)`
4. `_on_play_finished` 시점:
   - **measured_duration 학습** → smooth.json 에 저장 (다음 play 부터 정확)
   - **final state 보장** — events 의 마지막 width 강제 발사 (안전망)
   - `_stop_gripper_play_timeline(abort)`:
     - 정상 종료: grace **8s** (모터 시간 × N events 보장)
     - abort: 즉시 stop

## 데이터 포맷

`raw.json`:
```json
{
  ...
  "gripper_widths_mm": [110.0, 110.0, 109.5, ..., 5.0, 5.0]
}
```

`smooth.json`:
```json
{
  ...
  "record_duration_sec": 10.42,
  "measured_duration_sec": 5.46,         // 이전 play 측정 (학습됨)
  "measured_at_play_vel": 60.0,
  "gripper_events": [
    {"t_norm": 0.000, "kind": "close", "width_mm": 1.91},
    {"t_norm": 0.326, "kind": "open",  "width_mm": 10.75},
    {"t_norm": 0.503, "kind": "close", "width_mm": 3.25},
    {"t_norm": 1.176, "kind": "close", "width_mm": 1.76}  // t_norm > 1.0 가능
  ]
}
```

## 호환성

- 그리퍼 OFF / 미연결 → 기존 record/smooth/play 흐름 그대로 동작
- 옛 raw.json (gripper 필드 없음) → smoother 가 events 비워둠 → player 도 timeline thread 시작 안 함
- 옛 smooth.json → 그리퍼 timeline thread 시작 안 함

## 디버깅

연결 실패 시 GUI 로그:
```
[grip] 폴링 시작 (target 192.168.1.1:502, 4Hz)
[grip] 초기 연결 실패 — 5s 마다 재시도
```

다른 IP 시도:
```bash
GRIPPER_IP=192.168.1.99 ros2 run ros2_move_recoder gui
```

수동 ping 테스트:
```bash
ping -c 2 192.168.1.1
nc -zv 192.168.1.1 502
```

## 향후 개선

- Force 인자 GUI 노출 (현재 default 400)
- get_status 비트 모니터링 (busy / grip-detected → UI 표시)
- 캘리브레이션: fingertip offset 자동 적용
- Smoother 의 변화 임계 (5mm) GUI 노출
