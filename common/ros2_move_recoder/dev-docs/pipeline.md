# Pipeline — recorder → smoother → player

3단계 파이프라인. CLI에서는 명령 셋, GUI에서는 버튼 셋. 알고리즘은 동일.

```
[/dsr01/joint_states]
        │ subscribe @ ~60-100Hz
        ▼
   recorder.py        ─►  records/<name>/raw.json
        │
        ▼
   smoother.py        ─►  records/<name>/smooth.json
        │
        ▼
   player.py          ─►  movesj(waypoints, vel, acc)
                            via /dsr01/motion/move_spline_joint
```

---

## 1단계 — recorder

### 입력
- ROS 토픽 `/dsr01/joint_states` (`sensor_msgs/JointState`)
- 메시지의 `name` 필드는 `joint_1`..`joint_6` (라디안 단위)

### 처리
```python
name_to_pos = dict(zip(msg.name, msg.position))
joint_deg = [round(math.degrees(name_to_pos[f"joint_{i}"]), 4)
             for i in range(1, 7)]
self._buffer_t.append(time.monotonic())
self._buffer_q.append(joint_deg)
```

- 라디안 → degree 변환 (`math.degrees`)
- `time.monotonic()` 단조 시계로 타임스탬프 기록 (system clock 점프 영향 없음)
- 소수점 4자리로 반올림 (이후 평활화에서 충분)

### 출력 (`raw.json`)
```json
{
  "timestamps_ms": [0, 16, 33, ...],
  "joints_deg":    [[J1,J2,J3,J4,J5,J6], ...],
  "rate_hz_avg":   62.5,
  "duration_sec":  12.345,
  "samples":       772,
  "recorded_at":   "2026-04-27T..."
}
```

### 시작/정지 UX
- **CLI**: Enter 두 번 (start → stop)
- **GUI**: `● Record` → 액션명 입력 → `■ Stop`

### 함정
- ROBOT_MODE가 AUTONOMOUS면 펜던트 직접 조작 불가 → MANUAL 모드여야 시연 가능
- `/joint_states`는 RELIABLE QoS — depth=10이라 일시적 부하 시 drop 가능 (loss는 timestamps_ms gap으로 확인)

---

## 2단계 — smoother

### 알고리즘 (4 패스)

```
raw.json (N개 샘플)
    │
    ▼
[1] Savitzky-Golay 평활화           ◄── 펜던트 stepwise 노이즈 제거
    │   window = max(arg, min(N//10|1, 201))   (자동 확장, 항상 홀수)
    │   polyorder = 3 (default)
    │   smoothed[0], smoothed[-1] = 원본 (양 끝 보존)
    ▼
[2] 정지 구간 압축                   ◄── 펜던트 잠시 멈춘 구간 제거
    │   for i in 1..N-2:
    │       if ‖smoothed[i] - last_kept‖ ≥ eps:
    │           keep i, last_kept = smoothed[i]
    │   기본 eps = 0.5° (6축 L2 norm)
    ▼
[3] 방향 전환점 검출                 ◄── peak/trough 강제 보존
    │   각 관절별:
    │     if (max - min) < prom: skip   (거의 변화 없음)
    │     peaks   = find_peaks( sig, prominence=prom)
    │     troughs = find_peaks(-sig, prominence=prom)
    │   기본 prom = 2.0°
    ▼
[4] arc-length 등간격 다운샘플 + 전환점 union
    │   cum = [0, ‖Δq₁‖, ‖Δq₁‖+‖Δq₂‖, ...]   (누적 관절공간 거리)
    │   targets = linspace(0, cum[-1], n_target)
    │   equi_idx = searchsorted(cum, targets)
    │   final_idx = sorted(equi_idx ∪ turning_idx ∪ {0, end})
    │   n_target = max(2, max_pts - n_turning)
    ▼
smooth.json (K개 waypoint, K ≤ max_pts ≤ 100)
```

### 왜 이 4 패스인가

| 패스 | 목적 | 미적용 시 |
|---|---|---|
| 1. Savgol | 펜던트가 디지털 그리드에 snap 되며 생기는 stair-step 노이즈 제거 | 재생 시 미세 진동 |
| 2. 정지 압축 | 펜던트 잠시 놓고 생각하는 구간 (수백 샘플 동일 위치) → 1점으로 | waypoint 낭비 → arc-length가 짧은 이동 구간 디테일을 못 잡음 |
| 3. 전환점 보존 | 등간격 다운샘플은 peak/valley 사이를 직선으로 잘라낼 수 있음 | 둥글게 휘는 모션이 직선으로 압축 |
| 4. arc-length 등간격 | 시간 등간격은 빠른 구간의 디테일을 잃음. 거리 등간격이 균일 곡률 보존에 유리 | 경로 형태 왜곡 |

### 출력 (`smooth.json`)
```json
{
  "action_name": "my_first",
  "waypoints_deg": [[...6...], ...K개],
  "vel": 30.0,
  "acc": 60.0,
  "n_waypoints": 80,
  "n_raw": 772,
  "n_after_stationary_filter": 410,
  "n_turning_points": 7,
  "stationary_eps_deg": 0.5,
  "turning_prominence_deg": 2.0,
  "max_adjacent_jump_deg": 8.42,
  "smoothing": {
    "window": 51, "polyorder": 3, "max_pts": 80,
    "downsample": "stationary-removed + arc-length + turning-points"
  }
}
```

### 파라미터 가이드

| 파라미터 | 기본 | 작게 | 크게 |
|---|---|---|---|
| `window` (Savgol) | 51 | 노이즈 그대로 | 모서리가 둥글어짐 |
| `polyorder` | 3 | 단순 직선 | overfitting → 노이즈 추종 |
| `max_pts` | 80 | 빠르지만 직선 보간 → 경로 왜곡 | 100 초과 불가 (movesj 한계) |
| `eps` (정지) | 0.5° | 미세 떨림도 keep → 샘플 낭비 | 천천히 움직인 구간이 정지로 오인 |
| `prom` (전환점) | 2.0° | 미세 진동까지 보존 → waypoint 폭증 | 큰 곡선만 보존 → 디테일 손실 |
| `vel`, `acc` | 30, 60 | 안전, 느림 | 빠르지만 충돌 시 위험 |

---

## 3단계 — player

### 입력
- `smooth.json` 의 `waypoints_deg`, `vel`, `acc`

### 처리
```python
mode = get_robot_mode()
if mode != ROBOT_MODE_AUTONOMOUS:
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    # 폴링으로 전환 검증 (최대 2초)
pts = [posj(*w) for w in waypoints]
rc = movesj(pts, vel=vel, acc=acc)
```

### 왜 `movesj`인가

| 함수 | 시간 정보 | 균일 속도 | 비고 |
|---|---|---|---|
| `movej` 반복 | 무시 | X (각 segment마다 가감속) | waypoint 마다 정지/재출발 |
| `movesj` | **무시** | **O (전 구간 균일)** | spline 보간, vel/acc만 받음 |
| `servoj` 스트림 | 사용 | 시간 정확 | 1ms 단위 외부 제어, 지터 위험 |

`movesj`는 시간 정보를 무시하고 자체 보간 → 등간격 waypoint가 필요 → 그래서 smoother가 arc-length 다운샘플링.

### 안전 절차
1. **사용자 확인**: CLI는 `(y/N)`, GUI는 modal `Yes/No`
2. **모드 검증**: AUTONOMOUS 미전환 시 abort
3. **재생 중 E-STOP**: GUI는 Esc 키 → `stop(DR_SSTOP)`

### 함정
- `movesj` 반환 0 = 성공, 그 외 = DSR 에러 코드
- waypoint 100개 초과 시 reject
- 인접 waypoint 점프가 너무 크면 (>30° 정도) 컨트롤러가 거부 가능 → smoother가 `max_adjacent_jump_deg` 로깅

### 속도 제어 3-layer

| 레이어 | 대상 | GUI 컨트롤 | 효과 범위 |
|---|---|---|---|
| `vel` / `acc` 인자 | `movesj` 호출 단위 | spin_vel, spin_acc, 프리셋 4개, 📊 자동추천 | 이번 재생 1회 |
| `change_operation_speed` | 컨트롤러 전역 % 배율 | Operation Speed 슬라이더 (1–100%) | 다음 재호출까지 영구 |
| `set_operation_speed_ratio` (펜던트) | 펜던트 override | (외부) | 둘 중 작은 값 적용 |

`/dsr01/motion/change_operation_speed` 서비스 호출. 슬라이더는 드래그 중에는 라벨만 갱신, **손 뗀 순간**에 한 번만 컨트롤러로 전송 (호출 폭주 방지).

### 시연 속도 자동 추천 알고리즘

```
peak = percentile(|Δq| / Δt, 90)   # 90% 분위수, 평균 대신 (정지구간 영향 회피)
sug_vel = round(peak × 1.2 / 5) × 5   # 5°/s 단위 반올림, ×1.2 마진
sug_acc = sug_vel × 2
```

평균이 아닌 90% 분위수 사용 이유: 펜던트 시연 중 사용자가 잠시 생각하는 정지/저속 구간이 평균을 끌어내려 너무 느린 추천이 나오는 것을 방지.

---

## CLI 한 줄 사이클

```bash
ros2 run ros2_move_recoder recorder grasp_v1
ros2 run ros2_move_recoder smoother grasp_v1 --max-pts 60 --eps 0.3
ros2 run ros2_move_recoder player   grasp_v1
```

## GUI 한 사이클

```
● Record → 이름 입력 → (펜던트 조작) → ■ Stop
∿ Smooth (파라미터 조정) → ▶ Play (확인 → 재생)
필요시 🏠 Home / 🛑 E-STOP
```
