# Data Formats — JSON 스키마

`records/<name>/` 디렉토리에 `raw.json`, `smooth.json` 두 파일.

## raw.json — 원본 기록

```json
{
  "action_name":   "my_first",                  // (gui.py만 추가; recorder.py는 생략)
  "timestamps_ms": [0, 16, 33, 50, ...],        // int[N], 시작 시점 0
  "joints_deg":    [
    [0.0, 0.0, 90.0, 0.0, 90.0, 0.0],           // J1..J6 [deg]
    [0.001, 0.0, 89.998, 0.0, 90.0, 0.0],
    ...
  ],
  "rate_hz_avg":   62.5,                        // float, samples / duration
  "duration_sec":  12.345,
  "samples":       772,                         // = len(joints_deg)
  "recorded_at":   "2026-04-27T07:42:56.123+00:00",  // UTC ISO 8601
  "gripper_widths_mm": [110.0, 110.0, 109.5, ..., 5.0]  // (옵션) joints_deg 와 동일 길이, mm 단위, None 허용
}
```

### `gripper_widths_mm` (옵션 — OnRobot 그리퍼 통합)

OnRobot 그리퍼 활성 + 연결 상태에서 record 진행 시 추가됨. 1개라도 None 아닌 값이 있으면 키 추가, 그렇지 않으면 키 자체 미저장 (기존 raw.json 호환).

| 필드 | 타입 | 설명 |
|---|---|---|
| `gripper_widths_mm` | `(float\|null)[N]` | `len(joints_deg)` 와 동일. mm 단위 (펌웨어 1/10mm → /10). None = 그리퍼 비활성/미연결 sample |

자세한 그리퍼 통합은 [gripper.md](gripper.md) 참조.

### 필드 의미

| 필드 | 타입 | 단위 | 설명 |
|---|---|---|---|
| `timestamps_ms` | int[N] | ms | `time.monotonic()` 기준, 첫 샘플 = 0 |
| `joints_deg` | float[N][6] | degree | J1..J6 순서. `math.degrees(rad)` 변환 |
| `rate_hz_avg` | float | Hz | 평균 샘플링 속도. `/joint_states` 발행 주기에 종속 |
| `duration_sec` | float | s | 마지막 샘플 시각 |
| `samples` | int | - | `len(joints_deg)` |
| `recorded_at` | str | ISO8601 | UTC, microsecond 포함 |

### 무결성 체크
- `len(timestamps_ms) == len(joints_deg) == samples`
- 모든 inner array 길이 = 6
- `timestamps_ms`는 단조 증가
- 인접 샘플 시간 간격 ≈ `1000 / rate_hz_avg` ms (편차 있으면 message drop 또는 콜백 지연)

---

## smooth.json — 평활화 결과

```json
{
  "action_name":  "my_first",
  "waypoints_deg": [
    [0.0, 0.0, 90.0, 0.0, 90.0, 0.0],
    [10.5, 2.3, 88.0, -1.0, 92.0, 5.0],
    ...                                          // K개, K ≤ max_pts ≤ 100
  ],
  "vel": 30.0,                                   // [deg/s]
  "acc": 60.0,                                   // [deg/s²]
  "n_waypoints": 80,
  "n_raw": 772,
  "n_after_stationary_filter": 410,
  "n_turning_points": 7,
  "stationary_eps_deg": 0.5,
  "turning_prominence_deg": 2.0,
  "max_adjacent_jump_deg": 8.42,
  "smoothing": {
    "window": 51,
    "polyorder": 3,
    "max_pts": 80,
    "downsample": "stationary-removed + arc-length + turning-points"
  },
  "gripper_events": [                            // (옵션 — 그리퍼 width 변화 시점)
    {"t_norm": 0.0,  "width_mm": 110.0},
    {"t_norm": 0.42, "width_mm": 5.0},
    {"t_norm": 1.0,  "width_mm": 5.0}
  ]
}
```

### `gripper_events` (옵션)

raw.json 에 `gripper_widths_mm` 가 있을 때 smoother 가 변화 시점만 추출 (변화 ≥5mm 임계). final width 도 명시적으로 추가됨 (재생 시 마지막 상태 보장).

| 필드 | 타입 | 설명 |
|---|---|---|
| `t_norm` | float | 0.0 ~ 1.0 — sample 진행률 (`i / (n_raw - 1)`) |
| `width_mm` | float | 그 시점 그리퍼 width [mm] |

Player 가 amovesj 진행 시간 추정 (`arc_length / (vel × ops_ratio) × 1.10`) 으로 wall-clock timeline 에 따라 `gripper.move(width_mm)` 발사. 자세히 [gripper.md](gripper.md) 참조.

### 필드 의미

| 필드 | 타입 | 의미 |
|---|---|---|
| `waypoints_deg` | float[K][6] | 재생용 관절 좌표. `posj(*w)` → `movesj`에 그대로 전달 |
| `vel` | float | `movesj` 속도 [°/s]. 전 구간 균일 |
| `acc` | float | `movesj` 가속도 [°/s²] |
| `n_waypoints` | int | `len(waypoints_deg)` (= K) |
| `n_raw` | int | 원본 샘플 수 (raw.json의 samples) |
| `n_after_stationary_filter` | int | 정지 압축 후 |
| `n_turning_points` | int | 검출된 peak/trough 수 |
| `stationary_eps_deg` | float | 정지 판정 임계 (사용된 값) |
| `turning_prominence_deg` | float | 전환점 prominence 임계 |
| `max_adjacent_jump_deg` | float | 인접 waypoint 6축 L2 norm의 최대값 — **30° 넘으면 위험 신호** |
| `smoothing.window` | int | 실제 사용된 Savgol window (자동 보정 결과) |
| `smoothing.polyorder` | int | 실제 사용된 polyorder |
| `smoothing.max_pts` | int | 입력 max_pts |
| `smoothing.downsample` | str | 알고리즘 식별자 (버전 변경 시 갱신) |

### 검증 지표

| 지표 | 정상 범위 | 이상 시 대응 |
|---|---|---|
| `n_waypoints` | 5–100 | <5: 기록 너무 짧음 / 100: max_pts 도달, 더 많이 필요하면 분할 |
| `n_after_stationary_filter / n_raw` | 0.3–0.9 | <0.3: 정지 구간 과다 → eps 줄이기 / >0.95: 전혀 정지 없음 |
| `max_adjacent_jump_deg` | <15° | >30°: 점프 위험, max_pts 늘리거나 prom 줄이기 |

---

## 디렉토리 구조

```
records/
└── my_first/                  ← <name>
    ├── raw.json
    └── smooth.json            ← smoother 실행 후 생성
```

- 폴더가 없으면 recorder가 자동 생성
- 같은 name으로 다시 record 하면 **raw.json 덮어쓰기** (smooth.json은 남지만 stale)
- GUI 사이드바의 `[raw]` `[smooth]` 태그로 어느 단계까지 진행됐는지 표시
- `[삭제]` 버튼은 `shutil.rmtree(records/<name>)`

---

## 호환성 노트

- `joints_deg` 의 J1..J6 순서는 **항상 정렬됨**. recorder가 `name_to_pos[f"joint_{i}"]`로 명시적 매핑 → `JointState.name` 순서가 바뀌어도 안전
- `timestamps_ms`는 player가 사용하지 않음 (movesj는 시간 무시). 디버깅/시각화 전용
- 새 필드 추가 시 기존 player/GUI는 모름 → 전체 down 가능. 기존 reader가 `.get(key, default)` 쓰는지 확인 필요
