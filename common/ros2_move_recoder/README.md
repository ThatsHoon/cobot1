# ros2_move_recoder

Doosan m0609 협동로봇 매크로 기록 / 평활화 / 재생 GUI.

DualSense PS5 컨트롤러로 로봇을 직접 조작하며 동작을 녹화하고, Savitzky-Golay 평활화를 거쳐 재생합니다. OnRobot RG2/RG6 그리퍼의 Modbus TCP 제어도 기록·재생에 통합되어 있습니다.

---

## 주요 기능

| 기능 | 설명 |
|---|---|
| **Record / Smooth / Play** | 관절 궤적 녹화 → 평활화 → 재생 파이프라인 |
| **DualSense 컨트롤러** | 조인트 jog / TCP jog / 워크플로 전체 조작 |
| **OnRobot 그리퍼** | RG2/RG6 Modbus TCP 제어 — 기록·재생 연동 |
| **Mini-Jog 패널** | GUI 버튼으로 개별 조인트 ± 제어 (default 80°/s) |
| **Virtual / Real 모드** | 시작 시 선택 — virtual 모드에서는 그리퍼 연결 생략 |

---

## 설치

### 의존 패키지

```bash
sudo apt install \
  python3-numpy python3-scipy \
  python3-pyqt5 python3-pygame python3-pymodbus
```

### 워크스페이스 배치

```
~/cobot_ws/
└── src/
    ├── doosan-robot2/        ← 별도 클론 필요 (dsr_msgs2, DSR_ROBOT2, dsr_bringup2)
    └── ros2_move_recoder/    ← 이 레포
```

```bash
cd ~/cobot_ws/src
git clone https://github.com/ThatsHoon/ros2_move_recoder.git
```

### 빌드 및 실행

```bash
cd ~/cobot_ws
colcon build --packages-select ros2_move_recoder
source install/setup.bash
ros2 run ros2_move_recoder gui
```

---

## 사용 흐름

1. 실행 시 **Bringup 모드 선택** (Virtual / Real / 외부 launch 사용)
2. GUI 좌측 패널에서 **액션 이름 지정** 또는 자동 생성
3. **Record** → 로봇 수동 조작 또는 DualSense 패드로 jog → **Stop**
4. **Smooth** → Savitzky-Golay 평활화 + 그리퍼 이벤트 추출
5. **Play** → 녹화된 궤적 재생 (그리퍼 동작 포함)

DualSense 활성화: 메뉴 `[컨트롤러] → DualSense 활성화` (또는 `Ctrl+D`)

---

## DualSense 버튼 매핑

### 버튼

| 버튼 | 기능 |
|---|---|
| **Create** (짧게) | Record 시작 / 정지 토글 |
| **Create** (2초 hold) | 새 액션 프로파일 자동 생성 후 Record 시작 |
| **○** | Smooth + Play (smooth.json 있으면 즉시 Play) |
| **×** (짧게) | 재생 일시정지 ↔ 재개 |
| **×** (1초 hold) | 🛑 비상 정지 (E-Stop) + Home 자동 복귀 |
| **△** | Home 위치 복귀 |
| **□** | 그리퍼 Open ↔ Close 토글 |
| **Options** | 🦾 JOINT ↔ 🌐 TCP·BASE jog 모드 전환 |
| **L2** (hold) | 현재 선택 조인트 속도 −1°/s (10Hz 반복) |
| **R2** (hold) | 현재 선택 조인트 속도 +1°/s (10Hz 반복) |
| **L3 + R3** 동시 | MANUAL ↔ AUTONOMOUS 모드 전환 |

팝업 다이얼로그가 떠 있을 때: **○ → Yes/OK**, **× → No/Cancel**

### 스틱 (jog 모드에 따라 다름)

| 모드 | 좌측 스틱 | 우측 스틱 |
|---|---|---|
| **🦾 JOINT** | ↑/→ = 다음 조인트 선택, ↓/← = 이전 조인트 | 선택된 조인트 jog (최대 80°/s) |
| **🌐 TCP·BASE** | 좌/우 → ±X (mm/s), 위/아래 → ±Y (mm/s) | 위/아래 → ±Z (mm/s, 최대 50) |

> 우측 스틱을 hold 한 채 좌측 스틱으로 조인트를 바꾸면 손을 떼지 않고 연속 jog 전환 가능.

---

## 환경변수

| 변수 | 기본값 | 설명 |
|---|---|---|
| `ROS_DOMAIN_ID` | `0` | `~/.bashrc` 값 자동 상속 |
| `GRIPPER_IP` | `192.168.1.1` | OnRobot 그리퍼 Modbus IP |
| `GRIPPER_PORT` | `502` | Modbus TCP 포트 |
| `GRIPPER_TYPE` | `rg2` | `rg2` / `rg6` |

```bash
GRIPPER_IP=192.168.1.50 ros2 run ros2_move_recoder gui
```

## 로봇 IP 변경

현장 로봇 IP 가 `192.168.1.100` 이 아닌 경우 `gui.py` 두 곳 수정:

```python
# BringupManager.HOST_BY_MODE
"real": "192.168.1.100",  # ← 변경

# BringupDialog 표시 텍스트 (L181)
("Real    ·  192.168.1.100 : 12345", ...)  # ← 변경
```

---

## 녹화 데이터

`records/` 폴더에 액션별로 저장됩니다. 다른 PC 로 이관 시 폴더 통째로 복사.

```
records/
└── action_20260503_132550/
    ├── raw.json      ← 원본 궤적 + 그리퍼 width
    └── smooth.json   ← 평활화 궤적 + 그리퍼 이벤트
```

---

## 문서

자세한 내용은 [`dev-docs/`](dev-docs/README.md) 참조.

| 문서 | 내용 |
|---|---|
| [architecture.md](dev-docs/architecture.md) | 전체 아키텍처 + Qt 시그널 그래프 |
| [pipeline.md](dev-docs/pipeline.md) | Record → Smooth → Play 파이프라인 |
| [dualsense-mapping.md](dev-docs/dualsense-mapping.md) | DualSense 버튼 매핑 상세 |
| [gripper.md](dev-docs/gripper.md) | OnRobot 그리퍼 통합 |
| [data-formats.md](dev-docs/data-formats.md) | raw.json / smooth.json 스펙 |
| [troubleshooting.md](dev-docs/troubleshooting.md) | 자주 발생하는 문제 해결 |
| [CHANGELOG.md](dev-docs/CHANGELOG.md) | 변경 이력 |

---

## 라이선스 / 저작권

Copyright (c) 2026 ErifKim (gagea45@gmail.com). All rights reserved.

개인적·비상업적 목적의 사용·수정은 허용합니다.
상업적 이용, 판매, 재배포, 제품/서비스 통합은 저작권자의 서면 동의 없이 금지합니다.

자세한 내용은 [LICENSE](LICENSE) 참조.
