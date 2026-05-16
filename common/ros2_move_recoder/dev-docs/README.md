# ros2_move_recoder — 매크로 레코더 개발 문서

Doosan **m0609** 협동로봇용 매크로 레코더/플레이어 패키지.
티칭펜던트로 시연한 동작을 기록 → 평활화 → 균일 속도로 재생한다.

## 문서 구성

| 파일 | 내용 |
|---|---|
| [README.md](README.md) | 이 문서 — 패키지 전체 개요 + 빠른 시작 |
| [CHANGELOG.md](CHANGELOG.md) | **시간순 주요 변경 이력 (mini-jog / DualSense / OnRobot 통합)** |
| [architecture.md](architecture.md) | 데이터 플로우, 스레드 구조, 노드 / 토픽 / 서비스 매핑 |
| [gui.md](gui.md) | `gui.py` 전체 — 위젯 트리, Qt 시그널 그래프, 함정 모음 |
| [pipeline.md](pipeline.md) | recorder → smoother → player 단계별 알고리즘 + JSON 스키마 |
| [data-formats.md](data-formats.md) | `raw.json`, `smooth.json` 필드 명세 |
| [dualsense-plan.md](dualsense-plan.md) | DualSense 통합 **설계안** (초기 계획) |
| [dualsense-mapping.md](dualsense-mapping.md) | DualSense **실제 매핑** + 디버깅 + jog 안정화 정책 |
| [gripper.md](gripper.md) | OnRobot RG2/RG6 그리퍼 통합 + record/smooth/play 흐름 |
| [voice-vision-async-workflow.md](voice-vision-async-workflow.md) | 음성/비전 비동기 워크플로 설계 (계획) |
| [build-and-run.md](build-and-run.md) | 빌드, 실행, 디버깅, 자주 쓰는 ros2 CLI |
| [troubleshooting.md](troubleshooting.md) | 알려진 함정 — DR_init name mangling, executor 충돌 등 |
| [extending.md](extending.md) | 새 모션/필터/UI 패널 추가 가이드 |

## 빠른 시작

```bash
cd ~/cobot_ws
colcon build --packages-select ros2_move_recoder --symlink-install
source install/setup.bash

# GUI (권장) — 시작 시 bringup 모드 선택 다이얼로그
ros2 run ros2_move_recoder gui

# 또는 CLI 파이프라인
ros2 run ros2_move_recoder recorder my_first      # 펜던트 조작 → Enter 두 번
ros2 run ros2_move_recoder smoother my_first      # 평활화 + 다운샘플링
ros2 run ros2_move_recoder player   my_first      # 균일 속도 재생
```

## 패키지 구조

```
ros2_move_recoder/
├── package.xml                   # ament_python, dsr_msgs2, pygame, pymodbus 의존
├── setup.py                      # entry_points: run/recorder/smoother/player/gui
├── ros2_move_recoder/
│   ├── run.py                    # 즉시 실행 모션 테스트 (movej/movel 한 번)
│   ├── recorder.py               # /joint_states 구독 → raw.json
│   ├── smoother.py               # raw.json → smooth.json (Savgol + arc-length)
│   ├── player.py                 # smooth.json → movesj 재생
│   ├── gui.py                    # PyQt5 통합 GUI + mini-jog + DualSense + Gripper
│   ├── dualsense_worker.py       # PS5 컨트롤러 입력 → Qt 시그널 (pygame 기반)
│   ├── gripper_worker.py         # OnRobot RG 그리퍼 워커 (Modbus TCP)
│   └── onrobot.py                # OnRobot RG2/RG6 wrapper (Calibration_Tutorial 채택)
├── records/<name>/               # 매크로 저장소
│   ├── raw.json                  # 원본 기록
│   └── smooth.json               # 평활화 결과
├── macros/                       # (예약 — 미사용)
└── dev-docs/                     # 이 문서
```

## 5개 entry point 한 줄 요약

| 명령 | 역할 | 핵심 의존 |
|---|---|---|
| `ros2 run ros2_move_recoder gui` | 통합 GUI — bringup launch + record/smooth/play/home/estop | PyQt5, DSR_ROBOT2, sensor_msgs |
| `ros2 run ros2_move_recoder recorder <name>` | `/dsr01/joint_states` 고주기 기록 → `raw.json` | sensor_msgs/JointState |
| `ros2 run ros2_move_recoder smoother <name>` | Savitzky-Golay + 정지 압축 + 전환점 보존 + arc-length 다운샘플 | scipy, numpy |
| `ros2 run ros2_move_recoder player <name>` | `smooth.json` → `movesj(vel,acc)` 균일 속도 재생 | DSR_ROBOT2.movesj |
| `ros2 run ros2_move_recoder run` | 즉시 실행 모션 테스트 (코드 편집 후 재빌드 없이 반영) | DSR_ROBOT2 |

## 핵심 설계 결정

1. **단일 namespace `dsr01` 고정** — DSR_ROBOT2가 multi-robot 미지원이므로 코드 전체에서 하드코딩.
2. **모드 분리** — `MANUAL`(기록) ↔ `AUTONOMOUS`(재생). GUI가 자동 감지/전환.
3. **모션은 별도 스레드** — DRFL 50ms 콜백 규약 + Qt 메인 스레드 블로킹 회피. `DsrWorker`가 전담.
4. **Bringup 별도 프로세스** — `dsr_bringup2_rviz.launch.py`를 `subprocess.Popen`으로 띄움. GUI 종료 시 SIGINT.
5. **스플라인 재생은 `movesj`** — `vel/acc` 만 받고 시간 정보 무시 → 등간격 waypoint 필요 → arc-length 다운샘플링이 필수.

## 의존성

- ROS 2 (humble 이상, `dsr_msgs2` 빌드된 `cobot_ws`)
- Python: `rclpy`, `numpy`, `scipy`, `PyQt5`, **`pygame`** (DualSense), **`pymodbus`** (그리퍼)
- DSR_ROBOT2 Python 모듈 (`doosan-robot2/dsr_example2/py/...` 경로 자동 import)
- `dsr_bringup2` 패키지 (real / virtual 모드)
- (선택) DualSense PS5 컨트롤러 — USB 또는 Bluetooth, 커널 `hid-playstation` 자동 인식
- (선택) OnRobot RG2/RG6 그리퍼 — Modbus TCP `192.168.1.1:502` (환경변수로 변경 가능)

## 알아둘 것

- `m0609` 모델 고정 — 다른 모델 사용 시 `gui.py:42`, `run.py:12`, `player.py:18`의 `ROBOT_MODEL` 변경.
- 기록 주파수는 `/dsr01/joint_states` 발행 속도에 종속 (보통 60–100Hz).
- `smoother`의 `--max-pts`는 100 이하 — `movesj`가 최대 100 waypoint 제한.
- 자세한 함정은 [troubleshooting.md](troubleshooting.md) 참조.
