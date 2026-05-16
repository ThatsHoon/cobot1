# Build & Run — 빌드, 실행, 디버깅 CLI

## 1. 빌드

```bash
cd ~/cobot_ws
colcon build --packages-select ros2_move_recoder --symlink-install
source install/setup.bash
```

`--symlink-install` 권장: Python 파일 수정 시 재빌드 불필요 (entry_points만 빌드 시 박힘).

### 빌드 검증

```bash
ros2 pkg list | grep ros2_move_recoder         # 설치 확인
ros2 pkg executables ros2_move_recoder         # entry_points 확인
# 출력:
#   ros2_move_recoder gui
#   ros2_move_recoder player
#   ros2_move_recoder recorder
#   ros2_move_recoder run
#   ros2_move_recoder smoother
```

## 2. Bringup 먼저 (모든 ros2 run 전 필수)

### Real (실 로봇)
```bash
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
    mode:=real host:=192.168.1.100 port:=12345 model:=m0609
```

### Virtual (시뮬레이터, 로봇 없이 테스트)
```bash
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
    mode:=virtual host:=127.0.0.1 port:=12345 model:=m0609
```

⚠️ virtual에 real IP 절대 금지 → spawner timeout.

GUI를 쓰면 위 launch는 자동 실행됨 (`BringupDialog` 선택).

## 3. 실행 패턴

### 패턴 A — GUI 한 번에
```bash
ros2 run ros2_move_recoder gui
# → BringupDialog → real/virtual/skip 선택 → 자동 진행
```

### 패턴 B — CLI 분리 (디버깅, 자동화)
```bash
# Terminal 1: bringup
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=virtual host:=127.0.0.1 port:=12345 model:=m0609

# Terminal 2: 기록 (Enter 두 번)
ros2 run ros2_move_recoder recorder grasp_v1

# Terminal 3: 평활화
ros2 run ros2_move_recoder smoother grasp_v1 --max-pts 60 --eps 0.3

# Terminal 4: 재생
ros2 run ros2_move_recoder player grasp_v1
```

### 패턴 C — 즉시 실행 모션 (run.py)
```bash
ros2 run ros2_move_recoder run
# run.py 의 my_motion() 함수만 수정 → 재빌드 없이 즉시 반영 (--symlink-install 덕분)
```

## 4. 디버깅 CLI

### ROS 2 그래프 점검

```bash
# 노드 목록 (dsr_control_node, robot_state_publisher 등 모두 보여야 함)
ros2 node list

# 토픽 + 타입
ros2 topic list -t

# 한 토픽 상세 (QoS 양쪽 비교 — 안 받히면 1순위 점검)
ros2 topic info -v /dsr01/joint_states

# 발행 주기 확인
ros2 topic hz /dsr01/joint_states
# 정상: 60-100Hz

# 관절값 실시간 보기
ros2 topic echo /dsr01/joint_states --once

# 시스템 종합 진단
ros2 doctor --report
```

### 로그 모니터링

```bash
# 모든 노드 로그
ros2 topic echo /rosout

# 특정 노드만
ros2 topic echo /rosout --filter 'm.name == "ros2_move_recoder.macro_gui_node"'

# WARN 이상만 (level: DEBUG=10/INFO=20/WARN=30/ERROR=40)
ros2 topic echo /rosout --filter 'm.level >= 30'

# 두산 컨트롤러 알람 (rosout과 별개 채널)
ros2 topic echo /dsr01/dsr_robot2_msg

# rqt_console GUI (필터링 편함)
ros2 run rqt_console rqt_console
```

### DSR 서비스 직접 호출 테스트

```bash
# 모드 조회
ros2 service call /dsr01/system/get_robot_mode dsr_msgs2/srv/GetRobotMode

# AUTONOMOUS 전환
ros2 service call /dsr01/system/set_robot_mode dsr_msgs2/srv/SetRobotMode "{robot_mode: 1}"

# 홈 이동
ros2 service call /dsr01/motion/move_joint dsr_msgs2/srv/MoveJoint \
  "{pos: [0.0, 0.0, 90.0, 0.0, 90.0, 0.0], vel: 30.0, acc: 60.0, time: 0.0, sync_type: 0}"
```

### 데몬 캐시 무효화

```bash
# 좀비 노드/토픽이 ros2 node list 에 계속 보일 때
ros2 daemon stop && ros2 daemon start
```

## 5. 자주 쓰는 진단 시나리오

### "GUI 켰는데 mode 가 ?"
1. `ros2 topic list | grep dsr01` → controller 토픽 보이나?
2. 안 보이면 → bringup 안 떴거나 죽었음 → 터미널 로그 확인
3. 보이면 → service ready 대기 중. 2-3초 더 기다림
4. 그래도 ? → `ros2 service call /dsr01/system/get_robot_mode ...` 직접 호출

### "재생 했는데 movesj rc=非0"
1. `rc` 값을 두산 에러 코드 표와 대조
2. 자주 보는 것:
   - `0` 정상
   - `-1` 모드 X (AUTONOMOUS 아님)
   - `-2` waypoint 100개 초과
   - `-3` 인접 점프 과대 (max_adjacent_jump_deg > ~30°)
   - 보호 정지 발동 시도 추적 → `ros2 topic echo /dsr01/dsr_robot2_msg`

### "GUI 멈춤"
1. DSR 호출이 데드락 — 보통 controller down 또는 service 미준비
2. `Ctrl+C` 한 번으로는 안 죽으면 → 다른 터미널에서 `pkill -f gui.py`
3. bringup도 같이 죽음 (subprocess 부모-자식 관계 + SIGINT propagation)

## 6. 환경 점검 한 줄

```bash
echo $ROS_DISTRO && echo $ROS_DOMAIN_ID && which ros2 && ls install/ros2_move_recoder
```

기대값:
- `ROS_DISTRO`: humble (또는 더 상위)
- `ROS_DOMAIN_ID`: 0 (또는 팀 컨벤션 값)
- `ros2`: `/opt/ros/<distro>/bin/ros2`
- `install/ros2_move_recoder/`: 빌드 결과물

## 7. PyQt5 의존성 (Ubuntu)

```bash
sudo apt install python3-pyqt5 python3-numpy python3-scipy
```

`python3 -c "from PyQt5 import QtWidgets"` → 무에러면 OK.
