# 음성·비전·비동기 루프 통합 워크플로우

> **상태**: 설계안 (구현 전)
> **작성일**: 2026-04-28
> **목적**: m0609 로봇팔이 (1) 사물을 인식하고 (2) 음성 명령을 받아 (3) 동작 중에도 외부 이벤트에 반응할 수 있는 통합 시스템 설계
> **연관**: `dualsense-plan.md` (수동 조작) ↔ 본 문서 (자동 인지·반응)

---

## 0. 전체 그림

```
┌──────────────────────────────────────────────────────────────┐
│                     MainLoop (asyncio)                       │
│  ────────────────────────────────────────────────────────    │
│  - amovej / amovel 으로 모션 명령 발사 (non-blocking)        │
│  - asyncio.gather() 로 동시에 3개 워커 spin                  │
└──────┬─────────────────┬─────────────────┬───────────────────┘
       │                 │                 │
       ▼                 ▼                 ▼
┌──────────────┐  ┌──────────────┐  ┌────────────────┐
│ vision_node  │  │  tts_node    │  │  motion_node   │
│ (YOLO+Depth) │  │ (wake+cmd)   │  │ (DSR async)    │
└──────┬───────┘  └──────┬───────┘  └────────┬───────┘
       │                 │                   │
       │ /objects (PoseArray + class)        │
       │                 │ /voice_command (String)
       │                 │                   │
       └────────────►  ROS 2 Topics  ◄───────┘
                     (asyncio bridge)
```

---

## 1. 사물 분류 워크플로우 (Vision)

### 목표
RGB-D 카메라(예: Intel RealSense D435/D455)로 본 frame 에서 **특정 사물**(컵, 재료병, 그리퍼, 사람 손 등)을 분류하고 BASE 좌표계상의 3D 위치로 변환해 motion_node 에 전달.

### Phase 1 — 데이터 수집 + 라벨링
| 단계 | 도구 | 산출물 |
|---|---|---|
| 1.1 카메라 마운트 + 캘리브레이션 | RealSense SDK + ROS 2 `realsense2_camera` | extrinsics (camera ↔ base_link, TF) |
| 1.2 대상 사물 선정 | (수동) — 시연 시나리오 기반 | class list (≤10개 권장) |
| 1.3 이미지 수집 (≥500장/class) | `rosbag2 record /camera/color/image_raw` | rosbag → png 추출 |
| 1.4 라벨링 | Roboflow / Label Studio / CVAT | YOLO format labels |

### Phase 2 — YOLO 파인튜닝
| 단계 | 도구 | 비고 |
|---|---|---|
| 2.1 베이스 모델 선정 | **YOLOv8n** 또는 **YOLOv11n** (경량) | edge inference 가능 |
| 2.2 train | `ultralytics` Python API | `yolo train data=cfg.yaml model=yolov8n.pt epochs=100 imgsz=640` |
| 2.3 검증 | mAP@50, confusion matrix | mAP < 0.85 → 데이터 보강 |
| 2.4 배포 | `.pt` → `.onnx` (선택, TensorRT 가속) | `~/cobot_ws/models/objects_v1.pt` |

### Phase 3 — 실시간 노드
**새 패키지**: `vision_pipeline/` (별도 ROS 2 ament_python 패키지)

```
vision_pipeline/
├── vision_node.py            # main inference loop
├── object_to_3d.py           # depth + intrinsics → BASE 좌표
└── config/objects_v1.yaml    # class names + grasp offsets
```

**핵심 흐름** (`vision_node.py`):
```python
# subscribe
sub_rgb   = self.create_subscription(Image, '/camera/color/image_raw', cb_rgb, 10)
sub_depth = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', cb_depth, 10)

# inference @ ~10 Hz
results = model.predict(rgb_frame, conf=0.5)
for box in results[0].boxes:
    u, v = box.xywh[:2]
    z = depth_frame[v, u] * 0.001  # mm → m
    x, y = pixel_to_camera(u, v, z, intrinsics)
    pose_base = tf.transform(x, y, z, 'camera_link', 'base_0')
    pub_objects.publish(ObjectPose(class_name=cls, pose=pose_base, conf=conf))
```

**publish topic**: `/vision/detected_objects` (custom msg `ObjectPoseArray`)

### Phase 3.5 — Depth → 로봇팔 이동 좌표 추출 (상세)

YOLO 가 2D 박스만 주므로, depth 와 카메라 intrinsics/extrinsics 로 **m0609 BASE 좌표계**의 3D `posx(x, y, z, a, b, c)` 까지 환산해야 amovel 호출 가능.

#### 단계 0 — 카메라 마운트 결정

| 방식 | 위치 | 장점 | 단점 | 추천 |
|---|---|---|---|---|
| **eye-to-hand** | 로봇 외부 (작업대 천장/측면 고정) | TF 고정, 캘리브 1회 | 가림 발생, 시야 한정 | 고정 작업대 시나리오 |
| **eye-in-hand** | 로봇 6번 관절(flange)에 부착 | 시점 자유, 가림 ↓ | 동작마다 TF 변동 | 탐색·접근 시나리오 |

m0609 + RealSense D435 시연 환경 → **eye-to-hand** 권장 (D435 무게 ~75g 면 eye-in-hand 도 가능).

#### 단계 1 — 카메라 ↔ 로봇 캘리브레이션 (extrinsics)

**목표**: `T_base_camera` (4×4 동차행렬) 산출.

| 방법 | 도구 | 정확도 | 소요 |
|---|---|---|---|
| **AprilTag 기반 hand-eye** | `easy_handeye2` (ROS 2) + apriltag_ros | ±2mm | 30분, 권장 |
| ArUco 마커 + OpenCV `calibrateHandEye()` | OpenCV 직접 | ±3mm | 1시간 |
| 수동 jog + 4점 측정 | 자체 스크립트 | ±5~10mm | 빠름, MVP |

**수동 4점 캘리브 (MVP)**:
1. 마커(체커보드 1cm 격자) 를 작업대에 고정
2. m0609 TCP 로 마커 4개 모서리를 차례로 터치 → BASE 좌표 4개 기록
3. 같은 4점을 카메라 RGB 이미지에서 클릭 → camera 좌표 4개 산출 (depth 사용)
4. `cv2.solvePnP()` 또는 `np.linalg.lstsq` 로 affine transform 추정
5. 결과를 `static_transform_publisher` 로 TF 발행:
```bash
ros2 run tf2_ros static_transform_publisher \
  0.45 0.20 0.80  0 1.5708 0  base_0 camera_link
```

**검증**: 알려진 위치(자 끝 등)에 사물 두고 vision pipeline 출력 좌표 ↔ 실측 비교. ±5mm 이내면 OK.

#### 단계 2 — 픽셀 + Depth → Camera 좌표 (intrinsics)

RealSense `/camera/color/camera_info` 토픽에서 `K` 매트릭스 (fx, fy, cx, cy) 획득.

```python
def pixel_to_camera(u, v, z_m, K):
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    x = (u - cx) * z_m / fx
    y = (v - cy) * z_m / fy
    return np.array([x, y, z_m])    # camera frame
```

**중요**: depth 는 **aligned-to-color** 토픽 사용 (`/camera/aligned_depth_to_color/image_raw`). RGB-Depth resolution mismatch 회피.

#### 단계 3 — Depth 안정화 (노이즈 대책)

D435 raw depth 의 픽셀 단위 노이즈는 30cm 거리에서 ±2~3mm. 단일 픽셀 사용 시 outlier 위험.

```python
def stable_depth(depth_frame, u, v, patch=5):
    # 박스 중심 ±patch 영역
    roi = depth_frame[v-patch:v+patch+1, u-patch:u+patch+1]
    valid = roi[(roi > 100) & (roi < 2000)]   # 10cm ~ 2m 만 유효
    if len(valid) < 5:
        return None
    return np.median(valid) * 0.001            # mm → m, median (outlier robust)
```

**추가 안정화**:
- 다중 frame 평균 (3~5 frame moving median) — 정적 사물에 적용
- bilateral filter (depth 노이즈 제거, edge 보존)
- depth hole inpainting (RealSense `rs.spatial_filter`)

#### 단계 4 — Camera → BASE 변환

`tf2_ros` 으로 자동:
```python
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped

self.tf_buffer = Buffer()
TransformListener(self.tf_buffer, self)

def to_base(self, pt_camera):
    p = PointStamped()
    p.header.frame_id = 'camera_link'
    p.header.stamp = self.get_clock().now().to_msg()
    p.point.x, p.point.y, p.point.z = pt_camera
    p_base = self.tf_buffer.transform(p, 'base_0', timeout=Duration(seconds=0.1))
    return [p_base.point.x, p_base.point.y, p_base.point.z]   # meters in BASE
```

#### 단계 5 — 그립 자세 산출 (orientation 결정)

3D 위치만으로는 `amovel(posx)` 6DOF 부족. 그립 자세 a,b,c (Euler ZYZ for m0609) 결정 필요.

**방법 A — 클래스별 사전정의 grip preset (MVP)**:
```yaml
# config/objects_v1.yaml
cup:
  grasp_offset_z_mm: 80      # 컵 높이 절반 위
  grasp_orientation_zyz_deg: [0, 180, 0]   # tool down (Z 축 아래)
  approach_height_mm: 100
bottle:
  grasp_offset_z_mm: 120
  grasp_orientation_zyz_deg: [0, 180, 0]
  approach_height_mm: 150
```

**방법 B — 3D PCA 기반 (사물의 주축 추정)**:
- depth 이미지의 박스 영역 → point cloud (open3d 변환)
- `o3d.geometry.PointCloud.compute_mean_and_covariance()` → eigen vectors
- 가장 긴 축 = 그립 방향, 두 번째 = approach
- 컵/병 같은 회전대칭 객체에 효과적

**방법 C — pose estimation 모델** (고도화):
- DOPE / FoundationPose / GraspNet — 6DOF pose 직접 회귀
- 학습 데이터 비용 ↑, 정확도 ↑

**MVP 권장**: 방법 A. 사물별 grip_preset YAML.

#### 단계 6 — Approach + Grasp + Retreat 시퀀스 생성

단일 좌표가 아니라 3-step 경유점:
```python
def make_grasp_waypoints(obj_pose_base, preset):
    x, y, z = obj_pose_base
    a, b, c = preset["grasp_orientation_zyz_deg"]
    grasp_z = z + preset["grasp_offset_z_mm"] / 1000
    approach_z = grasp_z + preset["approach_height_mm"] / 1000

    return [
        ("approach", posx(x*1000, y*1000, approach_z*1000, a, b, c)),
        ("grasp",    posx(x*1000, y*1000, grasp_z*1000,    a, b, c)),
        ("lift",     posx(x*1000, y*1000, approach_z*1000, a, b, c)),
    ]
```

motion_executor 가 이 리스트를 amovel 로 순차 실행 + 그리퍼 닫기 (grasp 단계 후).

#### 단계 7 — 작업 영역 안전 검증

좌표가 m0609 작업 반경(약 900mm) + 안전 영역 안인지 확인:
```python
def is_reachable(x_m, y_m, z_m):
    r = math.sqrt(x_m**2 + y_m**2)
    if r > 0.85: return False     # 반경 한계 (안전 마진)
    if z_m < 0.05 or z_m > 0.90: return False  # 테이블 충돌 + 천장
    return True
```

위반 시 `amovel` 호출 전에 reject + 사용자 알림 (lightbar 적색).

#### 단계 8 — 발행 메시지 확장

`ObjectPose` custom msg 권장 필드:
```
string class_name
geometry_msgs/Pose pose          # BASE frame, grasp pose 포함
float32 confidence               # YOLO conf
float32 distance_m               # 카메라 거리 (정렬 우선순위)
geometry_msgs/Pose approach_pose # approach 위치
bool reachable                   # 작업영역 검증 결과
builtin_interfaces/Time stamp
```

### Phase 4 — 행동 응용
- `motion_node` 가 `/vision/detected_objects` 구독
- "컵" class 중 `reachable=true` & 가장 가까운 인스턴스 선택
- Phase 3.5 단계 6 의 approach → grasp → lift 시퀀스 amovel 발사
- 사람 손 검출 시 → 정지 / 후퇴

### Phase 5 — 정확도 보강 루프
- 실 사용 중 mis-classification 발생 → frame + label 자동 저장 (`hard_negatives/`)
- 주 1회 batch 로 재학습 (active learning)
- 그립 실패 시 (그리퍼 force-feedback or 비전 재확인) → grip_preset 파라미터 조정 로그 누적

### 의존성
- `pip install ultralytics opencv-python pyrealsense2 open3d`
- `apt install ros-humble-realsense2-camera ros-humble-image-pipeline ros-humble-tf2-ros ros-humble-apriltag-ros`
- (캘리브) `pip install easy_handeye2` 또는 OpenCV 직접

---

## 2. TTS 워크플로우 (Wake-word + Command Mapping)

### 목표
"비그루비" 호출어 → 5~6초 명령 녹음 → 의도 분류 → 로봇 동작 트리거.
**상시 가벼운 wake 모델**만 돌고 명령 파싱은 호출 시에만 무거운 모델 호출.

### Phase 1 — Wake-word 엔진 ("비그루비" 전용)

| 옵션 | 장단점 | 추천 상황 |
|---|---|---|
| **openWakeWord** | OSS, custom wake-word 학습 가능, 경량 (~50MB CPU) | 권장 |
| Porcupine (Picovoice) | 정확도 ↑, 한국어 custom wake 지원 | 라이선스 비용 |
| Snowboy (deprecated) | — | 비추천 |

**선정**: **openWakeWord** + custom training

| 단계 | 도구 | 비고 |
|---|---|---|
| 1.1 "비그루비" 음성 수집 | 본인 + 동료 ≥30회 (다양 톤/거리) | 16kHz mono wav |
| 1.2 negative 샘플 | LibriSpeech KO subset 또는 youtube 한국어 podcast | ≥30분 |
| 1.3 학습 | openWakeWord training script | `~/cobot_ws/models/bigroovy.tflite` |
| 1.4 inference | `openwakeword` Python lib | CPU 점유 < 5% |

### Phase 2 — 음성 녹음 + STT (호출 후 5~6초)

```python
async def on_wake_detected():
    audio = await record_async(seconds=6, sr=16000)
    text  = await stt_transcribe(audio)   # OpenAI Whisper or Google STT
    cmd   = classify_command(text, COMMAND_MAP)
    await motion_executor.dispatch(cmd)
```

**STT 옵션**:
- **whisper.cpp** (로컬, small/base 모델) — 오프라인, 한국어 지원 양호
- **faster-whisper** (CTranslate2 백엔드) — 로컬, 더 빠름
- Google Cloud STT — 정확도 ↑, 네트워크 필수

**선정**: **faster-whisper small** (로컬, RTX 환경 권장)

### Phase 3 — 명령 매핑

`config/command_map.yaml`:
```yaml
stop:
  desc: "멈추라는 내용 (멈춰, 정지, 그만, 스톱, 셧다운 등)"
  examples: ["멈춰", "정지해", "그만", "스톱"]
  action: motion_executor.stop
home:
  desc: "홈/집/시작 위치로 복귀"
  examples: ["홈으로", "집으로 가", "시작 위치"]
  action: motion_executor.go_home
pick:
  desc: "특정 사물을 잡아라"
  examples: ["컵 잡아", "병 들어", "저거 가져와"]
  action: motion_executor.pick   # vision 결과와 결합
```

**분류 방식 두 가지**:

| 방식 | 비용 | 정확도 | 추천 |
|---|---|---|---|
| Keyword + fuzzy match (rapidfuzz) | 0 (CPU만) | 단순 명령 ↑ | MVP |
| LLM zero-shot (claude-haiku 4.5) | API 비용 | 의도 모호 명령 ↑ | 운영 |

**MVP**: 키워드 매칭 → **Phase 후반**: claude-haiku 4.5 호출로 업그레이드 (`{cmd_keys, user_text} → key`)

### Phase 4 — 노드 구조

**새 패키지**: `voice_command/`

```
voice_command/
├── tts_node.py             # main: wake → record → stt → classify → publish
├── wake_detector.py        # openWakeWord wrapper
├── recorder.py             # asyncio mic recording
├── command_classifier.py   # keyword + (옵션) LLM
└── config/command_map.yaml
```

**publish topic**: `/voice_command` (`std_msgs/String`)

### Phase 5 — UX 피드백
- wake 감지 → **DualSense lightbar 보라색 펄스** (dualsense-plan.md 와 통합)
- 녹음 중 → lightbar 흰색 정지광
- 명령 인식 실패 → "다시 말씀해주세요" TTS 출력 (선택)

### 의존성
- `pip install openwakeword faster-whisper sounddevice rapidfuzz pyyaml`
- (옵션) anthropic SDK + ANTHROPIC_API_KEY

---

## 3. 비동기 메인루프 (Async Motion)

### 목표
모션 실행 중에도 vision/tts/제어 메시지를 처리 가능. **DSR async API** (`amovej`, `amovel`) + `asyncio` 기반 이벤트 루프.

### 핵심 변경 (기존 sync → async)

| 기존 (sync) | 변경 (async) | 효과 |
|---|---|---|
| `movej(pose)` (블로킹 ~수초) | `amovej(pose)` (즉시 리턴, 백그라운드 실행) | UI freeze 제거, 동시 이벤트 처리 가능 |
| `movel(pos)` | `amovel(pos)` | 동일 |
| `movesj(waypoints)` | (DSR async 미지원) → 대안: 외부에서 `mwait()` 폴링 | spline 재생 중에도 비전 갱신 처리 |
| `time.sleep(0.5)` | `await asyncio.sleep(0.5)` | 다른 task 양보 |

### 메인루프 구조

```python
# main_async.py
import asyncio
from rclpy.executors import MultiThreadedExecutor

async def main():
    rclpy.init()
    node = MotionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    # background spin
    spin_task = asyncio.create_task(asyncio.to_thread(executor.spin))

    # 3 concurrent workers
    await asyncio.gather(
        vision_consumer(node),     # /vision/detected_objects 구독 + 상태 갱신
        voice_consumer(node),      # /voice_command 구독 + 행동 dispatch
        motion_supervisor(node),   # 현재 모션 상태 추적, 중단 처리
    )

async def voice_consumer(node):
    async for cmd in node.voice_queue:    # asyncio.Queue
        if cmd == "stop":
            await node.dsr.stop_async()
        elif cmd == "home":
            await node.dsr.amovej_async(HOME_POSE)
        elif cmd == "pick":
            target = node.latest_objects.get("cup")
            if target:
                await node.dsr.amovel_async(target.pose)
```

### asyncio ↔ ROS 2 브리지

ROS 2 콜백은 executor thread 에서 실행 → asyncio.Queue 로 main loop 에 전달:
```python
def cb_voice(self, msg):
    asyncio.run_coroutine_threadsafe(self.voice_queue.put(msg.data), self.loop)
```

### asyncio ↔ DSR 브리지

DSR async API (`amovej`) 는 future 가 아니라 별도 callback 발사. asyncio Future 로 감쌈:
```python
async def amovej_async(self, pose):
    future = asyncio.get_event_loop().create_future()
    def on_done(rc):
        future.set_result(rc)
    set_motion_done_callback(on_done)   # DSR 콜백 1회용
    amovej(pose)
    return await future
```

### 우선순위 / 선점 (Preemption)

| 이벤트 | 동작 |
|---|---|
| `voice_command == "stop"` | 즉시 `stop(DR_SSTOP)` 호출, 진행 중 task cancel |
| 비전: 사람 손 detect | 현재 amovel cancel + 후퇴 amovel 발사 |
| dualsense touchpad 누름 | stop + manual mode |
| 일반 명령 (`pick`) | 진행 중 task 끝나길 await 후 실행 |

**구현**: `asyncio.Task` 의 `cancel()` + DSR `stop()` 조합. priority queue 도입 검토.

### Watchdog
- 모션 시작 후 N초 무응답 → timeout, stop, 알림
- vision/voice 노드 heartbeat 끊기면 안전 모드

### 새 패키지
**위치**: `motion_executor/` (또는 ros2_move_recoder 안에 통합)

```
motion_executor/
├── main_async.py
├── dsr_async.py             # amovej/amovel asyncio wrapper
├── motion_supervisor.py     # task lifecycle + preemption
├── voice_consumer.py
├── vision_consumer.py
└── config/policies.yaml     # 우선순위 / 선점 규칙
```

### 의존성
- DSR_ROBOT2 의 `amovej`, `amovel`, `set_motion_done_callback` 지원 확인 필요 (펌웨어 의존)
- Python ≥ 3.10 (asyncio.TaskGroup 활용)
- `pip install asyncio-mqtt` (선택, MQTT 통신 도입 시)

---

## 4. 통합 워크플로우 (3 features 함께 동작)

```
시나리오: "비그루비, 컵 가져와"
─────────────────────────────────────────────────
T+0.0s  [tts]  wake "비그루비" 감지 → lightbar 보라
T+0.1s  [tts]  6초 녹음 시작
T+6.1s  [tts]  whisper STT → "컵 가져와"
T+6.3s  [tts]  classify → "pick"
T+6.3s  [motion] /voice_command "pick" 수신
T+6.3s  [vision] /vision/detected_objects 에서 "cup" 최신 위치 lookup
T+6.4s  [motion] amovel(cup_pose + grasp_offset) 발사 (non-blocking)
T+6.4s  [main_loop] 다음 이벤트 대기 (모션 진행 중에도)
T+8.0s  [tts]  "비그루비, 멈춰" 감지
T+14.0s [motion] stop(DR_SSTOP) — 진행 중 amovel cancel
```

### 패키지 의존 그래프

```
ros2_move_recoder ──┬── (기존: record/smooth/play + dualsense jog)
                    │
motion_executor ────┼── amovej/amovel async wrapper
                    │
voice_command ──────┘── tts_node → /voice_command
vision_pipeline ───── vision_node → /vision/detected_objects
```

---

## 5. 마일스톤 (실행 순서 권장)

| M | 기간 | 산출물 |
|---|---|---|
| **M1** | 1주 | `motion_executor` async 골격 + amovej 래퍼 + dummy stop 시나리오 검증 |
| **M2** | 2주 | `vision_pipeline` YOLOv8n 파인튜닝 → 실시간 좌표 발행 |
| **M3** | 2주 | `voice_command` openWakeWord + faster-whisper + 키워드 매칭 |
| **M4** | 1주 | 통합: voice → vision lookup → amovel 시나리오 (pick) |
| **M5** | 1주 | claude-haiku 4.5 LLM 명령 분류 업그레이드 + 우선순위 정책 |
| **M6** | 1주 | DualSense + voice + vision 동시 동작 안정성 검증 |

---

## 6. 검증 (end-to-end)

### 6.1 단위 검증
- [ ] vision_node: 컵 들고 카메라 앞에 → `/vision/detected_objects` 에 cup pose 발행
- [ ] tts_node: "비그루비" 호출 후 "홈으로" → `/voice_command "home"` 발행
- [ ] motion_executor: stop 명령 도중 amovej 진행 중 → 즉시 정지

### 6.2 통합 검증
- [ ] "비그루비, 컵 가져와" → 컵 잡기 동작
- [ ] 동작 중 "비그루비, 멈춰" → 즉시 정지
- [ ] 동작 중 사람 손 감지 → 자동 후퇴
- [ ] DualSense 로 수동 jog → 음성 명령으로 전환 → 모션 정상 인계

### 6.3 회귀 검증
- [ ] 기존 record/smooth/play 정상 동작 (vision/voice 비활성 상태)
- [ ] DualSense jog 단독 동작 (vision/voice off)

---

## 7. 주의 / 위험

| 위험 | 대책 |
|---|---|
| DSR 펌웨어가 `amovej` 미지원 | 사전 확인. 미지원 시 `movej` + 별 thread 로 비동기화 |
| openWakeWord 한국어 정확도 | 자체 데이터 ≥100회 수집 + negative ≥1시간 |
| RealSense depth 노이즈 | median filter + 다중 frame 평균 |
| async + ROS 2 + DSR 동시 사용의 thread 안전성 | MultiThreadedExecutor + asyncio.Lock |
| LLM 호출 지연 (~1초) | 키워드 매칭으로 1차 분류, 모호 시만 LLM |
| 음성 도청 우려 | wake 전에는 마이크 buffer 즉시 폐기, 로컬 처리 |
| 사람 손 미검출로 충돌 | force-torque 센서 추가 또는 속도 제한 |

---

## 8. 향후 확장

- multi-modal: 음성 + 제스처 (손 모양) 동시 명령
- 여러 사물 시퀀스 명령 ("컵 들고 컵홀더에 넣어")
- 학습 모드: 사용자가 새 동작을 음성으로 가르침 ("이걸 따르기라고 부를게")
- 음성 응답 (TTS 출력) — VITS / Coqui TTS
