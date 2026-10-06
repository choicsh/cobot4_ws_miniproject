# 미니프로젝트 진행 계획

## 목표 흐름

```
webcam car detection ──(실패)──┐
        │ 성공                 └─> 반복
      undock
        │
     navigate (webcam 픽셀 → map 좌표로 변환한 차량 근처)
        │
     tracking <────────────(성공)──┐
        │                          │
  turtle car detection ──(실패)──> find ──> detection 재시도
```

## 이미 있는 것 (turtlebot4_ws)

| 단계 | 재사용할 코드 | 상태 |
|---|---|---|
| webcam car detection | `yolo_learning/yolov8_obj_det_wc.py` (`VideoCapture(2)`) + `src/mini_project/mini_project/yolo8n_best.pt` (`car`, `dummy`) | 동작함. ROS 노드는 아님 |
| webcam 픽셀 → map 좌표 | 없음 | **새로 작성** (아래 "webcam 위치 매핑") |
| undock / navigate | `src/turtlebot4/turtlebot4_navigation/.../turtlebot4_navigator.py` (`undock()`, `startToPose()`, `getDockedStatus()`), 예제 `turtlebot4_python_tutorials/nav_to_pose.py` | 그대로 사용 |
| map | `~/maps/my_map.yaml` (이것으로 확정) | 있음 |
| turtle car detection | `src/mini_project/mini_project/yolo_detection.py` + `yolo8n_merged_dataset_best.pt` (`/robot5/oakd/rgb/image_raw/compressed`) | 동작함. bbox를 로그로만 출력 |
| 거리 측정 | `depth_floor_ransac.py` (`/robot5/oakd/stereo/image_raw/compressedDepth` 디코드) | 디코드 코드 재사용 |
| tracking / find | 없음 | **새로 작성** |

## 새로 만들 것: `mini_project/mission.py` 노드 하나

상태 머신 하나로 전체 흐름을 처리한다. 노드를 나누거나 커스텀 msg를 만들지 않는다.

```
WAIT_CAR  -> webcam 프레임마다 YOLO, 'car' conf>=0.5가 N프레임 연속이면 다음 단계
UNDOCK    -> navigator.undock()
NAVIGATE  -> 차량 map 좌표에서 접근 GOAL을 계산해 navigator.startToPose(GOAL)
TRACK     -> 로봇 카메라 YOLO bbox + depth로 cmd_vel P 제어
FIND      -> 마지막으로 본 방향으로 제자리 회전, T초 안에 다시 보이면 TRACK
```

- `TurtleBot4Navigator(namespace='/robot5')` 노드 하나에 카메라 subscription도 붙인다. navigator의 blocking 호출과 executor가 충돌하지 않게 하려는 것.
- TRACK 제어:
  - `angular.z = -Kp_ang * (cx - W/2) / (W/2)`
  - `linear.x = Kp_lin * (depth - TARGET_DIST)`, 0 ~ `MAX_LIN`으로 clamp
  - depth는 bbox 중앙 영역 depth의 median (0과 범위 밖 값은 제외)
  - rgb와 depth 모두 **704x704**로 OAK-D 내부에서 align되어 있다 (`align_check.py`로 확인함). bbox 픽셀 좌표를 depth에 그대로 쓴다. 크기가 다르면 에러를 내고 해당 프레임은 건너뛴다.
  - cmd_vel은 `geometry_msgs/TwistStamped`, 토픽은 `/robot5/cmd_vel`
  - 입력 토픽은 항상 압축된 것을 쓴다 (`qos_profile_sensor_data`):
    - rgb: `/robot5/oakd/rgb/image_raw/compressed` (`CompressedImage`, `yolo_detection.py` 기본값과 같음)
    - depth: `/robot5/oakd/stereo/image_raw/compressedDepth` (`CompressedImage`). 디코드는 `depth_floor_ransac.py`의 방식 그대로: 12바이트 헤더를 버리고 PNG를 디코드하면 16UC1 mm 값
    - rgb와 depth의 짝은 가장 최근에 받은 depth를 쓴다. `# ponytail: stamp 동기화 없음. 빠르게 움직일 때 어긋나면 message_filters.ApproximateTimeSynchronizer 사용`
- TRACK에 들어가기 전에 `navigator.cancelTask()`를 호출한다. Nav2와 cmd_vel이 겹치지 않게 하기 위해서다.
- 조정 값(`Kp`, `TARGET_DIST`, `APPROACH_DIST`, conf, N, T)은 파일 상단 상수 또는 ros2 파라미터로 둔다. 실제 로봇에서 튜닝해야 한다.

## webcam 위치 매핑 (webcam 픽셀 → map 좌표)

webcam은 맵 바깥 회색(unknown) 영역에 고정되어 맵 안쪽 바닥을 비스듬히 내려다본다. 차는 항상 바닥 위에 있으므로, 바닥 평면 하나에 대한 **homography(3x3 행렬 H)** 하나면 픽셀을 map (x, y)로 바꿀 수 있다. 카메라 내부 파라미터나 TF가 필요 없다.

**1) 캘리브레이션 (webcam을 옮길 때마다 1회): `mini_project/webcam_calib.py`**
- 바닥 위 기준점 **4개 이상**(6~8개 권장)을 webcam 화면 전체에 고르게 퍼지도록 정한다. 몰려 있으면 가장자리 오차가 커진다.
- 각 점에 대해 (webcam 픽셀, map 좌표) 쌍을 모은다:
  - 픽셀: webcam 화면을 클릭
  - map 좌표: 그 점에 로봇을 세우고 `/robot5/amcl_pose`의 (x, y)를 읽는다. 로봇 위치를 화면에서 클릭하면 픽셀과 map 좌표가 한 번에 짝지어진다.
  - (대안) map에도 보이는 벽 모서리나 박스 모서리를 기준점으로 쓰고, rviz의 Publish Point(`/clicked_point`)로 map 좌표를 얻는다
- `cv2.findHomography(px, map_xy, cv2.RANSAC)`로 H를 구해 `webcam_H.npy`에 저장하고, 점별 재투영 오차(m)를 출력한다. 목표는 10cm 이하.

**2) 실행 시 (`mission.py` WAIT_CAR → NAVIGATE)**
- 차의 픽셀 위치는 **bbox 하단 중앙** `((x1+x2)/2, y2)`. 바닥에 닿는 점이라 homography 가정에 맞는다. bbox 중심을 쓰면 차 높이만큼 멀리 찍힌다.
- `cv2.perspectiveTransform`으로 map (x, y)를 구한다. N프레임 동안 모은 좌표의 median을 쓴다 (튀는 값 제거).
- 차 위치를 그대로 GOAL로 쓰지 않는다 (차가 장애물로 잡혀 Nav2가 실패한다). 로봇 현재 위치(도크) → 차 방향으로, 차 앞 `APPROACH_DIST`(약 0.7m, costmap inflation보다 크게) 지점을 GOAL로 하고, yaw는 차를 바라보게 한다. 도착하면 로봇 카메라에 차가 들어와 TRACK으로 바로 넘어갈 수 있다.
- `pixel_to_map(H, u, v)`와 `approach_goal(robot_xy, car_xy, dist)`는 `--selftest` assert로 검증한다.

`# ponytail: 렌즈 왜곡 무시. 광각 webcam이라 가장자리 오차가 크면 cv2.undistortPoints를 먼저 적용`

## 진행 단계 (각 단계에 완료 확인 방법 포함)

| # | 작업 | 완료 확인 | 상태 |
|---|---|---|---|
| 0 | 사전 확인. cmd_vel: **TwistStamped로 확인됨** (Create3 `motion_control_node.cpp:114`, nav2.yaml `enable_stamped_cmd_vel: true`, teleop `stamped:=true`). 해상도: rgb·depth 모두 704x704, **align 확인 완료** (`ros2 run mini_project align_check`) | 완료 | 완료 |
| 1 | localization + nav2 실행 (아래 "실행 방법" 1~2) 후 `nav_to_pose.py`의 좌표를 수정해 실행. undock → 임의 지점 이동 | 로봇이 지점에 도착 | 실기 확인 필요 |
| 1.5 | `ros2 run mini_project webcam_calib`로 H 생성 → `~/maps/webcam_H.npy` | 재투영 오차 10cm 이하. 다른 위치에 로봇을 세웠을 때 변환 좌표와 amcl_pose 차이가 15cm 이하 (H가 있으면 클릭할 때 오차가 출력됨) | H 생성 완료 (2026-10-06). 검증점 15cm 확인 필요 |
| 2 | `mission.py`: WAIT_CAR → UNDOCK → NAVIGATE(ING) | 차를 바닥에 놓으면 로봇이 차 앞으로 가서 차를 바라봄 | 코드 완료, 실기 확인 필요 |
| 3 | `mission.py` TRACK 회전 (`KP_ANG`) | 차를 좌우로 옮기면 로봇이 따라 돎 | 코드 완료, 실기 확인 필요 |
| 4 | `mission.py` TRACK 전진 (`KP_LIN`, `TARGET_DIST`). 후진은 안 함 | 일정 거리 유지, 너무 가까우면 정지 | 코드 완료, 실기 확인 필요 |
| 5 | `mission.py` FIND: 마지막으로 본 방향으로 회전. 한 바퀴(`FIND_SEC`) 돌아도 없으면 WAIT_CAR로 돌아가 webcam으로 위치를 다시 잡음 | 차를 가리면 회전하고, 다시 보이면 TRACK | 코드 완료, 실기 확인 필요 |
| 6 | 통합 테스트 + 파라미터 튜닝, 시연 bag 녹화 (`record_bag.py`) | 처음부터 끝까지 3회 연속 성공 | |

체크: `ros2 run mini_project mission --selftest` (approach_goal, track_cmd), `ros2 run mini_project webcam_calib --selftest` (homography)

### 실행 방법

```bash
# 1. localization (rviz에서 2D Pose Estimate까지)
ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot5 map:=$HOME/maps/my_map.yaml
# 2. nav2
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot5
# 3. (webcam을 옮겼을 때만) 캘리브레이션: 로봇을 teleop으로 기준점마다 이동 → 화면에서 로봇 바닥 중심 클릭 → s
ros2 run mini_project webcam_calib
# 4. 미션
ros2 run mini_project mission
```

- 캘리브레이션 중 로봇을 손으로 들어 옮기지 않는다. amcl은 주행으로만 위치를 갱신하므로 손으로 옮기면 amcl_pose가 틀어진다.
- `mission.py`의 조정 값은 모두 파일 상단 상수에 있다.
- NAVIGATING 중에 로봇 카메라에 차가 보이면 Nav2를 취소하고 바로 TRACK으로 넘어간다.

## 역할 분담 (예시)

- A: 0, 1, 1.5단계. Nav2, map, webcam 캘리브레이션
- B: 2단계. webcam 감지와 상태 머신 골격
- C: 3~5단계. tracking과 find

2단계 이후 B와 C는 `mission.py` 한 파일을 함께 수정하므로, 상태별 함수(`do_track()`, `do_find()`)를 나눠 맡으면 충돌이 줄어든다.

## 일부러 뺀 것 (필요해지면 추가)

- **detection 전용 노드와 토픽 분리**: 다른 PC에서 추론해야 할 때 분리한다.
- **복귀 및 재도킹**: 다이어그램에 없다. 필요하면 `navigator.dock()` 한 줄로 된다.
- **find 단계의 Nav2 기반 탐색** (마지막 위치로 이동, explore_lite): 제자리 회전으로 부족할 때 추가한다.
