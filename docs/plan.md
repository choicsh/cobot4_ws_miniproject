# 미니프로젝트 진행 계획

시스템 구조(토픽, 자료형, 상태 머신, 데이터 흐름)는 [architecture.md](architecture.md) 참고.

## 저장소 구조

```
cobot4_ws_miniproject/
├── docs/plan.md
├── config/nav2.yaml      turtlebot4 nav2.yaml 로컬 수정본(DWB) 복사 + xy_goal_tolerance 0.1 (nav2 실행 시 params_file로 지정)
├── maps/                 my_map.pgm, my_map.yaml (실행 시에는 ~/maps의 파일을 읽음)
└── src/mini_project/     ROS 패키지 (mission.py, webcam_calib.py, align_check.py, 모델 .pt)
```

- `~/turtlebot4_ws/src/mini_project`는 `src/mini_project`를 가리키는 **심볼릭 링크**다. 빌드는 기존처럼 `~/turtlebot4_ws`에서 `colcon build --packages-select mini_project`로 한다.
- 처음 받는 PC에서 할 일:
  1. `ln -s <repo>/src/mini_project ~/turtlebot4_ws/src/mini_project`
  2. `cp <repo>/maps/my_map.* ~/maps/`
  3. webcam 캘리브레이션을 해서 `~/maps/webcam_H.npy`를 만든다. 이 파일은 PC와 webcam 위치에 따라 달라서 저장소에 넣지 않는다.

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
WAIT_CAR  -> webcam 프레임마다 YOLO, 'car' conf>=0.5가 최근 10프레임 중 7개 이상이면 다음 단계
UNDOCK    -> navigator.undock() 후 UNDOCKED_POSE로 초기 위치 설정 (도크에서는 라이다 꺼짐)
LOCALIZE  -> 새 amcl_pose 수신 후 waitUntilNav2Active()
NAVIGATE  -> 지도에서 차가 보이는 지점(visible_goal, 없으면 직선 위 APPROACH_DIST)을 goToPose, NAVIGATING에서 완료 대기
TRACK     -> 로봇 카메라 YOLO bbox + depth → 카메라 TF로 차 map 좌표 → goToPose(차 앞 TRACK_DIST)
FIND      -> 마지막으로 본 방향으로 제자리 회전, T초 안에 다시 보이면 TRACK
```

- `TurtleBot4Navigator(namespace='/robot5')` 노드 하나에 카메라 subscription도 붙인다. navigator의 blocking 호출과 executor가 충돌하지 않게 하려는 것.
- TRACK 제어 (Nav2 goToPose):
  - depth는 bbox 중앙 영역 depth의 median (0과 `MAX_DEPTH_MM` 4m 이상(먼 벽/배경)은 제외). 바닥 평면 필터는 넣지 않음: ROI가 bbox 중앙 1/3이라 차가 45°로 서 있어도 바닥이 거의 들어오지 않음
  - bbox 중심 (u, v)와 depth z를 `oakd/stereo/camera_info`의 K로 역투영: `X=(u-cx)z/fx, Y=(v-cy)z/fy, Z=z` (camera optical frame, frame_id는 depth 메시지 header)
  - `tf_buffer.lookup_transform('map', frame, rgb stamp)` 한 번으로 차 map 좌표(`do_transform_point`)와 카메라 map 위치(translation)를 같이 얻는다
  - `approach_goal(카메라 위치, 차 위치, TRACK_DIST=1.0m)`로 goal을 만들어 `goToPose`. 차가 직전 goal 기준에서 `REGOAL_DIST`(0.1m) 이상 움직였을 때만 다시 보낸다 (새 goal이 이전 goal을 대체)
  - Nav2 `xy_goal_tolerance`를 0.25 → 0.1로 낮춘 `config/nav2.yaml`을 쓴다. 0.25면 10cm 옮긴 goal이 바로 도착 처리되어 로봇이 움직이지 않는다
  - TF 시각은 bbox를 만든 **rgb가 찍힌 시각**. 최신 TF(`Time()`)를 썼을 때 회전 중 영상 지연(0.2~0.4s)만큼 차 좌표가 좌우로 ±40cm 튀어, 가짜 이동 → goal 재전송 → 회전이 반복됐다 (실측). 과거 시각이라 TF가 버퍼에 있어 `spin_once` 루프에서도 대기 없이 조회된다. 없으면(extrapolation) 그 프레임은 건너뜀
  - TransformListener는 절대 토픽 `/tf`를 구독하므로 `main()`의 `rclpy.init`에서 `/tf:=/robot5/tf`, `/tf_static:=/robot5/tf_static`으로 remap
  - rgb와 depth 모두 **704x704**로, **rgb가 depth(stereo) 기준으로** OAK-D 내부에서 align되어 있다 (`align_check.py`, camera_info로 확인함). 그래서 K는 stereo의 것을 쓴다. bbox 픽셀 좌표를 depth에 그대로 쓴다. 크기가 다르면 에러를 내고 해당 프레임은 건너뛴다.
  - cmd_vel은 `geometry_msgs/TwistStamped`, 토픽은 `/robot5/cmd_vel` (FIND 제자리 회전에서만 사용)
  - 입력 토픽은 항상 압축된 것을 쓴다 (`qos_profile_sensor_data`):
    - rgb: `/robot5/oakd/rgb/image_raw/compressed` (`CompressedImage`, `yolo_detection.py` 기본값과 같음)
    - depth: `/robot5/oakd/stereo/image_raw/compressedDepth` (`CompressedImage`). 디코드는 `depth_floor_ransac.py`의 방식 그대로: 12바이트 헤더를 버리고 PNG를 디코드하면 16UC1 mm 값
    - rgb와 depth의 짝은 가장 최근에 받은 depth를 쓴다. `# ponytail: stamp 동기화 없음. 빠르게 움직일 때 어긋나면 message_filters.ApproximateTimeSynchronizer 사용`
- 감지 판정은 슬라이딩 윈도우(K-of-N): webcam 7/10, 로봇 카메라(NAVIGATING/FIND → TRACK) 5/10. 상태가 바뀌면 윈도우를 비운다.
- TRACK에서 depth 무효(stamp 차이 > `MAX_DT`, 유효 픽셀 없음), camera_info 없음, TF 실패인 프레임은 건너뛰고 진행 중인 goal을 유지한다.
- FIND에 들어갈 때 `navigator.cancelTask()`를 호출한다. TRACK의 Nav2 goal과 FIND의 cmd_vel 회전이 겹치지 않게 하기 위해서다.
- 조정 값(`APPROACH_DIST`, `TRACK_DIST`, `REGOAL_DIST`, conf, N, T)은 파일 상단 상수 또는 ros2 파라미터로 둔다. 실제 로봇에서 튜닝해야 한다.

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
- `cv2.perspectiveTransform`으로 map (x, y)를 구한다. 최근 10프레임 윈도우에서 감지된 좌표들의 median을 쓴다 (튀는 값 제거).
- 차 위치를 그대로 GOAL로 쓰지 않는다 (차가 장애물로 잡혀 Nav2가 실패한다). 로봇 현재 위치(도크) → 차 방향으로, 차 앞 `APPROACH_DIST`(0.5m) 지점을 GOAL로 하고, yaw는 차를 바라보게 한다. 가는 도중 로봇 카메라에 차가 들어오면 TRACK으로 넘어간다.
  - 1.4m였을 때 goal이 차와 벽을 사이에 둔 쪽에 찍혀, 차가 안 보인 채 goal 완료 → FIND → WAIT_CAR가 반복됐다 (실측). 0.1m는 webcam 오차·로봇 반경보다 작아 차에 닿을 수 있어 0.5m로 정했다.
  - **visible_goal (우선 사용):** `/robot5/map`(OccupancyGrid)에서 차 주위 반경 `VIS_RADII`(1.0 → 0.8 → 0.6 → 0.4m, 먼 것부터, 후보가 나온 첫 반경에서 멈춤) 원 위 후보(`VIS_STEP_DEG` 15° 간격) 중
    (1) 주변 `VIS_CLEARANCE`(0.3m) 안에 벽/unknown이 없고 (2) 후보→차 선분에 벽(≥50)이 없는 점 가운데 로봇과 가장 가까운 점을 goal로, yaw는 차를 바라봄.
    1.0m에서 찾으면 도착하자마자 TRACK 거리에서 차를 본다. 지도가 없거나 모든 반경에서 후보가 없으면 위의 직선 방식(`APPROACH_DIST` 0.5m). 로그: `goal [visible 0.8m]` / `goal [straight 0.5m]`
  - `# ponytail:` 후보는 직선 거리로 선택. 벽 반대편 후보가 경로상 더 멀면 `nav.getPath()` 경로 길이로 비교
- `pixel_to_map(H, u, v)`와 `approach_goal(robot_xy, car_xy, dist)`는 `--selftest` assert로 검증한다.

`# ponytail: 렌즈 왜곡 무시. 광각 webcam이라 가장자리 오차가 크면 cv2.undistortPoints를 먼저 적용`

## 도크에서 시작하기 (라이다 꺼짐 대응)

**문제:** 도크에 있는 동안 라이다가 꺼져 있다 (TurtleBot4 power saver). 그래서 amcl이 위치를 잡지 못하고 `amcl_pose`도 나오지 않는다. 지금 `main()`은 `amcl_pose`를 먼저 기다린 뒤에 WAIT_CAR를 시작하므로 영원히 대기한다.

**해결:** amcl이 필요 없는 단계를 앞으로 옮기고, 위치 추정은 undock 뒤로 미룬다.

```
[도크, 라이다 꺼짐]  WAIT_CAR (webcam + H만 사용, amcl 불필요)
                       │ 차 map 좌표 확정
                     UNDOCK -> 라이다 켜짐
[라이다 켜짐]        LOCALIZE -> 초기 위치 설정 -> 새 amcl_pose 수신 -> waitUntilNav2Active()
                     NAVIGATE -> ... (이후 동일)
```

- `main()`에서 `amcl_pose` 대기와 `waitUntilNav2Active()`를 빼고 LOCALIZE 상태로 옮긴다.
- **LOCALIZE의 초기 위치 설정** (`UNDOCKED_POSE` 상수로 선택):
  - `UNDOCKED_POSE = (0.0, 0.0, 180.0)`: my_map은 undock한 위치에서 SLAM을 시작해 만들었다. 그래서 위치는 map 원점 (0, 0)이고, 방향은 실측으로 180°다. undock이 끝나면 `nav.setInitialPose()`로 자동 설정한다. Create3의 undock은 매번 같은 동작이라 끝나는 위치가 거의 같다. 몇 cm 오차는 amcl이 스캔으로 보정한다. **(기본값, 사람 개입 없음)** map에서 (0, 0) 셀이 빈 공간이고 오른쪽 벽 근처인 것은 확인함.
  - `UNDOCKED_POSE = None`: rviz에서 2D Pose Estimate를 손으로 할 때까지 기다린다 (측정 전 임시 방법).
- 도크 위에서 초기 위치를 주지 않는다. amcl은 첫 스캔이 들어오기 전의 odom 이동을 반영하지 않는다. 그래서 undock 중에 생긴 후진과 회전이 그대로 오차로 남는다.
- **오래된 amcl_pose 무시:** amcl_pose는 TRANSIENT_LOCAL이라 이전 실행의 값이 남아 있을 수 있다. LOCALIZE에서는 undock이 끝난 시각 이후의 stamp만 받아들인다.
- **FIND → WAIT_CAR로 돌아온 경우:** 이미 도크를 떠나 위치를 잡은 상태다. UNDOCK과 LOCALIZE는 건너뛰고 바로 NAVIGATE로 간다.

**UNDOCKED_POSE 확인 (1회, 확인용):** 도크 위치가 SLAM 때와 같은지 본다.
1. localization, nav2, rviz를 띄우고 `ros2 action send_goal /robot5/undock irobot_create_msgs/action/Undock {}`로 undock한다.
2. rviz에서 2D Pose Estimate를 하고, 스캔이 지도와 겹치는지 확인한다.
3. `ros2 topic echo /robot5/amcl_pose --once`의 x, y와 yaw(= 2*atan2(z, w))가 (0, 0, 180°)에서 10cm, 10° 안쪽인지 본다. 벗어나면 그 값을 `UNDOCKED_POSE`에 넣는다. SLAM 이후 도크를 옮긴 경우가 여기에 해당한다.

## 진행 단계 (각 단계에 완료 확인 방법 포함)

| # | 작업 | 완료 확인 | 상태 |
|---|---|---|---|
| 0 | 사전 확인. cmd_vel: **TwistStamped로 확인됨** (Create3 `motion_control_node.cpp:114`, nav2.yaml `enable_stamped_cmd_vel: true`, teleop `stamped:=true`). 해상도: rgb·depth 모두 704x704, **align 확인 완료** (`ros2 run mini_project align_check`) | 완료 | 완료 |
| 1 | localization + nav2 실행 (아래 "실행 방법" 1~2) 후 `nav_to_pose.py`의 좌표를 수정해 실행. undock → 임의 지점 이동 | 로봇이 지점에 도착 | 실기 확인 필요 |
| 1.5 | `ros2 run mini_project webcam_calib`로 H 생성 → `~/maps/webcam_H.npy` | 재투영 오차 10cm 이하. 다른 위치에 로봇을 세웠을 때 변환 좌표와 amcl_pose 차이가 15cm 이하 (H가 있으면 클릭할 때 오차가 출력됨) | H 생성 완료 (2026-10-06). 검증점 15cm 확인 필요 |
| 2 | `mission.py`: WAIT_CAR → UNDOCK → NAVIGATE(ING) | 차를 바닥에 놓으면 로봇이 차 앞으로 가서 차를 바라봄 | 코드 완료, 실기 확인 필요 |
| 3 | `mission.py` TRACK 좌표: `stereo/camera_info`가 704x704인지, depth `frame_id` 확인 (시작 로그 `camera_info WxH, frame ...`) 후 TRACK 로그의 `car map (x, y)`가 rviz에서 실제 차 위치와 맞는지 | 오차 15cm 이하 | 코드 완료, 실기 확인 필요 |
| 4 | `mission.py` TRACK goToPose (`TRACK_DIST` = 1.0m, `REGOAL_DIST` = 0.1m, nav2 `xy_goal_tolerance` = 0.1m) | 차를 10cm 이상 옮기면 로봇이 차 앞 1.0m로 다시 가서 차를 바라봄. goal 근처에서 왔다 갔다 하지 않음 | 코드 완료, 실기 확인 필요 |
| 5 | `mission.py` FIND: 마지막으로 본 방향으로 회전. 한 바퀴(`FIND_SEC`) 돌아도 없으면 WAIT_CAR로 돌아가 webcam으로 위치를 다시 잡음 | 차를 가리면 회전하고, 다시 보이면 TRACK | 코드 완료, 실기 확인 필요 |
| 6 | 통합 테스트 + 파라미터 튜닝, 시연 bag 녹화 (`record_bag.py`) | 처음부터 끝까지 3회 연속 성공 | |

체크: `ros2 run mini_project mission --selftest` (approach_goal, pixel_to_cam), `ros2 run mini_project webcam_calib --selftest` (homography)

### 실행 방법

```bash
# 1. localization (로봇은 도크에 둔다. 2D Pose Estimate는 필요 없음: mission이 undock 후 자동 설정)
ROS_SUPER_CLIENT=False ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot5 map:=$HOME/maps/my_map.yaml
# 2. nav2
ROS_SUPER_CLIENT=False ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot5 params_file:=$HOME/cobot4_ws_miniproject/config/nav2.yaml
# 3. (webcam을 옮겼을 때만) 캘리브레이션: 로봇을 teleop으로 기준점마다 이동 → 화면에서 로봇 바닥 중심 클릭 → s
ros2 run mini_project webcam_calib
# 4. 미션
ROS_SUPER_CLIENT=False ros2 run mini_project mission
```

- `ROS_SUPER_CLIENT=False`: `/etc/turtlebot4_discovery/setup.bash`는 터미널에서 실행한 모든 노드를 super client로 만든다. 그러면 nav2 노드 11개가 각자 discovery 정보 전체를 WiFi로 받느라 서비스와 bond 연결이 수십 초씩 늦어지고, 결국 bringup이 중단된다. 실행용 노드는 False로 띄우고, `ros2 node list` 같은 확인용 CLI만 기본값(True)으로 쓴다.
- nav2는 도크에 있는 동안(초기 위치 없음, 라이다 꺼짐) planner_server 활성화 단계에서 `map` TF를 기다린다. 그래서 rviz에 navigation이 inactive로 나오는 것이 정상이다. mission이 undock 후 초기 위치를 주면 그때 active가 된다.

- 캘리브레이션 중 로봇을 손으로 들어 옮기지 않는다. amcl은 주행으로만 위치를 갱신하므로 손으로 옮기면 amcl_pose가 틀어진다.
- `mission.py`의 조정 값은 모두 파일 상단 상수에 있다.
- mission 종료는 Ctrl+C. 정지 명령(cmd_vel 0)과 진행 중 Nav2 goal 취소 후 종료한다 (`SignalHandlerOptions.NO`로 rclpy가 context를 먼저 닫지 않게 함).
- NAVIGATING 중에 로봇 카메라에 차가 보이면 바로 TRACK으로 넘어간다. TRACK의 첫 goal이 webcam 기준 goal을 대체한다.

## 역할 분담 (예시)

- A: 0, 1, 1.5단계. Nav2, map, webcam 캘리브레이션
- B: 2단계. webcam 감지와 상태 머신 골격
- C: 3~5단계. tracking과 find

2단계 이후 B와 C는 `mission.py` 한 파일을 함께 수정하므로, 상태별 함수(`do_track()`, `do_find()`)를 나눠 맡으면 충돌이 줄어든다.

## 일부러 뺀 것 (필요해지면 추가)

- **detection 전용 노드와 토픽 분리**: 다른 PC에서 추론해야 할 때 분리한다.
- **복귀 및 재도킹**: 다이어그램에 없다. 필요하면 `navigator.dock()` 한 줄로 된다.
- **find 단계의 Nav2 기반 탐색** (마지막 위치로 이동, explore_lite): 제자리 회전으로 부족할 때 추가한다.
