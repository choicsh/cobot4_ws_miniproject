# Nav2 플러그인 정리 (ROS 2 Jazzy)

이 문서는 `/opt/ros/jazzy`에 설치된 Nav2 패키지의 pluginlib XML과 BT 노드 라이브러리를 기준으로 작성했습니다.

- ★ 표시: `nav2_bringup/params/nav2_params.yaml`의 기본 설정에서 쓰는 플러그인
- 플러그인은 각 서버의 파라미터에서 `plugin: "<클래스 이름>"` 형식으로 지정합니다.

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
```

설치된 플러그인 목록은 다음 명령으로 직접 확인할 수 있습니다.

```bash
ros2 pkg list | grep -E "nav2|dwb|opennav"
```

---

## 목차

1. [Controller (로컬 플래너)](#1-controller-로컬-플래너--controller_server)
2. [Goal Checker / Progress Checker](#2-goal-checker--progress-checker--controller_server)
3. [Controller Critic](#3-controller-critic)
4. [Planner (글로벌 플래너)](#4-planner-글로벌-플래너--planner_server)
5. [Smoother](#5-smoother--smoother_server)
6. [Costmap Layer / Filter](#6-costmap-layer--filter--costmap_2d)
7. [Behavior (복구 동작)](#7-behavior-복구-동작--behavior_server)
8. [Navigator](#8-navigator--bt_navigator)
9. [Behavior Tree 노드](#9-behavior-tree-노드--bt_navigator)
10. [Waypoint Task Executor](#10-waypoint-task-executor--waypoint_follower)
11. [Docking](#11-docking--docking_server)
12. [Route Server](#12-route-server--route_server)
13. [AMCL Motion Model](#13-amcl-motion-model--amcl)
14. [RViz 플러그인](#14-rviz-플러그인)
15. [플러그인이 아닌 관련 노드](#15-플러그인이-아닌-관련-노드)

---

## 1. Controller (로컬 플래너) — `controller_server`

Base class: `nav2_core::Controller` / 파라미터: `controller_plugins`

| 플러그인 | 패키지 | 설명 |
|---|---|---|
| ★ `nav2_mppi_controller::MPPIController` | nav2_mppi_controller | 샘플링 기반 MPC(MPPI). Critic 조합으로 동작을 조정. 차동/전방향/아커만 지원 |
| `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` | nav2_regulated_pure_pursuit_controller | Pure Pursuit에 곡률·장애물 근접 시 감속을 추가. 튜닝이 쉬움 |
| `dwb_core::DWBLocalPlanner` | dwb_core | Dynamic Window 기반. 궤적 생성기와 Critic을 플러그인으로 구성 |
| `nav2_rotation_shim_controller::RotationShimController` | nav2_rotation_shim_controller | 경로 방향으로 먼저 제자리 회전한 뒤 내부 컨트롤러(`primary_controller`)에 넘김 |
| `nav2_graceful_controller::GracefulController` | nav2_graceful_controller | 부드러운 제어 법칙 기반. 도킹 등 정밀 접근에 적합 |

## 2. Goal Checker / Progress Checker — `controller_server`

| 플러그인 | Base class | 설명 |
|---|---|---|
| ★ `nav2_controller::SimpleGoalChecker` | `nav2_core::GoalChecker` | x, y, yaw가 허용 오차 안에 들어오면 도착 |
| `nav2_controller::StoppedGoalChecker` | `nav2_core::GoalChecker` | 위치 조건 + 선속도/각속도가 거의 0일 때 도착 |
| `nav2_controller::PositionGoalChecker` | `nav2_core::GoalChecker` | x, y만 검사하고 방향은 무시 |
| `dwb_plugins::SimpleGoalChecker` | `nav2_core::GoalChecker` | DWB 패키지 버전 (구버전 호환) |
| `dwb_plugins::StoppedGoalChecker` | `nav2_core::GoalChecker` | DWB 패키지 버전 (구버전 호환) |
| ★ `nav2_controller::SimpleProgressChecker` | `nav2_core::ProgressChecker` | 일정 시간 내 이동 거리가 임계값 이상인지 검사 |
| `nav2_controller::PoseProgressChecker` | `nav2_core::ProgressChecker` | 이동 거리 + 회전 각도까지 검사 (제자리 회전도 진행으로 인정) |

## 3. Controller Critic

### 3-1. MPPI Critic — Base class: `mppi::critics::CriticFunction`

| 플러그인 | 설명 |
|---|---|
| `mppi::critics::ConstraintCritic` | 속도·가속도 제한을 벗어나는 궤적에 페널티 |
| `mppi::critics::CostCritic` | 코스트맵 값으로 장애물 회피 (ObstaclesCritic 대체, 권장) |
| `mppi::critics::ObstaclesCritic` | 장애물까지 거리 기반 회피 |
| `mppi::critics::GoalCritic` | 목표 위치로 접근 |
| `mppi::critics::GoalAngleCritic` | 목표 지점 근처에서 목표 방향 맞추기 |
| `mppi::critics::PathAlignCritic` | 경로와 궤적 정렬 |
| `mppi::critics::PathFollowCritic` | 경로상 먼 지점을 향해 전진 |
| `mppi::critics::PathAngleCritic` | 경로 방향을 바라보도록 유도 |
| `mppi::critics::PreferForwardCritic` | 후진보다 전진을 선호 |
| `mppi::critics::TwirlingCritic` | 전방향 로봇의 불필요한 회전 억제 |
| `mppi::critics::VelocityDeadbandCritic` | 모터 데드밴드 구간 속도 억제 |

### 3-2. DWB Critic — Base class: `dwb_core::TrajectoryCritic`

| 플러그인 | 설명 |
|---|---|
| `dwb_critics::BaseObstacleCritic` | 로봇 중심점 기준 코스트맵 장애물 비용 |
| `dwb_critics::ObstacleFootprintCritic` | footprint 전체 기준 장애물 비용 |
| `dwb_critics::GoalDistCritic` | 목표까지 거리 |
| `dwb_critics::GoalAlignCritic` | 목표 방향 정렬 |
| `dwb_critics::PathDistCritic` | 경로까지 거리 |
| `dwb_critics::PathAlignCritic` | 경로 방향 정렬 |
| `dwb_critics::PreferForwardCritic` | 전진 선호 |
| `dwb_critics::RotateToGoalCritic` | 목표 근처에서 제자리 회전으로 방향 맞추기 |
| `dwb_critics::OscillationCritic` | 진동(왔다갔다) 억제 |
| `dwb_critics::TwirlingCritic` | 불필요한 회전 억제 |

### 3-3. DWB Trajectory Generator — Base class: `dwb_core::TrajectoryGenerator`

| 플러그인 | 설명 |
|---|---|
| `dwb_plugins::StandardTrajectoryGenerator` | 기본 궤적 샘플링 |
| `dwb_plugins::LimitedAccelGenerator` | 가속도 제한을 반영한 샘플링 |

## 4. Planner (글로벌 플래너) — `planner_server`

Base class: `nav2_core::GlobalPlanner` / 파라미터: `planner_plugins`

| 플러그인 | 패키지 | 설명 |
|---|---|---|
| ★ `nav2_navfn_planner::NavfnPlanner` | nav2_navfn_planner | Dijkstra / A* 기반. 원형 로봇, 간단한 환경에 적합 |
| `nav2_smac_planner::SmacPlanner2D` | nav2_smac_planner | 2D A* (8방향). NavFn 대체용 |
| `nav2_smac_planner::SmacPlannerHybrid` | nav2_smac_planner | Hybrid-A*. 최소 회전반경을 고려 (아커만/차동) |
| `nav2_smac_planner::SmacPlannerLattice` | nav2_smac_planner | State Lattice. 임의 형상·운동학 제약 로봇 |
| `nav2_theta_star_planner::ThetaStarPlanner` | nav2_theta_star_planner | Theta*. 격자에 얽매이지 않는 직선 위주 경로 |

## 5. Smoother — `smoother_server`

Base class: `nav2_core::Smoother` / 파라미터: `smoother_plugins`

| 플러그인 | 설명 |
|---|---|
| ★ `nav2_smoother::SimpleSmoother` | 단순 반복 스무딩 + 충돌 검사 |
| `nav2_smoother::SavitzkyGolaySmoother` | Savitzky-Golay 필터. 빠르고 노이즈 제거에 적합 |
| `nav2_constrained_smoother/ConstrainedSmoother` | Ceres 최적화로 부드러움과 장애물 거리 동시 개선 (무거움) |

## 6. Costmap Layer / Filter — `costmap_2d`

Base class: `nav2_costmap_2d::Layer` / 파라미터: `plugins`, `filters`

### 6-1. Layer

| 플러그인 | 설명 |
|---|---|
| ★ `nav2_costmap_2d::StaticLayer` | `map_server`의 OccupancyGrid를 그대로 반영 (global) |
| ★ `nav2_costmap_2d::ObstacleLayer` | LaserScan / PointCloud2로 장애물 표시·제거 (2D) |
| ★ `nav2_costmap_2d::VoxelLayer` | 3D 복셀 기반 장애물 레이어 (local, 3D 센서용) |
| ★ `nav2_costmap_2d::InflationLayer` | 장애물 주변으로 비용을 팽창시켜 안전 거리 확보 |
| `nav2_costmap_2d::RangeSensorLayer` | 초음파/IR 등 Range 센서 기반 장애물 |
| `nav2_costmap_2d::DenoiseLayer` | 센서 노이즈로 생긴 작은 고립 장애물 제거 |
| `nav2_costmap_2d::PluginContainerLayer` | 여러 레이어를 하나로 묶어 관리 |

### 6-2. Costmap Filter (마스크 맵 + `costmap_filter_info_server` 필요)

| 플러그인 | 설명 |
|---|---|
| `nav2_costmap_2d::KeepoutFilter` | 진입 금지 구역 / 선호 차선 |
| `nav2_costmap_2d::SpeedFilter` | 구역별 최대 속도 제한 |
| `nav2_costmap_2d::BinaryFilter` | 구역 진입 시 Bool 토픽 on/off (조명, 경고음 등 트리거) |

## 7. Behavior (복구 동작) — `behavior_server`

Base class: `nav2_core::Behavior` / 파라미터: `behavior_plugins`

| 플러그인 | 설명 |
|---|---|
| ★ `nav2_behaviors::Spin` | 제자리 회전 |
| ★ `nav2_behaviors::BackUp` | 후진 |
| ★ `nav2_behaviors::DriveOnHeading` | 지정 방향으로 일정 거리 직진 |
| ★ `nav2_behaviors::Wait` | 지정 시간 대기 |
| ★ `nav2_behaviors::AssistedTeleop` | 충돌 방지가 적용된 수동 조종 |

## 8. Navigator — `bt_navigator`

Base class: `nav2_core::NavigatorBase` / 파라미터: `navigators`

| 플러그인 | 액션 | 설명 |
|---|---|---|
| ★ `nav2_bt_navigator::NavigateToPoseNavigator` | `navigate_to_pose` | 단일 목표로 이동 |
| ★ `nav2_bt_navigator::NavigateThroughPosesNavigator` | `navigate_through_poses` | 여러 경유지를 거쳐 이동 |

## 9. Behavior Tree 노드 — `bt_navigator`

BT XML에서 쓰는 노드입니다. 라이브러리는 `/opt/ros/jazzy/lib/libnav2_*_bt_node.so`이며,
Jazzy에서는 기본 노드가 자동 로드되고 커스텀 노드만 `plugin_lib_names`에 추가하면 됩니다.
전체 포트 정의는 `/opt/ros/jazzy/share/nav2_behavior_tree/nav2_tree_nodes.xml`에 있습니다.

### 9-1. Action

| 분류 | 노드 |
|---|---|
| 경로 계획/추종 | `ComputePathToPose`, `ComputePathThroughPoses`, `FollowPath`, `SmoothPath` |
| 경로 가공 | `TruncatePath`, `TruncatePathLocal`, `RemovePassedGoals`, `GetPoseFromPath`, `ConcatenatePaths`, `GetCurrentPose` |
| 네비게이션 | `NavigateToPose`, `NavigateThroughPoses` |
| Route | `ComputeRoute`, `ComputeAndTrackRoute` |
| 복구 동작 | `Spin`, `BackUp`, `DriveOnHeading`, `Wait`, `AssistedTeleop` |
| 취소 | `CancelControl`, `CancelSpin`, `CancelBackUp`, `CancelDriveOnHeading`, `CancelWait`, `CancelAssistedTeleop`, `CancelComputeAndTrackRoute` |
| 코스트맵 | `ClearEntireCostmap`, `ClearCostmapAroundRobot`, `ClearCostmapAroundPose`, `ClearCostmapExceptRegion` |
| 플러그인 선택 | `PlannerSelector`, `ControllerSelector`, `SmootherSelector`, `GoalCheckerSelector`, `ProgressCheckerSelector` |
| 기타 | `ReinitializeGlobalLocalization` |
| 도킹 (opennav_docking_bt) | `DockRobot`, `UndockRobot` |

### 9-2. Condition

| 노드 | 설명 |
|---|---|
| `GoalReached` | 목표 도착 여부 |
| `GoalUpdated`, `GlobalUpdatedGoal` | 목표 변경 여부 |
| `IsPathValid` | 현재 경로가 장애물에 막혔는지 |
| `IsStuck` | 로봇이 멈춰 있는지 |
| `IsBatteryLow`, `IsBatteryCharging` | 배터리 상태 |
| `DistanceTraveled`, `TimeExpired`, `PathExpiringTimer` | 거리/시간 경과 |
| `TransformAvailable`, `InitialPoseReceived` | TF / 초기 위치 수신 여부 |
| `ArePosesNear` | 두 pose가 가까운지 |
| `AreErrorCodesPresent` | 지정한 에러 코드 발생 여부 |
| `WouldAControllerRecoveryHelp`, `WouldAPlannerRecoveryHelp`, `WouldASmootherRecoveryHelp` | 에러 코드 기준 복구 시도가 의미 있는지 |

### 9-3. Control / Decorator

| 종류 | 노드 | 설명 |
|---|---|---|
| Control | `PipelineSequence` | 앞 단계를 계속 재실행하면서 다음 단계 진행 (재계획 + 추종) |
| Control | `RecoveryNode` | 첫 자식 실패 시 두 번째 자식(복구)을 실행 후 재시도 |
| Control | `RoundRobin` | 실패할 때마다 다음 자식을 순서대로 시도 |
| Decorator | `RateController` | 지정 Hz로 자식 실행 (예: 1Hz 재계획) |
| Decorator | `DistanceController` | 일정 거리 이동마다 자식 실행 |
| Decorator | `SpeedController` | 속도에 비례한 주기로 자식 실행 |
| Decorator | `SingleTrigger` | 한 번만 실행 |
| Decorator | `GoalUpdater` | 외부 토픽으로 목표 갱신 |
| Decorator | `GoalUpdatedController` | 목표가 바뀌었을 때만 자식 실행 |
| Decorator | `PathLongerOnApproach` | 목표 근처에서 새 경로가 크게 길어지면 자식 실행 |

## 10. Waypoint Task Executor — `waypoint_follower`

Base class: `nav2_core::WaypointTaskExecutor` / 파라미터: `waypoint_task_executor_plugin`

| 플러그인 | 설명 |
|---|---|
| ★ `nav2_waypoint_follower::WaitAtWaypoint` | 웨이포인트 도착 후 지정 시간 대기 |
| `nav2_waypoint_follower::PhotoAtWaypoint` | 도착 시 카메라 이미지를 저장 |
| `nav2_waypoint_follower::InputAtWaypoint` | 외부 입력(토픽)이 올 때까지 대기 |

## 11. Docking — `docking_server`

Base class: `opennav_docking_core::ChargingDock` / 파라미터: `dock_plugins`

| 플러그인 | 설명 |
|---|---|
| ★ `opennav_docking::SimpleChargingDock` | 충전 도크. 배터리 상태 또는 접촉으로 충전 확인 |
| `opennav_docking::SimpleNonChargingDock` | 비충전 도크 (정위치 정차용) |
| `opennav_docking::TestFailureDock` | 단위 테스트용 실패 도크 |

## 12. Route Server — `route_server`

### 12-1. Edge Cost Function — Base class: `nav2_route::EdgeCostFunction`

| 플러그인 | 설명 |
|---|---|
| ★ `nav2_route::DistanceScorer` | 엣지 길이 (속도 제한 반영) |
| ★ `nav2_route::CostmapScorer` | 엣지 위 코스트맵 값 |
| `nav2_route::DynamicEdgesScorer` | 외부에서 엣지를 닫거나 비용을 동적으로 설정 |
| `nav2_route::PenaltyScorer` | 그래프 메타데이터의 페널티 값 |
| `nav2_route::SemanticScorer` | 의미 정보(semantic) 기반 비용 |
| `nav2_route::TimeScorer` | 과거 주행 시간 또는 추정 시간 |
| `nav2_route::GoalOrientationScorer` | 목표 방향과 경로 방향 일치 여부 |
| `nav2_route::StartPoseOrientationScorer` | 로봇 현재 방향과 시작 엣지 방향 일치 여부 |

### 12-2. Route Operation — Base class: `nav2_route::RouteOperation`

| 플러그인 | 설명 |
|---|---|
| ★ `nav2_route::ReroutingService` | 외부 서비스 요청 시 재계획 |
| ★ `nav2_route::AdjustSpeedLimit` | 그래프 메타데이터로 속도 제한 변경 |
| ★ `nav2_route::CollisionMonitor` | 경로 앞부분을 코스트맵으로 주기 검사 |
| `nav2_route::TriggerEvent` | 노드/엣지 이벤트에서 임의 서비스 호출 |
| `nav2_route::TimeMarker` | 실제 통과 시간을 엣지 메타데이터에 기록 |

### 12-3. Graph 파일 입출력

| 플러그인 | Base class | 설명 |
|---|---|---|
| `nav2_route::GeoJsonGraphFileLoader` | `nav2_route::GraphFileLoader` | GeoJSON 그래프 로드 |
| `nav2_route::GeoJsonGraphFileSaver` | `nav2_route::GraphFileSaver` | GeoJSON 그래프 저장 |

## 13. AMCL Motion Model — `amcl`

Base class: `nav2_amcl::MotionModel` / 파라미터: `robot_model_type`

| 플러그인 | 설명 |
|---|---|
| ★ `nav2_amcl::DifferentialMotionModel` | 차동 구동 로봇 (TurtleBot 등) |
| `nav2_amcl::OmniMotionModel` | 전방향(메카넘 등) 로봇 |

## 14. RViz 플러그인

패키지: `nav2_rviz_plugins`

| 플러그인 | 종류 | 설명 |
|---|---|---|
| `nav2_rviz_plugins/Navigation 2` | Panel | Nav2 시작/정지, 목표 전송, 상태 표시 |
| `nav2_rviz_plugins/Selector` | Panel | 실행 중 플래너/컨트롤러 등 플러그인 전환 |
| `nav2_rviz_plugins/Docking` | Panel | 도킹/언도킹 액션 실행 |
| `nav2_rviz_plugins/Route Tool` | Panel | Route 그래프 작성 |
| `nav2_rviz_plugins/GoalTool` | Tool | 맵에서 목표 pose 지정 (Nav2 Goal) |
| `nav2_rviz_plugins/CostmapCostTool` | Tool | 클릭한 지점의 코스트맵 값 확인 |
| `nav2_rviz_plugins/ParticleCloud` | Display | AMCL 파티클 표시 |

## 15. 플러그인이 아닌 관련 노드

플러그인 구조는 아니지만 Nav2 구성에 자주 함께 쓰이는 노드입니다.

| 노드 | 패키지 | 설명 |
|---|---|---|
| `collision_monitor` | nav2_collision_monitor | 센서 데이터로 정지/감속 구역(polygon) 감시, `cmd_vel` 필터링 |
| `velocity_smoother` | nav2_velocity_smoother | 속도·가속도 제한을 적용해 `cmd_vel` 부드럽게 |
| `map_server` / `map_saver` | nav2_map_server | 맵 로드/저장 |
| `lifecycle_manager` | nav2_lifecycle_manager | Nav2 노드 lifecycle 일괄 관리 |
| `nav2_simple_commander` | nav2_simple_commander | Python API (`BasicNavigator`)로 Nav2 제어 |

---

## 참고

- Nav2 공식 플러그인 목록: <https://docs.nav2.org/plugins/index.html>
- Nav2 설정 가이드: <https://docs.nav2.org/configuration/index.html>
- 기본 파라미터: `/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml`
