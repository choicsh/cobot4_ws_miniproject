import math
import os
import sys
import time
from collections import deque
from types import SimpleNamespace

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CameraInfo, CompressedImage
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from ultralytics import YOLO
import numpy as np
import cv2

from mini_project.align_check import depth_at
from mini_project.webcam_calib import AMCL_QOS, H_PATH, WEBCAM_INDEX, pixel_to_map

# ================================
# 설정 상수 (실제 로봇에서 튜닝)
# ================================
NAMESPACE = '/robot5'
MODEL_DIR = os.path.expanduser('~/turtlebot4_ws/src/mini_project/mini_project')  # .pt는 install에 복사 안 됨
WEBCAM_MODEL = os.path.join(MODEL_DIR, 'yolo8n_merged_dataset_best.pt')
ROBOT_MODEL = os.path.join(MODEL_DIR, 'yolo8n_merged_dataset_best.pt')
CAR_CLASS = 'car'
CONF = 0.8
WEBCAM_N, WEBCAM_K = 10, 7  # webcam: 최근 N프레임 중 K개 이상 car면 출발 (좌표는 감지된 것들의 median)
ROBOT_N, ROBOT_K = 10, 5    # 로봇 카메라: NAVIGATING/FIND에서 최근 N프레임 중 K개 이상이면 TRACK
APPROACH_DIST = 0.5       # NAVIGATE: webcam 좌표 기준 차 앞 몇 m를 GOAL로. 1.4m에서는 벽 너머라 차가 안 보인 채 도착 (실측).
#                           0.1m는 webcam 오차(10~15cm)·로봇 반경(0.19m)보다 작아 차에 닿을 수 있어 0.5m로
TRACK_DIST = 1.0          # TRACK: 차(가까운 표면) 앞 몇 m를 GOAL로. 카메라 0.8m 안쪽은 차가 화면 하단에 잘림 (실측)
VIS_RADII = (TRACK_DIST, 0.8, 0.6, 0.4)  # NAVIGATE: 차 주위 후보 원 반경, 먼 것부터 (m). 0.4 = 로봇 반경 + 차 반폭 + 여유
VIS_STEP_DEG = 15         # NAVIGATE: 후보 원 위 간격 (deg)
VIS_CLEARANCE = 0.3       # NAVIGATE: 후보 주변 이 반경(m) 안에 벽/unknown 있으면 제외 (로봇 반경 0.19 + 여유)
REGOAL_DIST = 0.1         # TRACK: 차 map 좌표가 직전 goal 기준보다 이만큼 움직이면 goal 다시 보냄 (m)
#                           Nav2 xy_goal_tolerance(config/nav2.yaml 0.1)보다 작으면 새 goal이 바로 도착 처리됨
LOST_SEC = 0.7            # 이 시간 동안 안 보이면 FIND
MAX_DEPTH_MM = 4000        # TRACK depth: 이 거리 이상(먼 벽/배경) 픽셀은 ROI median에서 제외 (mm)
MAX_DT = 0.1              # rgb와 depth stamp 차이가 이보다 크면 depth 안 씀 (s)
FIND_ANG = 0.3            # FIND 회전 속도 (rad/s)
FIND_SEC = 2 * math.pi / FIND_ANG + 1.0  # 한 바퀴 돌아도 없으면 webcam으로 다시 찾기
# undock 직후 map 자세 (x, y, yaw_deg). my_map은 undock 위치에서 SLAM을 시작해 위치는 원점, 방향은 실측 180°.
# None이면 undock 후 rviz 2D Pose Estimate를 기다림 (도크를 옮겼는데 다시 재지 않았을 때)
UNDOCKED_POSE = (0.0, 0.0, 180.0)  # 실측: 원점, 180° 회전
# ================================


def stamp_sec(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def hits(window):
    """슬라이딩 윈도우에서 감지된 항목(None 아닌 것)만."""
    return [x for x in window if x is not None]


def best_car(result, names):
    """YOLO 결과에서 conf가 가장 높은 car의 (x1,y1,x2,y2), 없으면 None."""
    cars = [b for b in result.boxes if names[int(b.cls[0])] == CAR_CLASS]
    if not cars:
        return None
    return max(cars, key=lambda b: float(b.conf[0])).xyxy[0].tolist()


def approach_goal(robot_xy, car_xy, dist=APPROACH_DIST):
    """로봇 -> 차 직선 위, 차 앞 dist 지점과 차를 바라보는 yaw [rad]."""
    dx, dy = car_xy[0] - robot_xy[0], car_xy[1] - robot_xy[1]
    d = math.hypot(dx, dy)
    k = max(d - dist, 0.0) / d if d > 1e-6 else 0.0
    return robot_xy[0] + dx * k, robot_xy[1] + dy * k, math.atan2(dy, dx)


def pixel_to_cam(u, v, z, K):
    """픽셀 (u,v) + depth z [m] -> camera optical frame (x 오른쪽, y 아래, z 앞) [m]."""
    return (u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z


def visible_goal(grid, res, origin, robot_xy, car_xy, radii=VIS_RADII,
                 step_deg=VIS_STEP_DEG, clearance=VIS_CLEARANCE):
    """정적 지도에서 차가 보이는 goal (x, y, yaw, r), 없으면 None.

    grid: OccupancyGrid.data (h, w) int8 (0 빈칸, 100 벽, -1 unknown), res [m/cell], origin (x, y) [m].
    radii를 먼 것부터 차례로, 차 주위 반경 r 원 위 후보 중 (1) 주변 clearance 안에 벽/unknown 없고
    (2) 후보->차 선분에 벽 없는 것 중 로봇과 가장 가까운 점. 후보가 나온 첫 반경에서 멈춤. yaw는 차를 바라봄.
    """
    # ponytail: 직선 거리로 선택. 벽 반대편 후보가 경로상 더 멀면 nav.getPath() 경로 길이로 비교
    h, w = grid.shape
    c = int(math.ceil(clearance / res))

    def cell(x, y):
        return int((y - origin[1]) / res), int((x - origin[0]) / res)  # (row, col)

    for r in radii:
        best = None
        for a in np.radians(np.arange(0, 360, step_deg)):
            x, y = car_xy[0] + r * math.cos(a), car_xy[1] + r * math.sin(a)
            j, i = cell(x, y)
            if not (c <= j < h - c and c <= i < w - c):
                continue
            if np.any(grid[j - c:j + c + 1, i - c:i + c + 1] != 0):  # 정사각형 창 (원보다 약간 보수적)
                continue
            n = int(r / res * 2) + 1  # 셀 크기의 절반 간격으로 선분 샘플
            rows, cols = zip(*(cell(x + (car_xy[0] - x) * t, y + (car_xy[1] - y) * t)
                               for t in np.linspace(0, 1, n)))
            if np.any(grid[np.clip(rows, 0, h - 1), np.clip(cols, 0, w - 1)] >= 50):
                continue
            d = math.dist((x, y), robot_xy)
            if best is None or d < best[0]:
                best = (d, x, y, math.atan2(car_xy[1] - y, car_xy[0] - x), r)
        if best is not None:
            return best[1:]
    return None


class Mission:
    def __init__(self):
        self.nav = TurtleBot4Navigator(namespace=NAMESPACE)
        n = self.nav
        # 네임스페이스 상대 토픽 -> /robot5/...
        self.cmd_pub = n.create_publisher(TwistStamped, 'cmd_vel', 10)
        n.create_subscription(CompressedImage, 'oakd/rgb/image_raw/compressed',
                              self.rgb_callback, qos_profile_sensor_data)
        n.create_subscription(CompressedImage, 'oakd/stereo/image_raw/compressedDepth',
                              self.depth_callback, qos_profile_sensor_data)
        # rgb가 depth(stereo) 기준으로 align(704x704)되어 있으므로 stereo의 K 사용 (depth frame_id와 일치)
        n.create_subscription(CameraInfo, 'oakd/stereo/camera_info', self.camera_info_callback,
                              qos_profile_sensor_data)
        n.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.amcl_callback, AMCL_QOS)
        n.create_subscription(OccupancyGrid, 'map', self.map_callback, AMCL_QOS)  # map도 RELIABLE + TRANSIENT_LOCAL

        self.H = np.load(H_PATH)
        self.webcam = cv2.VideoCapture(WEBCAM_INDEX)
        self.webcam.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # WAIT_CAR 재진입 시 오래된 프레임 방지
        self.webcam_model = YOLO(WEBCAM_MODEL)
        self.robot_model = YOLO(ROBOT_MODEL)
        self.tf_buffer = Buffer()
        # 전용 내부 노드 + 스레드로 TF 수신: 메인 루프(spin_once 1회 1콜백, YOLO 사이)에서 받으면 버퍼가 늦게 쌓여
        # rgb 시각 조회가 extrapolation으로 자주 실패함 (실측). /tf -> /robot5/tf remap은 main()의 전역 인자로 적용
        self.tf_listener = TransformListener(self.tf_buffer, None, spin_thread=True)

        self.rgb_msg = None
        self.depth_mm = None
        self.depth_stamp = 0.0
        self.depth_frame = ''
        self.K = None
        self.rgb_stamp = 0.0
        self.robot_xy = None
        self.amcl_fresh = False  # 마지막 undock 이후 amcl_pose를 받았는지
        self.localized = False
        self.pose_set = False  # mission이 초기 위치를 직접 설정했는지
        self.car_xy = None
        self.map = None  # (grid (h,w) int8, res, (origin x, y)) from map_server
        self.goal_car = None  # TRACK: 마지막 goal을 보낼 때의 차 map 좌표
        self.webcam_win = deque(maxlen=WEBCAM_N)  # 프레임마다 car map 좌표 또는 None
        self.robot_win = deque(maxlen=ROBOT_N)    # 프레임마다 car bbox 또는 None
        self.raw_dist = 0.0
        self.last_seen = 0.0
        self.last_dir = 1.0  # 마지막으로 본 방향 (+: 왼쪽 회전)
        self.find_start = 0.0
        self.state = 'WAIT_CAR'

    # ---------- 콜백: 최신 값만 저장 ----------
    def rgb_callback(self, msg):
        self.rgb_msg = msg

    def depth_callback(self, msg):
        # compressedDepth = 12바이트 헤더 + PNG (depth_floor_ransac.py와 동일)
        d = cv2.imdecode(np.frombuffer(msg.data, np.uint8)[12:], cv2.IMREAD_UNCHANGED)
        if d is not None:
            self.depth_mm = d
            self.depth_stamp = stamp_sec(msg)
            self.depth_frame = msg.header.frame_id

    def camera_info_callback(self, msg):
        if self.K is None:
            self.nav.info(f'camera_info {msg.width}x{msg.height}, frame {msg.header.frame_id}')
        self.K = np.array(msg.k).reshape(3, 3)

    def map_callback(self, msg):
        info = msg.info
        grid = np.array(msg.data, np.int8).reshape(info.height, info.width)
        self.map = (grid, info.resolution, (info.origin.position.x, info.origin.position.y))

    def amcl_callback(self, msg):
        p = msg.pose.pose.position
        self.robot_xy = (p.x, p.y)
        self.amcl_fresh = True

    # ---------- 공통 ----------
    def set_state(self, state):
        self.nav.info(f'{self.state} -> {state}')
        self.state = state
        # 이전 상태의 감지 기록이 다음 상태 판정에 섞이지 않도록
        self.webcam_win.clear()
        self.robot_win.clear()
        self.goal_car = None  # TRACK에 들어오면 첫 유효 좌표로 바로 goal

    def publish(self, lin, ang):
        msg = TwistStamped()
        msg.header.stamp = self.nav.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = lin
        msg.twist.angular.z = ang
        self.cmd_pub.publish(msg)

    def robot_frame(self):
        """새 rgb 프레임이 있으면 (car bbox 또는 None, (h, w)), 없으면 None."""
        msg, self.rgb_msg = self.rgb_msg, None
        if msg is None:
            return None
        self.rgb_stamp = stamp_sec(msg)
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        r = self.robot_model(img, conf=CONF, verbose=False)[0]
        cv2.imshow('robot', r.plot())
        cv2.waitKey(1)
        box = best_car(r, self.robot_model.names)
        self.robot_win.append(box)
        return box, img.shape[:2]

    def robot_seen(self):
        return len(hits(self.robot_win)) >= ROBOT_K

    def box_depth(self, box, shape):
        """bbox 중앙 영역 depth median [m], 모르면 0."""
        if self.depth_mm is None:
            return 0.0
        if self.depth_mm.shape != shape:  # align 기준 704x704 (align_check.py)
            self.nav.error(f'rgb {shape} != depth {self.depth_mm.shape}')
            return 0.0
        x1, y1, x2, y2 = box
        patch = max(2, int(min(x2 - x1, y2 - y1) / 6))
        raw = depth_at(self.depth_mm, int((x1 + x2) / 2), int((y1 + y2) / 2), patch, MAX_DEPTH_MM) / 1000.0
        self.raw_dist = raw  # 측정 로그용 (stamp 검사 전 값)
        dt = self.rgb_stamp - self.depth_stamp
        if abs(dt) > MAX_DT:  # 회전 중 어긋난 depth로 차 좌표를 잘못 잡지 않도록 (이 프레임은 건너뜀)
            self.nav.get_logger().warn(f'rgb-depth stamp 차이 {dt:+.3f}s > {MAX_DT}s, depth 무시',
                                       throttle_duration_sec=1.0)
            return 0.0
        return raw

    def car_in_map(self, box, shape):
        """bbox -> (카메라 map xy, 차 map xy), depth/K/TF 중 하나라도 없으면 None."""
        if self.K is None:
            self.nav.get_logger().warn('camera_info 대기 중', throttle_duration_sec=1.0)
            return None
        z = self.box_depth(box, shape)
        if z <= 0:
            return None
        try:
            # odom <- 카메라: bbox를 만든 rgb가 찍힌 시각. 최신 TF면 회전 중 영상 지연만큼 차 좌표가 좌우로 튐 (실측 ±40cm)
            # map <- odom: 최신 값. amcl이 scan을 버리며 수 초씩 발행을 멈춰도(실측 3.4s) extrapolation 실패하지 않도록.
            # 과거/최신 시각이라 버퍼에 있으므로 대기 없이 조회됨
            stamp = Time(nanoseconds=round(self.rgb_stamp * 1e9))
            tf = self.tf_buffer.lookup_transform_full('map', Time(), self.depth_frame, stamp, 'odom')
        except TransformException as e:
            self.nav.get_logger().warn(f'TF map <- {self.depth_frame!r} 실패: {e}', throttle_duration_sec=1.0)
            return None
        pt = PointStamped()
        pt.point.x, pt.point.y, pt.point.z = pixel_to_cam((box[0] + box[2]) / 2, (box[1] + box[3]) / 2, z, self.K)
        p = do_transform_point(pt, tf).point
        t = tf.transform.translation
        return (t.x, t.y), (p.x, p.y)

    def start_find(self):
        self.nav.cancelTask()  # TRACK의 Nav2 goal과 FIND의 cmd_vel이 겹치지 않도록 (끝난 goal이면 무시됨)
        self.find_start = time.monotonic()
        self.set_state('FIND')

    # ---------- 상태 ----------
    def step(self):
        getattr(self, 'do_' + self.state.lower())()

    def do_wait_car(self):
        ok, img = self.webcam.read()
        if not ok:
            return
        r = self.webcam_model(img, conf=CONF, verbose=False)[0]
        cv2.imshow('webcam', r.plot())
        cv2.waitKey(1)
        box = best_car(r, self.webcam_model.names)
        # bbox 하단 중앙 = 바닥 접점
        self.webcam_win.append(None if box is None else pixel_to_map(self.H, (box[0] + box[2]) / 2, box[3]))
        pts = hits(self.webcam_win)
        if len(pts) >= WEBCAM_K:
            self.car_xy = np.median(pts, axis=0)
            self.nav.info(f'webcam car at map ({self.car_xy[0]:.2f}, {self.car_xy[1]:.2f})')
            cv2.destroyWindow('webcam')  # 이후 갱신이 없어 멈춘 것처럼 보이므로 닫음
            # FIND에서 돌아온 경우 이미 위치를 잡았으니 바로 이동
            self.set_state('NAVIGATE' if self.localized else 'UNDOCK')

    def do_undock(self):
        # 도크에서는 라이다가 꺼져 있어 amcl이 위치를 못 잡음 -> 초기 위치는 undock 뒤에 설정.
        # 도크 위에서 주면 undock 중의 후진·회전이 amcl에 반영되지 않음
        if self.nav.getDockedStatus():
            self.nav.info('docked -> undock')
            self.nav.undock()
            self.amcl_fresh = False  # undock 전에 남아 있던(TRANSIENT_LOCAL) amcl_pose는 무효
            if UNDOCKED_POSE is not None:
                x, y, yaw = UNDOCKED_POSE
                self.nav.setInitialPose(self.nav.getPoseStamped([x, y], yaw))
                self.pose_set = True
        else:
            self.nav.warn('도킹 상태가 아님 (is_docked=false): undock과 초기 위치 설정을 건너뜀. '
                          '기존 amcl 위치가 없으면 rviz 2D Pose Estimate 필요')
        self.set_state('LOCALIZE')

    def do_localize(self):
        # amcl은 스캔을 처리해야 amcl_pose를 발행하므로, 새 amcl_pose = 라이다가 돌고 map->odom TF가 있음
        if not self.amcl_fresh:
            hint = '라이다 기동 중' if self.pose_set else 'rviz 2D Pose Estimate 필요'
            self.nav.get_logger().info(f'Waiting for amcl_pose ({hint})', throttle_duration_sec=2.0)
            rclpy.spin_once(self.nav, timeout_sec=1.0)
            return
        # amcl_pose를 받은 뒤에 호출해야 (0,0) 초기화를 하지 않음
        self.nav.waitUntilNav2Active()
        self.localized = True
        self.set_state('NAVIGATE')

    def do_navigate(self):
        g = None if self.map is None else visible_goal(*self.map, self.robot_xy, self.car_xy)
        if g is not None:
            x, y, yaw, r = g
            how = f'visible {r:.1f}m'
        else:  # 지도 없음 / 모든 반경에서 보이는 후보 없음 -> 로봇-차 직선 위 APPROACH_DIST
            (x, y, yaw), how = approach_goal(self.robot_xy, self.car_xy), f'straight {APPROACH_DIST:.1f}m'
        self.nav.info(f'goal [{how}] ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} deg)')
        self.nav.goToPose(self.nav.getPoseStamped([x, y], math.degrees(yaw)))
        self.set_state('NAVIGATING')

    def do_navigating(self):
        f = self.robot_frame()
        if f is not None and self.robot_seen():  # 가는 도중 로봇 카메라에 보이면 바로 추적 (goal은 TRACK이 덮어씀)
            self.nav.info('로봇 카메라 감지: map상 로봇-차 거리 {:.2f} m'.format(
                math.dist(self.robot_xy, self.car_xy)))
            self.last_seen = time.monotonic()
            self.set_state('TRACK')
        elif self.nav.isTaskComplete():  # 도착(또는 실패)했는데 안 보임
            self.start_find()

    def do_track(self):
        f = self.robot_frame()
        if f is None:
            return
        box, shape = f
        now = time.monotonic()
        if box is None:
            if now - self.last_seen > LOST_SEC:
                self.start_find()
            return  # 잠깐 놓친 건 진행 중인 goal 유지
        self.last_seen = now
        if self.K is not None:  # 차가 화면 왼쪽이면 FIND에서 왼쪽(+) 회전
            self.last_dir = 1.0 if (box[0] + box[2]) / 2 < self.K[0, 2] else -1.0
        r = self.car_in_map(box, shape)
        if r is None:
            return  # depth/TF 무효 프레임은 건너뜀 (기존 goal 유지)
        cam_xy, car_xy = r
        self.nav.get_logger().info(
            f'TRACK depth {self.raw_dist:.2f} m | car map ({car_xy[0]:.2f}, {car_xy[1]:.2f})',
            throttle_duration_sec=0.5)
        if self.goal_car is not None and math.dist(car_xy, self.goal_car) < REGOAL_DIST:
            return
        x, y, yaw = approach_goal(cam_xy, car_xy, TRACK_DIST)
        self.nav.info(f'TRACK goal ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} deg)')
        self.nav.goToPose(self.nav.getPoseStamped([x, y], math.degrees(yaw)))
        self.goal_car = car_xy

    def do_find(self):
        f = self.robot_frame()
        if f is None:
            return
        if self.robot_seen():
            self.publish(0.0, 0.0)  # TRACK 첫 goal 전까지 FIND 회전이 남지 않도록
            self.last_seen = time.monotonic()
            self.set_state('TRACK')
        elif time.monotonic() - self.find_start > FIND_SEC:
            self.publish(0.0, 0.0)
            self.nav.info('한 바퀴 돌아도 없음: webcam으로 다시 위치 확인')
            self.set_state('WAIT_CAR')
        else:
            self.publish(0.0, FIND_ANG * self.last_dir)


def selftest():
    x, y, yaw = approach_goal((0.0, 0.0), (2.0, 0.0), 0.5)
    assert abs(x - 1.5) < 1e-9 and abs(y) < 1e-9 and abs(yaw) < 1e-9
    x, y, yaw = approach_goal((1.0, 1.0), (1.0, 1.3), 0.5)  # 이미 가까우면 제자리에서 바라보기만
    assert (x, y) == (1.0, 1.0) and abs(yaw - math.pi / 2) < 1e-9
    # visible_goal: 3m x 3m 지도(0.05m), x=1.0m 세로 벽. 차(1.5,1.5), 로봇은 벽 왼쪽 (0.2,1.5)
    grid = np.zeros((60, 60), np.int8)
    grid[5:55, 20] = 100
    x, y, yaw, r = visible_goal(grid, 0.05, (0.0, 0.0), (0.2, 1.5), (1.5, 1.5), (1.0,), 15, 0.3)
    assert x - 1.0 > 0.3 and r == 1.0, (x, y)  # 벽 너머(왼쪽)·벽에 가까운 후보는 제외
    assert abs(math.dist((x, y), (1.5, 1.5)) - 1.0) < 1e-9
    assert abs(yaw - math.atan2(1.5 - y, 1.5 - x)) < 1e-9  # 차를 바라봄
    assert visible_goal(np.full((60, 60), 100, np.int8), 0.05, (0.0, 0.0), (0.2, 1.5), (1.5, 1.5)) is None
    # 차(2,2) 주위 반경 1.05m만 빈칸, 나머지 unknown -> 1.0/0.8m 원은 clearance 0.3 불가, 0.6m에서 찾음
    yy, xx = np.mgrid[0:80, 0:80] * 0.05 + 0.025
    grid = np.where(np.hypot(xx - 2, yy - 2) < 1.05, 0, -1).astype(np.int8)
    x, y, yaw, r = visible_goal(grid, 0.05, (0.0, 0.0), (2.0, 0.2), (2.0, 2.0), (1.0, 0.8, 0.6, 0.4), 15, 0.3)
    assert r == 0.6 and abs(math.dist((x, y), (2.0, 2.0)) - 0.6) < 1e-9, (x, y, r)
    K = np.array([[500.0, 0, 352], [0, 500.0, 352], [0, 0, 1]])
    assert pixel_to_cam(352, 352, 2.0, K) == (0.0, 0.0, 2.0)  # 주점 -> 광축 위
    x, y, z = pixel_to_cam(602, 102, 2.0, K)  # 오른쪽 위: x = 250*2/500 = 1, y = -250*2/500 = -1
    assert (x, y, z) == (1.0, -1.0, 2.0)
    # stamp 차이가 MAX_DT를 넘으면 depth 무시 (rclpy 없이 box_depth만 확인)
    m = Mission.__new__(Mission)
    m.depth_mm = np.full((704, 704), 1000, np.uint16)
    m.nav = SimpleNamespace(get_logger=lambda: SimpleNamespace(warn=lambda *a, **k: None))
    m.rgb_stamp, m.depth_stamp = 10.05, 10.0
    assert abs(m.box_depth((300, 300, 400, 400), (704, 704)) - 1.0) < 1e-9
    m.depth_mm[:, :360] = MAX_DEPTH_MM + 500  # ROI(열 334~366)의 절반 이상이 먼 벽 -> 제외되고 차(1m)만 남음
    assert abs(m.box_depth((300, 300, 400, 400), (704, 704)) - 1.0) < 1e-9
    m.rgb_stamp = 10.0 + MAX_DT + 0.1
    assert m.box_depth((300, 300, 400, 400), (704, 704)) == 0.0
    # car_in_map: map<-odom은 오래된 값(t=1)뿐이어도, odom<-카메라는 rgb 시각(10.5)으로 보간해서 조회
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import Buffer as TfBuffer

    def tfs(parent, child, t, x):
        msg = TransformStamped()
        msg.header.frame_id, msg.child_frame_id = parent, child
        msg.header.stamp = Time(seconds=t).to_msg()
        msg.transform.translation.x, msg.transform.rotation.w = x, 1.0
        return msg
    m.tf_buffer = TfBuffer()
    m.tf_buffer.set_transform(tfs('map', 'odom', 1.0, 1.0), 'test')
    m.tf_buffer.set_transform(tfs('odom', 'cam', 10.0, 0.0), 'test')
    m.tf_buffer.set_transform(tfs('odom', 'cam', 11.0, 2.0), 'test')  # 카메라가 1초에 2m 이동
    m.K, m.depth_frame = K, 'cam'
    m.depth_mm = np.full((704, 704), 1000, np.uint16)
    m.rgb_stamp = m.depth_stamp = 10.5
    cam_xy, car_xy = m.car_in_map((302, 302, 402, 402), (704, 704))  # 주점 -> 카메라 광축 1m 앞
    assert np.allclose(cam_xy, (2.0, 0.0)) and np.allclose(car_xy, (2.0, 0.0)), (cam_xy, car_xy)
    m.rgb_stamp = m.depth_stamp = 11.5  # odom 데이터보다 미래 -> 건너뜀
    assert m.car_in_map((302, 302, 402, 402), (704, 704)) is None
    # K-of-N: 중간에 놓친 프레임이 있어도 K개 이상이면 감지
    w = deque([1, None, 1, 1, None, 1, 1, None, None, None], maxlen=ROBOT_N)
    assert len(hits(w)) == ROBOT_K
    w.append(None)  # 가장 오래된 감지 하나가 밀려남
    assert len(hits(w)) == ROBOT_K - 1
    # 도크 시작: undock -> UNDOCKED_POSE로 초기 위치 -> 새 amcl_pose 전까지 LOCALIZE 대기
    calls = []
    m.nav = SimpleNamespace(
        getDockedStatus=lambda: True, undock=lambda: calls.append('undock'),
        getPoseStamped=lambda xy, yaw: (tuple(xy), yaw), setInitialPose=lambda p: calls.append(p),
        waitUntilNav2Active=lambda: calls.append('nav2'), info=lambda *a: None, warn=lambda *a: None)
    m.state, m.webcam_win, m.robot_win = 'UNDOCK', deque(), deque()
    m.amcl_fresh, m.localized, m.pose_set = True, False, False  # 도크 전 값이 남아 있는 상황
    m.do_undock()
    assert calls == ['undock', (UNDOCKED_POSE[:2], UNDOCKED_POSE[2])] and m.state == 'LOCALIZE'
    assert not m.amcl_fresh
    m.amcl_fresh = True  # 라이다가 돌고 amcl_pose 도착
    m.do_localize()
    assert calls[-1] == 'nav2' and m.localized and m.state == 'NAVIGATE'
    print('selftest ok')


def main():
    if '--selftest' in sys.argv:
        selftest()
        return
    # TransformListener는 절대 토픽 /tf, /tf_static을 구독 -> 로봇 네임스페이스로 remap
    # SignalHandlerOptions.NO: Ctrl+C에 rclpy가 context를 먼저 닫지 않게 함 -> finally의 정지/goal 취소가 실행됨
    rclpy.init(args=sys.argv + ['--ros-args', '-r', f'/tf:={NAMESPACE}/tf',
                                '-r', f'/tf_static:={NAMESPACE}/tf_static'],
               signal_handler_options=SignalHandlerOptions.NO)
    m = Mission()
    try:
        # 도크(라이다 꺼짐)에서 바로 WAIT_CAR 시작. amcl/Nav2 대기는 undock 뒤 LOCALIZE에서
        while rclpy.ok():
            rclpy.spin_once(m.nav, timeout_sec=0.01)
            m.step()
    except KeyboardInterrupt:
        pass
    finally:
        m.publish(0.0, 0.0)
        if m.state in ('NAVIGATING', 'TRACK'):
            m.nav.cancelTask()
        m.webcam.release()
        m.nav.destroy_node()
        rclpy.try_shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
