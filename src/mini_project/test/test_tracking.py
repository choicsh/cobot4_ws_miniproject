"""TRACK 추종 성능 테스트 (단계별).

1. 좌표 계산: bbox + depth + TF -> 차 map 좌표가 정답과 1cm 이내
2. 회전 중 안정성: 회전하며 지연된 영상을 처리해도 정지한 차 좌표 편차 2cm 이내 (최신 TF면 실패하는 것도 확인)
3. 폐루프 추종 시뮬레이션: 차 궤적별로 단순화한 로봇이 TRACK_DIST를 유지하는지 (Nav2 근사 모델)
4. 실기 로그 분석기(track_report) 파서

실행: python3 -m pytest src/mini_project/test/test_tracking.py -v   (ROS 환경 source 후)
"""
import math
from types import SimpleNamespace

import numpy as np
import pytest
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer

from mini_project.mission import LOST_SEC, TRACK_DIST, Mission, approach_goal
from mini_project import track_report

K = np.array([[500.0, 0, 352], [0, 500.0, 352], [0, 0, 1]])
CAM_H = 0.25  # 카메라 높이 (m)
# optical(x 오른쪽, y 아래, z 앞) -> base(x 앞, y 왼쪽, z 위)
R_BASE_OPT = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)


def tf_msg(parent, child, t, xyz, rot):
    m = TransformStamped()
    m.header.frame_id, m.child_frame_id = parent, child
    m.header.stamp = Time(nanoseconds=round(t * 1e9)).to_msg()
    m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = map(float, xyz)
    q = Rotation.from_matrix(rot).as_quat()  # x, y, z, w
    m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = map(float, q)
    return m


def cam_pose(x, y, yaw):
    """odom에서 카메라 optical frame의 (위치, 회전행렬)."""
    return np.array([x, y, CAM_H]), Rotation.from_euler('z', yaw).as_matrix() @ R_BASE_OPT


def make_mission(buf):
    m = Mission.__new__(Mission)
    m.nav = SimpleNamespace(get_logger=lambda: SimpleNamespace(warn=lambda *a, **k: None), error=lambda *a: None)
    m.K, m.depth_frame, m.tf_buffer = K, 'cam', buf
    return m


def observe(m, pos, rot, car_odom):
    """정답 카메라 자세에서 차 점을 찍은 bbox와 depth를 m에 넣고 bbox 반환. 화면 밖이면 None."""
    p = rot.T @ (np.asarray(car_odom) - pos)  # optical 좌표
    if p[2] <= 0.1:
        return None
    u, v = K[0, 0] * p[0] / p[2] + K[0, 2], K[1, 1] * p[1] / p[2] + K[1, 2]
    if not (40 <= u <= 664 and 40 <= v <= 664):
        return None
    m.depth_mm = np.full((704, 704), round(p[2] * 1000), np.uint16)
    return (u - 30, v - 30, u + 30, v + 30)


def map_odom(buf, t0, t1):
    """map <- odom: (1, 2) 이동 + 0.3rad 회전 (체인 검증용), 오래된 값만 있어도 됨."""
    for t in (t0, t1):
        buf.set_transform(tf_msg('map', 'odom', t, (1.0, 2.0, 0.0), Rotation.from_euler('z', 0.3).as_matrix()), 't')


def to_map(xy):
    c, s = math.cos(0.3), math.sin(0.3)
    return np.array([1.0 + c * xy[0] - s * xy[1], 2.0 + s * xy[0] + c * xy[1]])


# ---------- 1. 좌표 계산 ----------
@pytest.mark.parametrize('yaw', [0.0, math.pi / 2, -3 * math.pi / 4])
@pytest.mark.parametrize('rng, bearing', [(1.5, 0.0), (1.0, 0.4), (2.5, -0.35)])
def test_1_car_in_map_accuracy(yaw, rng, bearing):
    buf = Buffer()
    map_odom(buf, 9.0, 11.0)
    pos, rot = cam_pose(0.3, -0.2, yaw)
    for t in (9.0, 11.0):
        buf.set_transform(tf_msg('odom', 'cam', t, pos, rot), 't')
    car = pos + rng * np.array([math.cos(yaw + bearing), math.sin(yaw + bearing), 0]) + [0, 0, -0.1]
    m = make_mission(buf)
    m.rgb_stamp = 10.0
    box = observe(m, pos, rot, car)
    assert box is not None
    cam_xy, car_xy = m.car_in_map(box, (704, 704))
    assert np.linalg.norm(np.array(car_xy) - to_map(car[:2])) < 0.01, (car_xy, to_map(car[:2]))
    assert np.linalg.norm(np.array(cam_xy) - to_map(pos[:2])) < 0.01


# ---------- 2. 회전 중 안정성 ----------
def rotating_buffer(omega, t0=9.0, t1=13.0, hz=50):
    """제자리 회전(omega rad/s)하는 카메라의 odom TF + 오래된 map<-odom."""
    buf = Buffer()
    map_odom(buf, t0, t0 + 0.1)
    for t in np.arange(t0, t1, 1 / hz):
        pos, rot = cam_pose(0.0, 0.0, omega * (t - 10.25))
        buf.set_transform(tf_msg('odom', 'cam', t, pos, rot), 't')
    return buf


def car_spread(latency, use_rgb_stamp, omega=1.0):
    """정지한 차(카메라 앞 1.5m)를 회전 중 8Hz로 찍고 latency 뒤 처리 -> 차 map 좌표 최대 편차 (m)."""
    buf = rotating_buffer(omega)
    m = make_mission(buf)
    car = np.array([1.5, 0.0, 0.15])
    pts = []
    for t_img in np.arange(10.0, 10.5, 1 / 8):
        pos, rot = cam_pose(0.0, 0.0, omega * (t_img - 10.25))
        box = observe(m, pos, rot, car)
        assert box is not None
        m.rgb_stamp = t_img if use_rgb_stamp else t_img + latency  # 최신 TF = 처리 시각의 자세
        pts.append(m.car_in_map(box, (704, 704))[1])
    pts = np.array(pts)
    return float(np.max(np.linalg.norm(pts - to_map(car[:2]), axis=1)))


@pytest.mark.parametrize('latency', [0.06, 0.12, 0.2])
def test_2_rotation_stable_with_rgb_stamp(latency):
    assert car_spread(latency, use_rgb_stamp=True) < 0.02


def test_2_latest_tf_would_drift():
    # 이전 방식(최신 TF): 1rad/s 회전, 0.2s 지연이면 1.5m 앞 차가 ~0.3m 튐 (실측 ±40cm와 같은 현상)
    assert car_spread(0.2, use_rgb_stamp=False) > 0.1


# ---------- 3. 폐루프 추종 시뮬레이션 ----------
MAX_V, MAX_W = 0.26, 1.0          # config/nav2.yaml DWB max_vel_x, max_vel_theta
XY_TOL, YAW_TOL = 0.1, 0.25       # config/nav2.yaml goal tolerance
HFOV = math.atan(352 / 500)       # K 기준 반화각 (~35deg)
PERCEPT_HZ, LATENCY = 8.0, 0.12   # rgb-depth 짝 처리 주기, 처리 지연 (실측)
REPLAN_HZ = 4.0                   # config/follow_car.xml RateController hz
NEAR, FAR = 0.8, 4.0              # 0.8m 안쪽은 차가 화면 하단에 잘림 (실측), 4m 이상은 depth 필터


def simulate(car_at, t_end, robot=(0.0, 0.0, 0.0), noise=0.03, seed=0):
    """car_at(t) -> (x, y). 반환: 시각, 로봇-차 거리 배열, 놓친 횟수 (LOST_SEC 이상 미감지).

    TRACK = follow_car.xml 근사: 차 측정(PERCEPT_HZ, LATENCY 전 위치 + 노이즈)은 goal_update로만 들어가고,
    REPLAN_HZ마다 로봇->차 직선 경로를 TRACK_DIST만큼 잘라 끝점을 goal로 (approach_goal과 같은 점).
    로봇 = Nav2 근사: goal 방향으로 회전(MAX_W), 방향 오차 0.5rad 안이면 전진(MAX_V, 비례 감속),
    XY_TOL 안이면 goal yaw로 회전만. 후진 없음.
    """
    rnd = np.random.default_rng(seed)
    x, y, th = robot
    dt, goal, meas, last_seen, next_p, next_r = 0.02, None, None, 0.0, 0.0, 0.0
    ts, dists, lost, was_lost = [], [], 0, False
    for t in np.arange(0.0, t_end, dt):
        if t >= next_p:
            next_p += 1 / PERCEPT_HZ
            cx, cy = car_at(max(t - LATENCY, 0.0))
            d, b = math.hypot(cx - x, cy - y), math.atan2(cy - y, cx - x) - th
            if NEAR <= d <= FAR and abs(math.atan2(math.sin(b), math.cos(b))) < HFOV:
                meas = (cx + rnd.normal(0, noise), cy + rnd.normal(0, noise))
                last_seen, was_lost = t, False
            elif t - last_seen > LOST_SEC and not was_lost:
                lost, was_lost = lost + 1, True
        if meas is not None and t >= next_r:  # RateController: 최신 goal_update로 경로 재계산 + TruncatePath
            next_r += 1 / REPLAN_HZ
            goal = approach_goal((x, y), meas, TRACK_DIST)
        if goal is not None:
            gx, gy, gyaw = goal
            dist = math.hypot(gx - x, gy - y)
            if dist > XY_TOL:
                err = math.atan2(gy - y, gx - x) - th
                v = min(MAX_V, 1.0 * dist) if abs(math.atan2(math.sin(err), math.cos(err))) < 0.5 else 0.0
            else:
                err, v = gyaw - th, 0.0
            err = math.atan2(math.sin(err), math.cos(err))
            w = 0.0 if dist <= XY_TOL and abs(err) < YAW_TOL else max(-MAX_W, min(MAX_W, 2.0 * err))
            x, y, th = x + v * math.cos(th) * dt, y + v * math.sin(th) * dt, th + w * dt
        cx, cy = car_at(t)
        ts.append(t)
        dists.append(math.hypot(cx - x, cy - y))
    return np.array(ts), np.array(dists), lost


def straight(speed, start=(1.2, 0.0)):
    # 1.2m에서 출발: 실기 로그에서 TRACK 진입 거리 1.16~1.26m. 2.0m에서 출발하면 0.2m/s는
    # 로봇(0.26m/s)과의 속도 차가 작아 초기 과도 구간만 길어짐 (정상 상태는 같음)
    return lambda t: (start[0] + speed * t, start[1])


def test_3_static_car_settles_at_track_dist():
    ts, d, lost = simulate(lambda t: (2.5, 0.0), 20.0)
    assert lost == 0
    assert abs(d[-1] - TRACK_DIST) <= XY_TOL + 0.02, d[-1]


def test_3_step_move_resettles():
    car = lambda t: (2.0, 0.0) if t < 10 else (2.0, 0.5)  # noqa: E731  10s에 옆으로 0.5m
    ts, d, lost = simulate(car, 25.0)
    assert lost == 0
    assert abs(d[-1] - TRACK_DIST) <= XY_TOL + 0.02, d[-1]


@pytest.mark.parametrize('speed', [0.1, 0.2])
def test_3_moving_car_followed(speed):
    ts, d, lost = simulate(straight(speed), 30.0)
    assert lost == 0
    assert d[ts > 5].max() <= 1.5, d[ts > 5].max()


def test_3_arc_followed():
    r, w = 1.5, 0.1  # 반지름 1.5m 원호, 0.15 m/s
    ts, d, lost = simulate(lambda t: (r * math.cos(w * t), r * math.sin(w * t)), 30.0, robot=(-0.2, -1.2, 1.2))
    assert lost == 0
    assert d[ts > 5].max() <= 1.5, d[ts > 5].max()


def test_3_too_fast_car_not_followed():
    # 한계 기록: 차 0.4 m/s > 로봇 MAX_V 0.26 m/s -> 따라가지 못함 (실측 0.46 m/s에서 놓침과 같은 현상)
    ts, d, lost = simulate(straight(0.4), 30.0)
    assert d.max() > 1.5 or lost > 0


# ---------- 4. 실기 로그 분석기 ----------
LOG = """\
[INFO] [100.000000] [robot5.basic_navigator]: NAVIGATING -> TRACK
[INFO] [100.100000] [robot5.basic_navigator]: TRACK depth 1.00 m | car map (-1.00, 2.00) | goal (-1.0, 1.0, 90 deg)
[INFO] [100.200000] [robot5.basic_navigator]: Navigating to goal: -1.0 1.0...
[INFO] [101.000000] [robot5.basic_navigator]: TRACK depth 1.10 m | car map (-1.01, 2.01) | goal (-1.0, 1.0, 90 deg)
[INFO] [101.200000] [robot5.basic_navigator]: Navigating to goal: -1.0 1.0...
[INFO] [102.000000] [robot5.basic_navigator]: TRACK depth 1.10 m | car map (-1.00, 2.01) | goal (-1.0, 1.0, 90 deg)
[INFO] [103.000000] [robot5.basic_navigator]: TRACK depth 1.10 m | car map (-1.01, 2.00) | goal (-1.0, 1.0, 90 deg)
[INFO] [104.000000] [robot5.basic_navigator]: TRACK depth 1.20 m | car map (-1.00, 2.02) | goal (-1.0, 1.0, 90 deg)
[WARN] [104.500000] [robot5.basic_navigator]: TF map <- 'cam' 실패: extrapolation
[INFO] [105.000000] [robot5.basic_navigator]: TRACK -> FIND
[INFO] [105.500000] [robot5.basic_navigator]: Navigating to goal: -9 9...
[INFO] [106.000000] [robot5.basic_navigator]: FIND -> TRACK
[INFO] [108.000000] [robot5.basic_navigator]: TRACK -> FIND
"""


def test_4_track_report():
    r = track_report.report(LOG.splitlines())
    assert r['track_sec'] == pytest.approx(5.0 + 2.0)
    assert r['track_entries'] == 2 and r['track_to_find'] == 2
    assert r['goal_hz'] == pytest.approx(2 / 7.0)  # TRACK 밖(105.5)의 goal은 제외
    assert r['depth_min_med_max'] == pytest.approx((1.0, 1.1, 1.2))  # 1.0, 1.1, 1.1, 1.1, 1.2
    assert r['tf_fail_logs'] == 1
    (sec, sx, sy), = r['still']  # 100.1~104.0 동안 2cm 안에 머묾
    assert sec == pytest.approx(3.9) and sx < 0.01 and sy < 0.01
