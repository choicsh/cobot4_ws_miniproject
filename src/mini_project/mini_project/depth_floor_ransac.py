import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, CameraInfo
import numpy as np
import cv2

# ================================
# 설정 상수
# ================================
DEPTH_TOPIC = '/robot5/oakd/stereo/image_raw/compressedDepth'
CAMERA_INFO_TOPIC = '/robot5/oakd/stereo/camera_info'
NORMALIZE_DEPTH_RANGE = 3.0   # 시각화 정규화 범위 (m)
MIN_DEPTH = 0.3               # 유효 거리 범위 (m), 밖은 무효 처리
MAX_DEPTH = 3.0               # 3m 이상은 노이즈(p95 ~10cm)로 바닥/물체 구분 불가
SAMPLE_STEP = 4               # RANSAC용 픽셀 다운샘플 간격
RANSAC_ITERS = 100            # RANSAC 가설 수
THRESH_BASE = 0.02            # 바닥 판정 거리 = BASE + K*z^2 (m)
THRESH_K = 0.01               # 스테레오 오차 ~ z^2: 1m 3cm, 2m 6cm, 3m 11cm
MAX_TILT_DEG = 30.0           # 바닥 법선과 카메라 +y(아래) 축 사이 허용 각도
OPEN_KERNEL = 5               # 남은 점 마스크 morphology open 크기 (0이면 끔)
# 고정 평면 (a, b, c, d): a*x + b*y + c*z + d = 0, 카메라 optical frame, m 단위.
# 법선은 아래(+y, b>0)를 향해야 함. 's' 키로 출력된 값을 그대로 넣으면 RANSAC을 건너뜀
FIXED_PLANE = None
WINDOW_NAME = 'Raw | Filtered + floor removed  (s: print plane, q: quit)'
# ================================


def floor_thresh(z):
    return THRESH_BASE + THRESH_K * z * z


def fit_floor_ransac(pts, iters=RANSAC_ITERS, max_tilt_deg=MAX_TILT_DEG,
                     rng=np.random.default_rng()):
    """pts (N,3) [m] -> 평면 (a,b,c,d) 또는 None. 가설 전체를 한 번에 벡터 연산."""
    if len(pts) < 3:
        return None
    p0, p1, p2 = (pts[i] for i in rng.integers(0, len(pts), (3, iters)))
    n = np.cross(p1 - p0, p2 - p0)
    norm = np.linalg.norm(n, axis=1)
    ok = norm > 1e-9
    n = n[ok] / norm[ok, None]
    p0 = p0[ok]
    # 바닥은 법선이 카메라 y축(아래)과 거의 평행 → 벽/물체 평면 배제
    ok = np.abs(n[:, 1]) > np.cos(np.radians(max_tilt_deg))
    n, p0 = n[ok], p0[ok]
    if len(n) == 0:
        return None
    d = -np.einsum('ij,ij->i', n, p0)
    thr = floor_thresh(pts[:, 2])
    counts = (np.abs(pts @ n.T + d) < thr[:, None]).sum(axis=0)
    best = np.argmax(counts)
    inliers = pts[np.abs(pts @ n[best] + d[best]) < thr]
    # 인라이어로 최소자승 재추정 (고정값 추출 시 안정적)
    c = inliers.mean(axis=0)
    nn = np.linalg.svd(inliers - c, full_matrices=False)[2][2]
    if nn[1] < 0:  # 부호 통일: 법선이 카메라 아래(+y)를 향하도록
        nn = -nn
    return np.array([*nn, -nn @ c])


class DepthFloorRansac(Node):
    def __init__(self):
        super().__init__('depth_floor_ransac')
        self.K = None
        self.info_size = None  # camera_info (w, h)
        self.rays = None  # 픽셀별 (x/z, y/z)
        self.plane = None if FIXED_PLANE is None else np.array(FIXED_PLANE, float)
        self.kernel = np.ones((OPEN_KERNEL, OPEN_KERNEL), np.uint8) if OPEN_KERNEL else None
        self.last_stamp = None
        self.hz = 0.0
        self.should_exit = False

        self.create_subscription(CompressedImage, DEPTH_TOPIC,
                                 self.depth_callback, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC,
                                 self.camera_info_callback, qos_profile_sensor_data)

    def camera_info_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.info_size = (msg.width, msg.height)
            self.get_logger().info(f'CameraInfo received: {msg.width}x{msg.height}, K={self.K.tolist()}')

    def build_rays(self, w, h):
        K = self.K.copy()
        iw, ih = self.info_size
        if (iw, ih) != (w, h) and iw and ih:
            # align_depth 등으로 camera_info 해상도와 depth 해상도가 다를 때
            self.get_logger().warn(f'camera_info {iw}x{ih} != depth {w}x{h}, scaling K')
            K[0] *= w / iw
            K[1] *= h / ih
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        u, v = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
        self.rays = ((u - cx) / fx, (v - cy) / fy)

    def depth_callback(self, msg):
        if self.should_exit:
            return
        if self.K is None:
            self.get_logger().warn('Waiting for CameraInfo...', throttle_duration_sec=2.0)
            return
        if '16UC1' not in msg.format:
            self.get_logger().error(f'Unsupported format: {msg.format}', once=True)
            return
        t0 = time.perf_counter()
        if self.last_stamp is not None:
            self.hz = 0.9 * self.hz + 0.1 / max(t0 - self.last_stamp, 1e-6)
        self.last_stamp = t0

        # compressedDepth = 12바이트 헤더(format int32 + depthParam float[2]) + PNG
        depth_mm = cv2.imdecode(np.frombuffer(msg.data, np.uint8)[12:], cv2.IMREAD_UNCHANGED)
        if depth_mm is None:
            self.get_logger().error('PNG decode failed', throttle_duration_sec=2.0)
            return
        z = depth_mm.astype(np.float32) / 1000.0
        h, w = z.shape
        if self.rays is None or self.rays[0].shape != z.shape:
            self.build_rays(w, h)
        xr, yr = self.rays

        raw_valid = z > 0
        valid = (z >= MIN_DEPTH) & (z <= MAX_DEPTH)

        if FIXED_PLANE is None:
            s = np.s_[::SAMPLE_STEP, ::SAMPLE_STEP]
            vs, zs = valid[s], z[s]
            pts = np.stack([xr[s][vs] * zs[vs], yr[s][vs] * zs[vs], zs[vs]], axis=1)
            plane = fit_floor_ransac(pts)
            if plane is not None:
                self.plane = plane

        keep = valid
        if self.plane is not None:
            a, b, c, d = self.plane
            # 부호 있는 거리: 법선이 아래(+y)라서 바닥 위 물체는 음수, 바닥 아래(반사/노이즈)는 양수
            signed = z * (a * xr + b * yr + c) + d
            keep = valid & (signed < -floor_thresh(z))
        if self.kernel is not None:
            keep = cv2.morphologyEx(keep.astype(np.uint8), cv2.MORPH_OPEN, self.kernel).astype(bool)
        proc_ms = (time.perf_counter() - t0) * 1000

        vis = (np.clip(z / NORMALIZE_DEPTH_RANGE, 0, 1) * 255).astype(np.uint8)
        vis = cv2.applyColorMap(vis, cv2.COLORMAP_JET)
        removed = vis.copy()
        vis[~raw_valid] = 0
        removed[~keep] = 0
        out = np.hstack([vis, removed])
        txt = f'{self.hz:4.1f} Hz  proc {proc_ms:4.1f} ms'
        if self.plane is not None:
            txt += '  plane ' + ', '.join(f'{x:+.3f}' for x in self.plane)
            txt += ' (FIXED)' if FIXED_PLANE is not None else ''
        cv2.putText(out, txt, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.imshow(WINDOW_NAME, out)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.should_exit = True
        elif key == ord('s') and self.plane is not None:
            self.get_logger().info(f'FIXED_PLANE = ({", ".join(f"{x:.4f}" for x in self.plane)})')


def selftest():
    rng = np.random.default_rng(0)
    # 카메라 0.3m 위 바닥(y=0.3) 2000점 + 그보다 큰 벽(z=2.0) 4000점
    floor = np.stack([rng.uniform(-1, 1, 2000), np.full(2000, 0.3), rng.uniform(0.5, 3, 2000)], 1)
    wall = np.stack([rng.uniform(-1, 1, 4000), rng.uniform(-1, 0.3, 4000), np.full(4000, 2.0)], 1)
    pts = np.vstack([floor, wall]) + rng.normal(0, 0.003, (6000, 3))
    a, b, c, d = fit_floor_ransac(pts, rng=rng)
    assert b > 0.99 and abs(d + 0.3) < 0.01, (a, b, c, d)
    # 2.8m 바닥에 측정 p95 수준(9cm) 오차가 있어도 바닥 판정 범위 안
    assert floor_thresh(2.8) > 0.09
    print('selftest ok:', np.round([a, b, c, d], 4))


def main():
    if '--selftest' in sys.argv:
        selftest()
        return
    rclpy.init()
    node = DepthFloorRansac()
    try:
        while rclpy.ok() and not node.should_exit:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
