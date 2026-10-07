import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2

# ================================
# 설정 상수
# ================================
RGB_TOPIC = '/robot5/oakd/rgb/image_raw/compressed'
DEPTH_TOPIC = '/robot5/oakd/stereo/image_raw/compressedDepth'
NEAR_M = 0.2                  # 컬러 범위 (m): NEAR 이하 = 빨강, FAR 이상 = 파랑
FAR_M = 1.5                   # 실행 중 '[' / ']' 키로 0.25m씩 조절
GAMMA = 0.5                   # <1이면 가까운 쪽에 색 단계를 더 배분 (1 = 선형)
ALPHA = 0.5                   # depth 오버레이 투명도 (0: rgb만, 1: depth만)
PATCH = 2                     # 클릭 지점 주변 (2*PATCH+1)^2 픽셀 median
WINDOW_NAME = 'RGB + Depth overlay (click: depth, [ ]: far range, q: quit)'
# ================================


def depth_to_heat(depth_mm, near=NEAR_M, far=FAR_M, gamma=GAMMA):
    """depth [mm] -> 0~255 (가까울수록 큼 = JET에서 빨강)."""
    t = np.clip((depth_mm / 1000.0 - near) / (far - near), 0, 1) ** gamma
    return ((1 - t) * 255).astype(np.uint8)


def depth_at(depth_mm, u, v, patch=PATCH):
    """(u,v) 주변 patch의 유효(>0) depth median [mm], 없으면 0."""
    h, w = depth_mm.shape
    roi = depth_mm[max(v - patch, 0):min(v + patch + 1, h),
                   max(u - patch, 0):min(u + patch + 1, w)]
    roi = roi[roi > 0]
    return float(np.median(roi)) if roi.size else 0.0


class AlignCheck(Node):
    def __init__(self):
        super().__init__('align_check')
        self.depth_mm = None
        self.rgb = None
        self.should_exit = False
        self.far = FAR_M

        self.create_subscription(CompressedImage, RGB_TOPIC,
                                 self.rgb_callback, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, DEPTH_TOPIC,
                                 self.depth_callback, qos_profile_sensor_data)

        cv2.namedWindow(WINDOW_NAME)
        cv2.setMouseCallback(WINDOW_NAME, self.mouse_callback)

    def depth_callback(self, msg):
        # compressedDepth = 12바이트 헤더 + PNG (depth_floor_ransac.py와 동일)
        d = cv2.imdecode(np.frombuffer(msg.data, np.uint8)[12:], cv2.IMREAD_UNCHANGED)
        if d is not None:
            self.depth_mm = d

    def rgb_callback(self, msg):
        if self.should_exit:
            return
        self.rgb = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if self.rgb is None or self.depth_mm is None:
            self.get_logger().warn('Waiting for rgb/depth...', throttle_duration_sec=2.0)
            return
        h, w = self.rgb.shape[:2]
        # OAK-D align으로 rgb와 depth 모두 704x704 → 픽셀 좌표가 1:1 대응
        if self.depth_mm.shape != (h, w):
            self.get_logger().error(f'rgb {w}x{h} != depth {self.depth_mm.shape[1]}x{self.depth_mm.shape[0]}, '
                                    'OAK-D align/해상도 설정 확인', throttle_duration_sec=2.0)
            return
        depth = self.depth_mm
        vis = cv2.applyColorMap(depth_to_heat(depth, far=self.far), cv2.COLORMAP_JET)
        out = cv2.addWeighted(self.rgb, 1 - ALPHA, vis, ALPHA, 0)
        out[depth == 0] = self.rgb[depth == 0]  # depth 없는 곳은 rgb 그대로

        # 범례: 왼쪽(가까움, 빨강) -> 오른쪽(멂, 파랑)
        bar = depth_to_heat(np.linspace(NEAR_M, self.far, w) * 1000, far=self.far)
        out[h - 20:] = cv2.applyColorMap(np.tile(bar, (20, 1)), cv2.COLORMAP_JET)
        for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
            x = min(int(frac * (w - 1)), w - 45)
            cv2.putText(out, f'{NEAR_M + frac * (self.far - NEAR_M):.2f}', (x, h - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        cv2.imshow(WINDOW_NAME, out)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.should_exit = True
        elif key in (ord('['), ord(']')):
            self.far = max(NEAR_M + 0.25, self.far + (0.25 if key == ord(']') else -0.25))
            self.get_logger().info(f'color range {NEAR_M:.2f} ~ {self.far:.2f} m')

    def mouse_callback(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN or self.rgb is None or self.depth_mm is None:
            return
        self.get_logger().info(f'(u={x}, v={y}) = {depth_at(self.depth_mm, x, y) / 1000:.3f} m')


def selftest():
    d = np.zeros((10, 10), np.uint16)
    d[4:7, 4:7] = 1500
    d[5, 5] = 9999  # 튀는 값 하나는 median이 무시
    assert depth_at(d, 5, 5, patch=1) == 1500
    assert depth_at(d, 0, 0, patch=1) == 0.0  # 유효값 없음
    heat = depth_to_heat(np.array([100, 500, 1000, 3000]))
    assert heat[0] == 255 and heat[-1] == 0  # NEAR 이하 최대, FAR 이상 최소
    assert heat[0] >= heat[1] > heat[2] > heat[3]  # 가까울수록 큼
    assert depth_to_heat(np.array([700]))[0] < 128  # gamma: 앞쪽 0.5m에 색 범위 절반 이상
    print('selftest ok')


def main():
    if '--selftest' in sys.argv:
        selftest()
        return
    rclpy.init()
    node = AlignCheck()
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
