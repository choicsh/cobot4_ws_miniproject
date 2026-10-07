import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import cv2

# ================================
# 설정 상수
# ================================
WEBCAM_INDEX = 2                                    # yolov8_obj_det_wc.py와 동일
AMCL_TOPIC = '/robot5/amcl_pose'
H_PATH = os.path.expanduser('~/maps/webcam_H.npy')  # my_map 전용 (webcam 옮기면 다시 생성)
MIN_POINTS = 4
WINDOW_NAME = 'Webcam calib (click: robot floor center, u: undo, s: save, q: quit)'
# amcl_pose는 TRANSIENT_LOCAL로 발행됨 (nav2_simple_commander와 동일)
AMCL_QOS = QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                      reliability=QoSReliabilityPolicy.RELIABLE, depth=1)
# ================================


def fit_homography(px, xy):
    """px (N,2) webcam 픽셀, xy (N,2) map [m] -> (H, 점별 재투영 오차 [m])."""
    px, xy = np.float32(px), np.float32(xy)
    H, _ = cv2.findHomography(px, xy, cv2.RANSAC, 0.1)
    if H is None:
        return None, None
    err = np.linalg.norm(cv2.perspectiveTransform(px[:, None], H)[:, 0] - xy, axis=1)
    return H, err


def pixel_to_map(H, u, v):
    """webcam 픽셀 (u,v) -> map (x,y) [m]. 바닥 위의 점이어야 함."""
    return cv2.perspectiveTransform(np.float32([[[u, v]]]), H)[0, 0]


class WebcamCalib(Node):
    def __init__(self):
        super().__init__('webcam_calib')
        self.robot_xy = None
        self.px, self.xy = [], []
        self.H = np.load(H_PATH) if os.path.exists(H_PATH) else None
        if self.H is not None:
            self.get_logger().info(f'{H_PATH} 로드됨: 클릭하면 예측 좌표와 amcl 오차도 출력')
        self.create_subscription(PoseWithCovarianceStamped, AMCL_TOPIC, self.amcl_callback, AMCL_QOS)

    def amcl_callback(self, msg):
        p = msg.pose.pose.position
        self.robot_xy = (p.x, p.y)

    def on_click(self, event, u, v, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if self.robot_xy is None:
            self.get_logger().warn('amcl_pose 없음: rviz에서 2D Pose Estimate 먼저')
            return
        self.px.append((u, v))
        self.xy.append(self.robot_xy)
        self.show_points(f'추가: {self.point_str(len(self.px) - 1)}')

    def undo(self):
        if not self.px:
            self.get_logger().warn('지울 점 없음')
            return
        removed = self.point_str(len(self.px) - 1)
        self.px.pop()
        self.xy.pop()
        self.show_points(f'삭제: {removed}')

    def point_str(self, i):
        (u, v), (mx, my) = self.px[i], self.xy[i]
        s = f'#{i + 1} px=({u},{v}) map=({mx:.3f},{my:.3f})'
        if self.H is not None:
            x, y = pixel_to_map(self.H, u, v)
            s += f' | H 예측=({x:.3f},{y:.3f}) 오차 {np.hypot(x - mx, y - my):.3f} m'
        return s

    def show_points(self, header):
        """변경 내용과 현재까지 누적된 점 전체를 출력."""
        lines = [header, f'--- 현재 {len(self.px)}개 (저장하려면 최소 {MIN_POINTS}개) ---']
        lines += [self.point_str(i) for i in range(len(self.px))] or ['(없음)']
        self.get_logger().info('\n'.join(lines))

    def save(self):
        if len(self.px) < MIN_POINTS:
            self.get_logger().warn(f'점 {len(self.px)}개, 최소 {MIN_POINTS}개 필요')
            return
        H, err = fit_homography(self.px, self.xy)
        if H is None:
            self.get_logger().error('homography 실패: 점이 한 줄로 몰려 있지 않은지 확인')
            return
        for i, e in enumerate(err, 1):
            self.get_logger().info(f'#{i} 재투영 오차 {e:.3f} m')
        np.save(H_PATH, H)
        self.H = H
        self.get_logger().info(f'저장: {H_PATH} (최대 오차 {err.max():.3f} m, 목표 0.10 m 이하)')


def selftest():
    # webcam이 바닥을 비스듬히 보는 임의 H로 점을 만들고 다시 맞춰본다
    H_true = np.array([[0.004, 0.001, -1.0], [0.0005, -0.006, 3.0], [0.0001, 0.001, 1.0]])
    px = np.float32([[100, 400], [540, 420], [320, 250], [80, 150], [560, 160], [320, 460]])
    xy = cv2.perspectiveTransform(px[:, None], H_true)[:, 0]
    H, err = fit_homography(px, xy)
    assert err.max() < 1e-3, err
    assert np.allclose(pixel_to_map(H, 200, 300), pixel_to_map(H_true, 200, 300), atol=1e-3)
    print('selftest ok')


def main():
    if '--selftest' in sys.argv:
        selftest()
        return
    rclpy.init()
    node = WebcamCalib()
    cap = cv2.VideoCapture(WEBCAM_INDEX)
    cv2.namedWindow(WINDOW_NAME)
    cv2.setMouseCallback(WINDOW_NAME, node.on_click)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            ok, img = cap.read()
            if not ok:
                continue
            for i, (u, v) in enumerate(node.px, 1):
                cv2.circle(img, (u, v), 5, (0, 0, 255), -1)
                cv2.putText(img, str(i), (u + 6, v - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            txt = 'amcl: none' if node.robot_xy is None else 'amcl: ({:.2f}, {:.2f})'.format(*node.robot_xy)
            cv2.putText(img, f'{txt}  points: {len(node.px)}', (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow(WINDOW_NAME, img)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('u'):
                node.undo()
            elif key == ord('s'):
                node.save()
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
