import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import time

# ================================
# 설정 상수 (ros2 파라미터로 덮어쓰기 가능)
# ================================
IMAGE_TOPIC = '/robot5/oakd/rgb/image_raw/compressed'  # '/compressed'로 끝나면 CompressedImage로 구독
MODEL_PATH = 'src/mini_project/mini_project/yolo8n_merged_dataset_best.pt'                              # 학습한 모델이 있으면 경로로 변경
CONF_THRESHOLD = 0.5
WINDOW_NAME = 'YOLO Detection'
# ================================


class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')

        self.declare_parameter('image_topic', IMAGE_TOPIC)
        self.declare_parameter('model_path', MODEL_PATH)
        self.declare_parameter('conf', CONF_THRESHOLD)
        self.declare_parameter('show', True)
        self.declare_parameter('reliable', False)  # True: RELIABLE (WiFi에서 프레임이 밀려 지연 발생)

        image_topic = self.get_parameter('image_topic').value
        model_path = self.get_parameter('model_path').value
        self.conf = self.get_parameter('conf').value
        self.show = self.get_parameter('show').value
        reliable = self.get_parameter('reliable').value

        self.bridge = CvBridge()
        self.model = YOLO(model_path).to("cuda:0")
        self.get_logger().info(f'Model loaded: {model_path}')

        self.msg = None
        self.new_frame = False
        self.should_exit = False
        self.prev_time = time.time()

        # 진단용 통계 (1초마다 출력)
        self.recv_count = 0
        self.proc_count = 0
        self.t_decode = 0.0
        self.t_infer = 0.0
        self.t_total = 0.0
        self.latency = 0.0
        self.msg_kb = 0.0
        self.img_size = ''
        self.create_timer(1.0, self.report_stats)

        # 토픽 이름으로 압축/비압축 자동 판별
        self.compressed = image_topic.endswith('/compressed')
        msg_type = CompressedImage if self.compressed else Image
        if reliable:
            qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        else:
            qos = qos_profile_sensor_data
        self.subscription = self.create_subscription(
            msg_type,
            image_topic,
            self.image_callback,
            qos)
        self.get_logger().info(
            f'Subscribed to {image_topic} ({"CompressedImage" if self.compressed else "Image"}, '
            f'{"RELIABLE" if reliable else "BEST_EFFORT"})')

        # 결과 이미지 퍼블리시 (rqt_image_view 등으로 확인 가능)
        self.result_pub = self.create_publisher(Image, '/yolo/detection_image', 10)

    def image_callback(self, msg):
        # 콜백에서는 최신 메시지만 저장 (디코딩/추론은 메인 루프에서 수행 → 버려질 프레임은 디코딩 안 함)
        self.msg = msg
        self.new_frame = True
        self.recv_count += 1

    def report_stats(self):
        n = max(self.proc_count, 1)
        self.get_logger().info(
            f'[stats] recv {self.recv_count} Hz | processed {self.proc_count} Hz | '
            f'decode {self.t_decode / n * 1000:.1f} ms, infer {self.t_infer / n * 1000:.1f} ms, '
            f'total {self.t_total / n * 1000:.1f} ms | '
            f'latency {self.latency / n * 1000:.0f} ms | {self.img_size}, {self.msg_kb / n:.0f} KB/msg')
        self.recv_count = self.proc_count = 0
        self.t_decode = self.t_infer = self.t_total = self.latency = self.msg_kb = 0.0

    def process(self):
        if not self.new_frame or self.msg is None:
            return
        self.new_frame = False
        msg = self.msg
        t0 = time.perf_counter()
        # 지연 = 지금 시각 - 카메라 촬영 시각 (로봇과 PC 시계가 맞아야 정확함)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.latency += self.get_clock().now().nanoseconds * 1e-9 - stamp
        self.msg_kb += len(msg.data) / 1024

        if self.compressed:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        else:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        t1 = time.perf_counter()
        self.img_size = f'{frame.shape[1]}x{frame.shape[0]}'

        results = self.model(frame, conf=self.conf, verbose=False)[0]
        t2 = time.perf_counter()
        annotated = results.plot()

        for box in results.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            self.get_logger().info(
                f'{self.model.names[cls_id]} ({conf:.2f}) '
                f'bbox=({x1},{y1},{x2},{y2}) center=({(x1 + x2) // 2},{(y1 + y2) // 2})')

        # FPS 표시
        now = time.time()
        fps = 1.0 / max(now - self.prev_time, 1e-6)
        self.prev_time = now
        cv2.putText(annotated, f'FPS: {fps:.1f}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

        out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        out_msg.header = msg.header
        self.result_pub.publish(out_msg)

        if self.show:
            cv2.imshow(WINDOW_NAME, annotated)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.should_exit = True

        self.proc_count += 1
        self.t_decode += t1 - t0
        self.t_infer += t2 - t1
        self.t_total += time.perf_counter() - t0


def main():
    rclpy.init()
    node = YoloDetectionNode()

    try:
        while rclpy.ok() and not node.should_exit:
            rclpy.spin_once(node, timeout_sec=0.01)
            node.process()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
