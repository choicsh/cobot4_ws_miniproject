import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
import rosbag2_py
import os
from datetime import datetime

# ================================
# 설정 상수 (ros2 파라미터로 덮어쓰기 가능)
# ================================
IMAGE_TOPIC = '/robot5/oakd/rgb/image_raw/compressed'
BAG_ROOT = 'bags'          # 이 폴더 아래에 rgb_YYYYmmdd_HHMMSS 형태로 저장
STORAGE_ID = 'mcap'        # 'mcap' 또는 'sqlite3'
# ================================


class BagRecorderNode(Node):
    def __init__(self):
        super().__init__('bag_recorder_node')

        self.declare_parameter('image_topic', IMAGE_TOPIC)
        self.declare_parameter('bag_root', BAG_ROOT)
        self.declare_parameter('storage_id', STORAGE_ID)
        self.declare_parameter('reliable', False)

        self.topic = self.get_parameter('image_topic').value
        bag_root = self.get_parameter('bag_root').value
        storage_id = self.get_parameter('storage_id').value
        reliable = self.get_parameter('reliable').value

        os.makedirs(bag_root, exist_ok=True)
        self.bag_path = os.path.join(bag_root, datetime.now().strftime('rgb_%Y%m%d_%H%M%S'))

        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(
            rosbag2_py.StorageOptions(uri=self.bag_path, storage_id=storage_id),
            rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                        output_serialization_format='cdr'))
        self.writer.create_topic(rosbag2_py.TopicMetadata(
            id=0,
            name=self.topic,
            type='sensor_msgs/msg/CompressedImage',
            serialization_format='cdr'))

        if reliable:
            qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
        else:
            qos = qos_profile_sensor_data

        # raw=True: 역직렬화 없이 직렬화된 바이트를 그대로 받아서 bag에 기록
        self.subscription = self.create_subscription(
            CompressedImage, self.topic, self.callback, qos, raw=True)

        self.count = 0
        self.total = 0
        self.create_timer(1.0, self.report)
        self.get_logger().info(f'Recording {self.topic} -> {self.bag_path} (Ctrl+C to stop)')

    def callback(self, serialized_msg):
        self.writer.write(self.topic, serialized_msg, self.get_clock().now().nanoseconds)
        self.count += 1

    def report(self):
        self.total += self.count
        self.get_logger().info(f'{self.count} Hz | total {self.total} frames')
        self.count = 0

    def close(self):
        # writer를 해제해야 bag의 metadata.yaml이 기록됨
        del self.writer
        print(f'Saved: {self.bag_path} ({self.total + self.count} frames)')  # Ctrl+C 후엔 rosout 사용 불가


def main():
    rclpy.init()
    node = BagRecorderNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
