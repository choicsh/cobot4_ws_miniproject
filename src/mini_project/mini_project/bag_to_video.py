import argparse
import os
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2

# ================================
# 설정 상수
# ================================
IMAGE_TOPIC = '/robot5/oakd/rgb/image_raw/compressed'
# 확장자별 코덱. pip/apt OpenCV에는 H.264 인코더가 없어서 mp4(mp4v)는 브라우저에서 재생 안 됨
# → 브라우저(Firefox/Chrome)에서 열려면 webm(VP8) 사용
FOURCC = {'.webm': 'VP80', '.mp4': 'mp4v', '.avi': 'MJPG'}
# ================================


def main():
    parser = argparse.ArgumentParser(description='rosbag의 CompressedImage 토픽을 영상(webm/mp4/avi)으로 저장')
    parser.add_argument('bag_path', help='bag 폴더 경로 (예: bags/rgb_20261002_180000)')
    parser.add_argument('-o', '--output', default=None, help='출력 파일 (기본: <bag_path>.webm)')
    parser.add_argument('-t', '--topic', default=IMAGE_TOPIC)
    parser.add_argument('--fps', type=float, default=None, help='미지정 시 bag의 실제 수신 속도로 계산')
    args = parser.parse_args()

    output = args.output or args.bag_path.rstrip('/') + '.webm'
    ext = os.path.splitext(output)[1].lower()
    if ext not in FOURCC:
        print(f'지원 확장자: {list(FOURCC)}')
        return

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag_path, storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[args.topic]))

    # 1차: 프레임 수와 시간 범위로 실제 fps 계산
    fps = args.fps
    if fps is None:
        stamps = []
        while reader.has_next():
            stamps.append(reader.read_next()[2])
        if len(stamps) < 2:
            print(f'{args.topic} 프레임이 부족합니다 ({len(stamps)}개)')
            return
        fps = (len(stamps) - 1) / ((stamps[-1] - stamps[0]) * 1e-9)
        reader.seek(0)

    # 2차: 디코딩해서 영상으로 기록
    writer = None
    count = 0
    while reader.has_next():
        _, data, _ = reader.read_next()
        msg = deserialize_message(data, CompressedImage)
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            continue
        if writer is None:
            h, w = frame.shape[:2]
            writer = cv2.VideoWriter(output, cv2.VideoWriter_fourcc(*FOURCC[ext]), fps, (w, h))
            if not writer.isOpened():
                print(f'VideoWriter 열기 실패: {output} ({FOURCC[ext]})')
                return
            print(f'{w}x{h} @ {fps:.1f} fps -> {output}')
        writer.write(frame)
        count += 1

    if writer is not None:
        writer.release()
    print(f'Saved {count} frames to {output}')


if __name__ == '__main__':
    main()
