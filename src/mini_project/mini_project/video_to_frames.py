import argparse
import os
import cv2


def main():
    parser = argparse.ArgumentParser(description='영상을 프레임 단위 이미지로 저장')
    parser.add_argument('video', help='입력 영상 (webm/mp4/avi)')
    parser.add_argument('-o', '--output', default=None, help='저장 폴더 (기본: <영상이름>_frames)')
    parser.add_argument('-s', '--step', type=int, default=1,
                        help='N 프레임마다 1장 저장 (예: 30fps 영상에서 10 → 초당 3장)')
    parser.add_argument('-p', '--prefix', default=None, help='파일명 접두어 (기본: 영상 파일 이름)')
    parser.add_argument('-e', '--ext', default='jpg', choices=['jpg', 'png'])
    args = parser.parse_args()

    name = os.path.splitext(os.path.basename(args.video))[0]
    output = args.output or os.path.splitext(args.video)[0] + '_frames'
    prefix = args.prefix or name
    os.makedirs(output, exist_ok=True)

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print(f'영상을 열 수 없습니다: {args.video}')
        return

    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f'{args.video}: {total} frames, {cap.get(cv2.CAP_PROP_FPS):.1f} fps, step={args.step}')

    idx = 0
    saved = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        if idx % args.step == 0:
            cv2.imwrite(os.path.join(output, f'{prefix}_{idx:06d}.{args.ext}'), frame)
            saved += 1
        idx += 1

    cap.release()
    print(f'Saved {saved} images to {output}')


if __name__ == '__main__':
    main()
