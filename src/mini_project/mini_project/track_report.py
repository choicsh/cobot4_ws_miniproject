"""mission 로그 -> 추종 성능 지표 (4단계: 실기 결과 측정).

사용: ros2 run mini_project track_report [로그 파일]
로그 파일을 안 주면 ~/.ros/log에서 basic_navigator 로그가 있는 가장 최근 python3_*.log.
"""
import glob
import os
import re
import sys

import numpy as np

STILL_M = 0.05    # 정지 구간: 구간 시작점에서 이 거리 안에 머무는 car map 좌표들
STILL_SEC = 3.0   # 정지 구간으로 보는 최소 길이

LINE = re.compile(r'\[(\d+\.\d+)\] \[[^\]]*basic_navigator\]: (.*)')
STATE = re.compile(r'^(\w+) -> (\w+)$')
TRACK = re.compile(r'TRACK depth ([\d.]+) m \| car map \(([-\d.]+), ([-\d.]+)\)')


def parse(lines):
    """로그 줄들 -> (상태 전이 [(t, from, to)], TRACK 측정 [(t, depth, x, y)], goal 전송 시각 [t], TF 실패 수)."""
    trans, track, goals, tf_fail = [], [], [], 0
    for line in lines:
        m = LINE.search(line)
        if not m:
            continue
        t, msg = float(m.group(1)), m.group(2)
        if (s := STATE.match(msg)):
            trans.append((t, s.group(1), s.group(2)))
        elif (s := TRACK.search(msg)):
            track.append((t, float(s.group(1)), float(s.group(2)), float(s.group(3))))
        elif msg.startswith('Navigating to goal'):
            goals.append(t)
        elif msg.startswith('TF map <-'):
            tf_fail += 1
    return trans, track, goals, tf_fail


def track_intervals(trans, t_end):
    """TRACK 상태였던 [(시작, 끝)] 구간."""
    out, start = [], None
    for t, a, b in trans:
        if b == 'TRACK':
            start = t
        elif a == 'TRACK' and start is not None:
            out.append((start, t))
            start = None
    if start is not None:
        out.append((start, t_end))
    return out


def still_segments(track):
    """car map 좌표가 STILL_M 안에 STILL_SEC 이상 머문 구간들의 [(길이 s, std x, std y)]."""
    out, seg = [], []
    for p in track:
        if seg and (np.hypot(p[2] - seg[0][2], p[3] - seg[0][3]) > STILL_M or p[0] - seg[-1][0] > 1.5):
            if seg[-1][0] - seg[0][0] >= STILL_SEC:
                out.append(seg)
            seg = []
        seg.append(p)
    if seg and seg[-1][0] - seg[0][0] >= STILL_SEC:
        out.append(seg)
    return [(s[-1][0] - s[0][0], float(np.std([q[2] for q in s])), float(np.std([q[3] for q in s]))) for s in out]


def report(lines):
    """지표 dict."""
    trans, track, goals, tf_fail = parse(lines)
    t_end = max([t for t, *_ in trans] + [p[0] for p in track] + goals + [0.0])
    iv = track_intervals(trans, t_end)
    track_sec = sum(b - a for a, b in iv)
    n_goal = sum(1 for g in goals if any(a <= g <= b for a, b in iv))
    depth = [p[1] for p in track]
    return {
        'track_sec': track_sec,
        'track_entries': len(iv),
        'track_to_find': sum(1 for _, a, b in trans if a == 'TRACK' and b == 'FIND'),
        'goal_hz': n_goal / track_sec if track_sec > 0 else 0.0,
        'depth_min_med_max': (min(depth), float(np.median(depth)), max(depth)) if depth else None,
        'still': still_segments(track),
        'tf_fail_logs': tf_fail,
    }


def latest_log():
    logs = sorted(glob.glob(os.path.expanduser('~/.ros/log/python3_*.log')), key=os.path.getmtime, reverse=True)
    for f in logs:
        with open(f, errors='ignore') as fh:
            if 'basic_navigator' in fh.read(4096):
                return f
    return None


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else latest_log()
    if not path:
        print('mission 로그를 찾지 못함: 경로를 인자로 주세요')
        return
    with open(path, errors='ignore') as f:
        r = report(f)
    print(f'로그: {path}')
    print(f"TRACK 시간 {r['track_sec']:.1f}s, 진입 {r['track_entries']}회, TRACK->FIND {r['track_to_find']}회")
    print(f"TRACK 중 goal 전송 {r['goal_hz']:.1f} Hz, TF 실패 로그 {r['tf_fail_logs']}회 (1초 throttle)")
    if r['depth_min_med_max']:
        print('depth min/median/max: {:.2f} / {:.2f} / {:.2f} m'.format(*r['depth_min_med_max']))
    for sec, sx, sy in r['still']:
        print(f'정지 구간 {sec:.1f}s: car map std x {sx * 100:.1f} cm, y {sy * 100:.1f} cm')
    if not r['still']:
        print(f'정지 구간 없음 (car map이 {STILL_M * 100:.0f}cm 안에 {STILL_SEC:.0f}s 이상 머문 구간)')


if __name__ == '__main__':
    main()
