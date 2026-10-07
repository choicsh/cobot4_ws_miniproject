"""docs/algorithm.md 그림 생성. 실제 코드 함수(visible_goal, depth_at, 테스트 simulate)를 그대로 호출한다.

실행 (ROS 환경 source 후, 저장소 루트에서): python3 docs/img/make_figures.py
"""
import math
import os
import sys

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path[:0] = [os.path.join(ROOT, 'src/mini_project'), os.path.join(ROOT, 'src/mini_project/test')]
from mini_project.align_check import depth_at  # noqa: E402
from mini_project.mission import (MAX_DEPTH_MM, TRACK_DIST, VIS_CLEARANCE, VIS_RADII,  # noqa: E402
                                  VIS_STEP_DEG, visible_goal)
from test_tracking import XY_TOL, simulate, straight  # noqa: E402

OUT = os.path.dirname(os.path.abspath(__file__))
# dataviz 기준 팔레트 (light, 브라우저 validator로 adjacent 6 / all-pairs 3 통과 확인)
C = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', '#008300']
SURFACE, INK, INK2, GRID = '#fcfcfb', '#0b0b0b', '#52514e', '#e4e3df'
plt.rcParams.update({
    'font.family': 'NanumGothic', 'axes.unicode_minus': False, 'figure.facecolor': SURFACE,
    'axes.facecolor': SURFACE, 'axes.edgecolor': INK2, 'axes.labelcolor': INK, 'text.color': INK,
    'xtick.color': INK2, 'ytick.color': INK2, 'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.8,
    'axes.spines.top': False, 'axes.spines.right': False, 'lines.linewidth': 2, 'font.size': 10,
})


# ---------- 1. visible_goal (실제 my_map) ----------
def fig_visible_goal():
    img = cv2.imread(os.path.expanduser('~/maps/my_map.pgm'), cv2.IMREAD_GRAYSCALE)[::-1]
    grid = np.where(img >= 250, 0, np.where(img <= 50, 100, -1)).astype(np.int8)  # trinary 근사
    res, origin = 0.05, (-6.104, -4.447)
    robot, car = (0.0, 0.0), (-2.4, 2.5)  # undock 위치, 예시 차 위치 (세 판정이 모두 나오는 곳)
    gx, gy, gyaw, gr = visible_goal(grid, res, origin, robot, car)

    # 후보별 판정 (visible_goal과 같은 검사, 그림 표시용)
    h, w = grid.shape
    c = int(math.ceil(VIS_CLEARANCE / res))

    def cell(x, y):
        return int((y - origin[1]) / res), int((x - origin[0]) / res)

    cand = {'통과': [], 'clearance 탈락 (벽/unknown 0.3m 안)': [], '시야 탈락 (차까지 선분에 벽)': []}
    for r in VIS_RADII:
        for a in np.radians(np.arange(0, 360, VIS_STEP_DEG)):
            x, y = car[0] + r * math.cos(a), car[1] + r * math.sin(a)
            j, i = cell(x, y)
            if not (c <= j < h - c and c <= i < w - c) or np.any(grid[j - c:j + c + 1, i - c:i + c + 1] != 0):
                cand['clearance 탈락 (벽/unknown 0.3m 안)'].append((x, y))
                continue
            n = int(r / res * 2) + 1
            rows, cols = zip(*(cell(x + (car[0] - x) * t, y + (car[1] - y) * t) for t in np.linspace(0, 1, n)))
            ok = not np.any(grid[np.clip(rows, 0, h - 1), np.clip(cols, 0, w - 1)] >= 50)
            cand['통과' if ok else '시야 탈락 (차까지 선분에 벽)'].append((x, y))
        if r == gr:
            break  # visible_goal은 후보가 나온 첫 반경에서 멈춤

    fig, ax = plt.subplots(figsize=(6.4, 6.4))
    vis = np.where(grid == 0, 252, np.where(grid == 100, 40, 200)).astype(np.uint8)
    ext = (origin[0], origin[0] + w * res, origin[1], origin[1] + h * res)
    ax.imshow(vis, cmap='gray', vmin=0, vmax=255, origin='lower', extent=ext, interpolation='nearest')
    ax.grid(False)
    for k, (name, pts) in enumerate(cand.items()):
        if pts:
            p = np.array(pts)
            ax.scatter(p[:, 0], p[:, 1], s=46, color=C[k], edgecolors=SURFACE, linewidths=1.5, label=name, zorder=3)
    ax.plot([gx, car[0]], [gy, car[1]], color=C[0], lw=1.5, ls='--', zorder=2)
    ax.scatter([gx], [gy], marker='*', s=320, color=C[0], edgecolors=INK, linewidths=1, zorder=4,
               label=f'선택된 goal (r={gr:.1f}m, 로봇과 가장 가까운 통과 후보)')
    ax.annotate('', xy=(gx + 0.35 * math.cos(gyaw), gy + 0.35 * math.sin(gyaw)), xytext=(gx, gy),
                arrowprops=dict(arrowstyle='->', color=INK, lw=1.5), zorder=5)
    ax.scatter(*car, marker='s', s=90, color=INK, zorder=5, label='차 (예시 map 좌표)')
    ax.scatter(*robot, marker='o', s=90, facecolors=SURFACE, edgecolors=INK, linewidths=2, zorder=5,
               label='로봇 (undock 위치)')
    ax.text(car[0] - 0.12, car[1] - 0.22, '차', color=INK, ha='right')
    ax.text(robot[0] + 0.12, robot[1] - 0.05, '로봇', color=INK)
    ax.set_xlim(min(car[0], robot[0]) - 1.4, max(car[0], robot[0]) + 0.8)
    ax.set_ylim(min(car[1], robot[1]) - 0.8, max(car[1], robot[1]) + 1.4)
    ax.set_xlabel('map x [m]')
    ax.set_ylabel('map y [m]')
    ax.set_title(f'NAVIGATE: visible_goal 후보 (반경 {", ".join(f"{r:g}" for r in VIS_RADII)}m 순, '
                 f'{VIS_STEP_DEG}° 간격)', loc='left', fontsize=11)
    ax.legend(loc='lower left', fontsize=8, framealpha=0.95)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'visible_goal.png'), dpi=130)
    plt.close(fig)
    return (gx, gy, math.degrees(gyaw), gr), {k: len(v) for k, v in cand.items()}


# ---------- 2. depth ROI ----------
def fig_depth_roi():
    hh = ww = 704
    yy, xx = np.mgrid[0:hh, 0:ww]
    depth = np.full((hh, ww), 4500, np.float32)                      # 먼 벽 4.5m
    floor = yy > 470
    depth[floor] = 1100 + (704 - yy[floor]) * 4.0                    # 바닥: 아래로 갈수록 가까움
    x1, y1, x2, y2 = 230, 250, 520, 480                              # bbox (비스듬히 선 길쭉한 차)
    cx, cy, a = 375, 365, math.radians(-38)                          # 차 실루엣: 회전한 직사각형 -> bbox 모서리는 배경
    ru = (xx - cx) * math.cos(a) + (yy - cy) * math.sin(a)
    rv = -(xx - cx) * math.sin(a) + (yy - cy) * math.cos(a)
    body = (np.abs(ru) <= 170) & (np.abs(rv) <= 55)
    depth[body] = 1150 + (xx[body] - cx) * 0.8                       # 비스듬한 면: 좌우로 depth 변화
    rng = np.random.default_rng(0)
    depth = np.clip(depth + rng.normal(0, 15, depth.shape), 0, None)
    depth[rng.random(depth.shape) < 0.04] = 0                        # OAK-D 무효 픽셀
    d = depth.astype(np.uint16)

    patch = max(2, int(min(x2 - x1, y2 - y1) / 6))
    u, v = int((x1 + x2) / 2), int((y1 + y2) / 2)
    roi_med = depth_at(d, u, v, patch, MAX_DEPTH_MM) / 1000
    box = d[y1:y2, x1:x2]
    box_med = float(np.median(box[box > 0])) / 1000

    fig, (a0, a1) = plt.subplots(1, 2, figsize=(10.5, 4.4), gridspec_kw={'width_ratios': [1, 1.15]})
    im = a0.imshow(np.where(d == 0, np.nan, d / 1000), cmap='Blues_r', vmin=0.8, vmax=4.6)
    a0.grid(False)
    a0.add_patch(plt.Rectangle((x1, y1), x2 - x1, y2 - y1, fill=False, ec=C[1], lw=2))
    a0.add_patch(plt.Rectangle((u - patch, v - patch), 2 * patch + 1, 2 * patch + 1, fill=False, ec=INK, lw=2))
    a0.text(x1, y1 - 10, 'YOLO bbox', color=C[1], fontsize=9)
    a0.annotate(f'ROI {2 * patch + 1}px\n(bbox 짧은 변의 ~1/3)', xy=(u - patch, v), xytext=(20, 150), fontsize=9,
                color=INK, bbox=dict(fc=SURFACE, ec='none', alpha=0.9), arrowprops=dict(arrowstyle='->', color=INK))
    a0.set_xticks([])
    a0.set_yticks([])
    a0.set_title('depth 이미지 (합성 예시: 차 1.15m, 바닥, 벽 4.5m)', loc='left', fontsize=11)
    fig.colorbar(im, ax=a0, fraction=0.046, label='depth [m]')

    bins = np.arange(0.8, 4.8, 0.05)
    a1.hist(box[box > 0] / 1000, bins=bins, color=C[1], alpha=0.55, label='bbox 전체 픽셀')
    roi = d[v - patch:v + patch + 1, u - patch:u + patch + 1]
    a1.hist(roi[roi > 0] / 1000, bins=bins, color=C[0], label='ROI 픽셀')
    a1.axvline(MAX_DEPTH_MM / 1000, color=INK2, ls='--', lw=1.5)
    a1.text(MAX_DEPTH_MM / 1000 - 0.05, a1.get_ylim()[1] * 0.45, f'MAX_DEPTH {MAX_DEPTH_MM / 1000:.0f}m\n이상 제외',
            ha='right', fontsize=9, color=INK2)
    a1.axvline(roi_med, color=C[0], lw=2)
    a1.text(roi_med + 0.05, a1.get_ylim()[1] * 0.6, f'ROI median\n{roi_med:.2f}m', color=INK, fontsize=9)
    a1.axvline(box_med, color=C[1], lw=2, ls=':')
    a1.text(box_med + 0.05, a1.get_ylim()[1] * 0.35, f'bbox 전체 median\n{box_med:.2f}m (배경 섞임)', color=INK, fontsize=9)
    a1.set_xlabel('depth [m]')
    a1.set_ylabel('픽셀 수')
    a1.set_title('depth 분포: ROI median은 차 표면, bbox 전체는 배경에 끌림', loc='left', fontsize=11)
    a1.legend(loc='upper left', bbox_to_anchor=(0.32, 1.0), fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'depth_roi.png'), dpi=130)
    plt.close(fig)
    return roi_med, box_med


# ---------- 3. 폐루프 추종 시뮬레이션 ----------
def fig_tracking_sim():
    scen = [
        ('정지', lambda t: (2.5, 0.0), {}),
        ('0.5m 계단 이동 (10s)', lambda t: (2.0, 0.0) if t < 10 else (2.0, 0.5), {}),
        ('0.1 m/s 직선', straight(0.1), {}),
        ('0.2 m/s 직선', straight(0.2), {}),
        ('0.15 m/s 원호', lambda t: (1.5 * math.cos(0.1 * t), 1.5 * math.sin(0.1 * t)), {'robot': (-0.2, -1.2, 1.2)}),
        ('0.4 m/s 직선 (한계)', straight(0.4), {}),
    ]
    fig, ax = plt.subplots(figsize=(9, 4.6))
    ax.axhspan(TRACK_DIST - XY_TOL, TRACK_DIST + XY_TOL, color=GRID, alpha=0.8, lw=0)
    ax.text(29.6, TRACK_DIST - 0.05, f'TRACK_DIST {TRACK_DIST:g}±{XY_TOL:g}m', ha='right', va='top', color=INK2, fontsize=9)
    ax.axhline(1.5, color=INK2, ls='--', lw=1.2)
    ax.text(29.6, 1.53, '이동 차 기준 1.5m', ha='right', va='bottom', color=INK2, fontsize=9)
    rows = []
    for k, (name, car, kw) in enumerate(scen):
        ts, d, lost = simulate(car, 30.0, **kw)
        ax.plot(ts, np.minimum(d, 2.6), color=C[k], label=name)
        end = d[-1]
        rows.append((name, float(d[ts > 5].max()), float(end), lost))
        if end > 2.6:  # 축 밖으로 나간 시나리오만 직접 표시 (나머지 값은 문서의 표)
            ax.text(10.4, 2.48, f'← {name}: {end:.2f}m까지 멀어져 놓침 (축 밖)', color=INK, fontsize=8.5, ha='left', va='top')
    ax.set_xlim(0, 30)
    ax.set_ylim(0.5, 2.6)
    ax.set_xlabel('시간 [s]')
    ax.set_ylabel('로봇-차 거리 [m]')
    ax.set_title('TRACK 폐루프 시뮬레이션 (follow_car.xml 근사: 측정 8Hz·0.12s 지연, 4Hz 재계산, 최대 0.26m/s)',
                 loc='left', fontsize=11)
    ax.legend(loc='upper left', bbox_to_anchor=(1.01, 1.0), fontsize=8.5, title='차 움직임', title_fontsize=9)
    fig.subplots_adjust(right=0.78)
    fig.savefig(os.path.join(OUT, 'tracking_sim.png'), dpi=130)
    plt.close(fig)
    return rows


if __name__ == '__main__':
    print('visible_goal:', fig_visible_goal())
    print('depth_roi (roi, bbox median):', fig_depth_roi())
    for r in fig_tracking_sim():
        print('sim:', r)
