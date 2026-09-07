#!/usr/bin/env python3
"""把 RMUC2025 场地网格（STL）光栅化为二维占据栅格（.npz）。

用法：
    python3 tools/rasterize_field.py

输入：worlds/models/rmuc_2025/meshes/rmuc_2025.stl
输出：src/uav_planning/maps/rmuc_2025_occ.npz
      （occ[nx,ny] uint8 + x0/y0/res，NED 系，已含 0.5 m 膨胀）

原理：在三角面上按面积采样点云，保留 z ∈ [Z_MIN, Z_MAX]（飞行高度带内
可能撞到的结构），投影到 NED 平面栅格化，再按机体半径膨胀。
坐标关系与 worlds/rmuc_2025_field.sdf 中场地 yaw=+90° 一致：
    x_ned = mesh_x，y_ned = -mesh_y

依赖：仅 numpy。膨胀固定用纯 numpy 3x3 迭代——不依赖 scipy
（很多机器上 scipy 与 NumPy 2.x 不兼容会报 _ARRAY_API 错误），
且保证任何机器生成的地图完全一致（比 scipy 的十字结构元略保守，
向 8 连通方向膨胀，对避障更安全）。
"""

import os
import struct

import numpy as np

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STL = os.path.join(WS, 'worlds', 'models', 'rmuc_2025', 'meshes', 'rmuc_2025.stl')
OUT = os.path.join(WS, 'src', 'uav_planning', 'maps', 'rmuc_2025_occ.npz')

RES = 0.25          # 栅格分辨率 m
X0, X1 = -15.0, 15.0   # NED x（北）范围，略大于场地 29.16 m
Y0, Y1 = -8.5, 8.5     # NED y（东）范围，略大于场地 16.16 m
Z_MIN, Z_MAX = 0.35, 3.8  # 高度带：离地 0.35 m 以上、最高飞行层 +0.3 m
INFLATE = 0.5       # 机体膨胀半径 m


def load_stl(path):
    dt = np.dtype([('norm', '<f4', (3,)), ('v', '<f4', (3, 3)), ('attr', '<u2')])
    with open(path, 'rb') as f:
        f.read(80)
        n = struct.unpack('<I', f.read(4))[0]
    tris = np.fromfile(path, dtype=dt, offset=84, count=n)
    return tris['v'].astype(np.float64)


def dilate(grid, cells):
    """纯 numpy 膨胀：3x3 结构元（8 连通）× cells 轮。"""
    g = grid.copy()
    for _ in range(cells):
        p = np.pad(g, 1)
        g = np.zeros_like(g)
        for di in range(3):
            for dj in range(3):
                g |= p[di:di + g.shape[0], dj:dj + g.shape[1]]
    return g


def main():
    if not os.path.isfile(STL):
        raise SystemExit(f'未找到场地网格：{STL}\n请先运行 scripts/fetch_field_model.sh')
    np.random.seed(42)   # 固定采样种子，保证地图可复现
    v = load_stl(STL)
    print(f'三角面数: {len(v)}')

    # 只保留与高度带相交的三角面，再按面积采样
    zmin, zmax = v[:, :, 2].min(axis=1), v[:, :, 2].max(axis=1)
    tb = v[(zmax >= Z_MIN) & (zmin <= Z_MAX)]
    e1, e2 = tb[:, 1] - tb[:, 0], tb[:, 2] - tb[:, 0]
    area = 0.5 * np.linalg.norm(np.cross(e1, e2), axis=1)
    nsamp = np.clip((area / 0.01).astype(int), 1, 2000)   # ~每 10cm² 一个点
    idx = np.repeat(np.arange(len(tb)), nsamp)
    r1, r2 = np.random.rand(len(idx)), np.random.rand(len(idx))
    sq = np.sqrt(r1)
    a, b, c = 1 - sq, sq * (1 - r2), sq * r2
    pts = tb[idx, 0] * a[:, None] + tb[idx, 1] * b[:, None] + tb[idx, 2] * c[:, None]
    pts = pts[(pts[:, 2] >= Z_MIN) & (pts[:, 2] <= Z_MAX)]
    print(f'高度带 [{Z_MIN}, {Z_MAX}] m 内采样点: {len(pts)}')

    # NED 栅格化：x_ned = mesh_x，y_ned = -mesh_y
    nx, ny = int((X1 - X0) / RES), int((Y1 - Y0) / RES)
    grid = np.zeros((nx, ny), dtype=bool)
    ix = ((pts[:, 0] - X0) / RES).astype(int)
    iy = ((-pts[:, 1] - Y0) / RES).astype(int)
    ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    grid[ix[ok], iy[ok]] = True
    print(f'占据栅格（未膨胀）: {grid.sum()} / {nx * ny}')

    occ = dilate(grid, int(round(INFLATE / RES)))
    print(f'膨胀 {INFLATE} m 后: {occ.sum()}')

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    np.savez(OUT, occ=occ.astype(np.uint8), x0=X0, y0=Y0, res=RES,
             note='NED frame: x=north=mesh_x, y=east=-mesh_y; inflated 0.5m; z band 0.35-3.8m')
    print(f'已写出: {OUT}')


if __name__ == '__main__':
    main()
