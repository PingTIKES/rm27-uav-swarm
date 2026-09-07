#!/usr/bin/env python3
"""RMUC2025 场地网格处理工具：削围墙/顶棚防护网 + 清理退化三角形 + 双面化。

用法：
    python3 tools/trim_field_walls.py [输入STL] [输出STL]
    不带参数时原地处理 worlds/models/rmuc_2025/meshes/rmuc_2025.stl
   （处理前自动备份为同目录 .bak；对处理过的网格再跑是幂等的）

处理规则（针对 RMUC2025 场地扫描网格，单位 m，网格坐标系）：
  1. 顶棚防护网：三角形质心 z > 3.5——网索只有 40 个三角形
    （z≈6.1/12.3/18.6 三层），该规则精确且无命中场地结构。
  2. 周边围栏：几何法线近竖直（|nz| < 0.7）、质心 0.3 < z <= 3.5
     且距场地边界（|x|=14.58, |y|=8.08）不足 EDGE_D（默认 0.75 m）。
     该阈值经校准：在原始网格上零误删场地结构，
     在已削墙成品网格上零命中（幂等）。
     注意：距边界 0.75 m 以外的围栏拐角弧段（约 489 个三角形）本规则
     保留——zip 发行包内的成品网格是手工完全削净的标准版（推荐），
     本工具用于从原始网格快速得到一个安全可用的场地。
  3. 退化三角形：2*面积 < 1e-8 删除（原始网格含 402 个，双面化后）。
     这类碎屑会诱发 ODE 三角网格碰撞构建断言，导致 gz-server
     在模型落场接触地面时 SIGABRT（"已中止 (核心已转储)"）。
  4. 残余零法线按叉积重算。
  5. 双面化：扫描网格是单面的，Gazebo 背面剔除会让部分面从内侧/
     上方消失；已为双面（>=95% 三角形存在重合反向副本）时自动跳过。

依赖：仅 numpy。
"""

import os
import shutil
import sys

import numpy as np

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_STL = os.path.join(WS, 'worlds', 'models', 'rmuc_2025', 'meshes', 'rmuc_2025.stl')

Z_CEIL = 3.5        # 顶棚防护网高度阈值
Z_WALL_LO = 0.3     # 围栏高度下限（保留地面）
FIELD_X = 14.58185  # 场地半长
FIELD_Y = 8.07789   # 场地半宽
EDGE_D = 0.75       # 围栏距边界阈值（校准值：原始网格零误删）
NZ_VERT = 0.7       # 近竖直判定（几何法线 |nz| 上限）
AREA2_MIN = 1e-8    # 退化三角形阈值（2*面积，m^2）

DT = np.dtype([('norm', '<f4', (3,)), ('v', '<f4', (3, 3)), ('attr', '<u2')])


def load_stl(path):
    with open(path, 'rb') as f:
        header = f.read(80)
        n = np.frombuffer(f.read(4), dtype='<u4')[0]
        mesh = np.frombuffer(f.read(), dtype=DT).copy()
    assert len(mesh) == n, f'STL 声明 {n} 个三角形，实际读到 {len(mesh)} 个'
    return header, mesh


def save_stl(path, header, mesh):
    with open(path, 'wb') as f:
        f.write(header)
        f.write(np.uint32(len(mesh)).tobytes())
        mesh.tofile(f)


def geom_nz(v):
    """几何法线的 |nz|（叉积，不依赖扫描件里不可靠的存储法线）。"""
    g = np.cross(v[:, 1] - v[:, 0], v[:, 2] - v[:, 0])
    return np.abs(g[:, 2]) / np.maximum(np.linalg.norm(g, axis=1), 1e-30), g


def is_double_sided(v):
    """>=95% 三角形存在重合反向副本则视为已双面化。"""
    keys = np.sort(np.round(v.reshape(len(v), -1), 4), axis=1)
    _, counts = np.unique(keys, axis=0, return_counts=True)
    return (counts == 2).mean() >= 0.95


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_STL
    dst = sys.argv[2] if len(sys.argv) > 2 else src
    if not os.path.isfile(src):
        sys.exit(f'未找到输入网格：{src}')

    header, mesh = load_stl(src)
    v = mesh['v']
    centroid = v.mean(axis=1)
    gnz, gvec = geom_nz(v)
    area2 = np.linalg.norm(gvec, axis=1)
    print(f'读入 {len(mesh)} 个三角形：{src}')

    # 规则 1：顶棚防护网
    m_ceil = centroid[:, 2] > Z_CEIL
    # 规则 2：周边围栏（近竖直 + 贴边 + 离地面）
    d_edge = np.minimum(FIELD_X - np.abs(centroid[:, 0]),
                        FIELD_Y - np.abs(centroid[:, 1]))
    m_wall = (gnz < NZ_VERT) & (centroid[:, 2] > Z_WALL_LO) \
        & (centroid[:, 2] <= Z_CEIL) & (d_edge < EDGE_D)
    # 规则 3：退化三角形（ODE 碰撞断言隐患）
    m_degen = area2 < AREA2_MIN

    drop = m_ceil | m_wall | m_degen
    print(f'删除：防护网 {m_ceil.sum()}，围栏 {m_wall.sum()}，退化 {m_degen.sum()}'
          f'（去重后共 {drop.sum()}）')
    clean = mesh[~drop]

    # 规则 4：零法线重算
    norm = clean['norm']
    bad = np.linalg.norm(norm, axis=1) < 1e-6
    if bad.any():
        c = np.cross(clean['v'][bad, 1] - clean['v'][bad, 0],
                     clean['v'][bad, 2] - clean['v'][bad, 0])
        clean['norm'][bad] = c / np.linalg.norm(c, axis=1, keepdims=True)
        print(f'重算零法线 {bad.sum()} 条')

    # 规则 5：双面化（已双面则跳过，保证幂等）
    if is_double_sided(clean['v']):
        print('网格已是双面，跳过双面化')
    else:
        flip = clean.copy()
        flip['v'] = clean['v'][:, ::-1]          # 反转绕向
        flip['norm'] = -clean['norm']            # 反转法线
        clean = np.concatenate([clean, flip])
        print(f'双面化：三角形数翻倍至 {len(clean)}')

    if dst == src and not os.path.isfile(src + '.bak'):
        shutil.copy2(src, src + '.bak')
        print(f'已备份原网格 -> {src}.bak')
    save_stl(dst, header, clean)
    print(f'写出 {len(clean)} 个三角形 -> {dst}')


if __name__ == '__main__':
    main()
