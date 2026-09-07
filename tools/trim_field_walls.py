#!/usr/bin/env python3
"""RMUC2025 场地网格削墙工具：去除围墙/顶棚防护网，并清理退化三角形。

用法：
    python3 tools/trim_field_walls.py [输入STL] [输出STL]
    不带参数时原地处理 worlds/models/rmuc_2025/meshes/rmuc_2025.stl
   （处理前自动备份为同目录 .bak，已处理过的网格再跑一遍是无害幂等操作）

处理规则（针对 RMUC2025 场地扫描网格，单位 m，网格坐标系）：
  1. 顶棚/防护网：三角形质心 z > Z_CEIL（默认 3.5）直接删除。
     场地内最高建筑约 3.2 m，该阈值不会误删场地结构。
  2. 围墙：边缘带（|x| > EDGE_X 或 |y| > EDGE_Y）内质心 z > Z_WALL
     （默认 0.3）的三角形删除——只删竖直墙体，保留边缘地面。
  3. 退化三角形：2*面积 < AREA2_MIN（默认 1e-8）删除。
     扫描网格里的零面积/碎屑三角形会诱发 ODE 三角网格碰撞构建断言，
     导致 gz-server 在模型落场接触地面时 SIGABRT（核心已转储）。
  4. 残余零法线按叉积重算。

依赖：仅 numpy。
"""

import os
import shutil
import sys

import numpy as np

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_STL = os.path.join(WS, 'worlds', 'models', 'rmuc_2025', 'meshes', 'rmuc_2025.stl')

Z_CEIL = 3.5        # 顶棚/防护网高度阈值
EDGE_X = 13.6       # 边缘带 x（场地半长 14.58）
EDGE_Y = 7.4        # 边缘带 y（场地半宽 8.08）
Z_WALL = 0.3        # 边缘带内墙体高度阈值
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


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_STL
    dst = sys.argv[2] if len(sys.argv) > 2 else src
    if not os.path.isfile(src):
        sys.exit(f'未找到输入网格：{src}')

    header, mesh = load_stl(src)
    n0 = len(mesh)
    v = mesh['v']
    centroid = v.mean(axis=1)
    print(f'读入 {n0} 个三角形：{src}')

    # 规则 1：顶棚/防护网
    m_ceil = centroid[:, 2] > Z_CEIL
    # 规则 2：边缘带围墙（保留边缘地面）
    edge = (np.abs(centroid[:, 0]) > EDGE_X) | (np.abs(centroid[:, 1]) > EDGE_Y)
    m_wall = edge & (centroid[:, 2] > Z_WALL)
    # 规则 3：退化三角形（ODE 碰撞断言隐患）
    e1 = v[:, 1] - v[:, 0]
    e2 = v[:, 2] - v[:, 0]
    cross = np.cross(e1, e2)
    area2 = np.linalg.norm(cross, axis=1)
    m_degen = area2 < AREA2_MIN

    drop = m_ceil | m_wall | m_degen
    print(f'删除：顶棚/防护网 {m_ceil.sum()}，围墙 {m_wall.sum()}，退化 {m_degen.sum()}'
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

    if dst == src and not os.path.isfile(src + '.bak'):
        shutil.copy2(src, src + '.bak')
        print(f'已备份原网格 -> {src}.bak')
    save_stl(dst, header, clean)
    print(f'写出 {len(clean)} 个三角形 -> {dst}')


if __name__ == '__main__':
    main()
