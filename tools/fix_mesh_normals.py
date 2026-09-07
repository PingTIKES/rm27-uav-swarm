#!/usr/bin/env python3
"""把 STL 网格"双面化"：为每个三角面追加一个顶点序反转、法向取反的副本。

背景：RMUC2025 场地网格（rmu_gazebo_simulator 的 rmuc_2025.stl）中，
比赛地板等大面片的法向朝下，Gazebo(ogre2) 默认背面剔除，从上方看
地板不可见（"场地只剩半边/碎片"）。最稳妥的修法是把网格双面化——
任一方向看都有朝向相机的面片。碰撞也随之天然双面，无副作用。

只用 Python 标准库（不依赖 numpy），避免目标机器缺包导致修复静默失败。

用法：
    python3 tools/fix_mesh_normals.py [输入stl [输出stl]]
缺省：输入=输出=worlds/models/rmuc_2025/meshes/rmuc_2025.stl（原地修复，
已是双面网格时自动跳过）。输出前会把原文件备份为 .single_sided.bak。
"""

import os
import struct
import sys

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT = os.path.join(WS, 'worlds', 'models', 'rmuc_2025', 'meshes', 'rmuc_2025.stl')

# 二进制 STL 单条记录：法向 3×f4 + 3 顶点 9×f4 + 属性 u2 = 50 字节
REC = struct.Struct('<12fH')
REC_SIZE = REC.size  # 50

ORIG_TRIS = 114234          # rmu 原版三角面数（用于幂等判断）
DOUBLE_TRIS = ORIG_TRIS * 2


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else DEFAULT
    dst = sys.argv[2] if len(sys.argv) > 2 else src
    with open(src, 'rb') as f:
        header = f.read(80)
        (n,) = struct.unpack('<I', f.read(4))
        body = f.read()
    if len(body) < n * REC_SIZE:
        print(f'错误：文件长度不足（{len(body)} 字节 < {n} 面 x {REC_SIZE}），'
              f'文件可能损坏：{src}')
        sys.exit(1)

    if n == DOUBLE_TRIS:
        print(f'已是双面网格（{n} 面），跳过：{src}')
        return
    if n != ORIG_TRIS:
        print(f'警告：面数 {n} 既不是原版 {ORIG_TRIS} 也不是双面版 {DOUBLE_TRIS}，仍将处理')

    out = bytearray(n * 2 * REC_SIZE)
    for k in range(n):
        rec = body[k * REC_SIZE:(k + 1) * REC_SIZE]
        vals = REC.unpack(rec)
        nx, ny, nz = vals[0:3]
        v = vals[3:12]                      # v0=(0:3) v1=(3:6) v2=(6:9)
        attr = vals[12]
        out[(2 * k) * REC_SIZE:(2 * k + 1) * REC_SIZE] = rec
        # 顶点序反转 v2,v1,v0 => 绕向取反；法向取反
        out[(2 * k + 1) * REC_SIZE:(2 * k + 2) * REC_SIZE] = REC.pack(
            -nx, -ny, -nz,
            v[6], v[7], v[8], v[3], v[4], v[5], v[0], v[1], v[2],
            attr)

    if os.path.abspath(src) == os.path.abspath(dst) \
            and not os.path.exists(src + '.single_sided.bak'):
        os.rename(src, src + '.single_sided.bak')
    with open(dst, 'wb') as f:
        f.write(header)
        f.write(struct.pack('<I', n * 2))
        f.write(out)
    print(f'双面化完成：{n} -> {n * 2} 面，已写出 {dst}'
          + (f'（原文件备份 {src}.single_sided.bak）'
             if os.path.exists(src + '.single_sided.bak') else ''))


if __name__ == '__main__':
    main()
