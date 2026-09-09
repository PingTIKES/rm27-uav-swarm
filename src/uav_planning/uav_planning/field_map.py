"""
RM2027 赛场占据栅格地图 + A* 路径搜索（公共系 NED 坐标）。

地图来源（二选一）：
  1. map_file 指向离线栅格（.npz）——由 RMUC2027 场地网格
     （worlds/models/rmuc_2027：2025 网格推平中央结构 + 战场中央机库）
     离线光栅化生成：取 z∈[0.35, 3.8] m 的所有三角面采样投影到 NED 平面，
     并按机体半径膨胀 0.5 m。对应 worlds/rmuc_2027_field.sdf（默认场地）。
     机库平台顶 0.24 m 低于高度带下限，不在栅格中（可飞越、可降落）。
  2. 内置解析障碍清单 OBSTACLES——与简化场地 worlds/rm2025_field.sdf
     一一对应（map_file 为空或文件缺失时的回退）。

约定：
  - 坐标均为公共系 NED（x 北 / y 东），单位 m
  - 栅格 cell 值：0=空闲 1=占据（含膨胀）
  - 搜索目标柱 target_marker 故意不算障碍（否则目标点永远不可达；
    它高 1.0 m，低于最低飞行层 2.0 m，从上方掠过无碰撞）

实机演进：本模块接口保持不变，障碍物来源从"写死的场地几何"换成
D430i 深度点云局部建图（或裁判系统给的静态场地 + 局部动态障碍）。
"""

import heapq
import math
import os


class FieldMap:
    # 公共系 NED 边界（围挡内缘）
    X_MIN, X_MAX = -13.8, 13.8      # 南北向（长轴）
    Y_MIN, Y_MAX = -7.3, 7.3        # 东西向（短轴）

    # 障碍矩形：(中心x, 中心y, x向尺寸, y向尺寸)，NED
    OBSTACLES = [
        (13.0, 0.0, 1.8, 2.2),      # 蓝方基地
        (-13.0, 0.0, 1.8, 2.2),     # 红方基地
        (0.0, 4.8, 1.0, 1.0),       # 前哨站（东）
        (0.0, -4.8, 1.0, 1.0),      # 前哨站（西）
        (0.0, 0.0, 3.2, 3.2),       # 中央资源岛（含能量机关架）
        (6.5, 5.0, 3.0, 2.4),       # 蓝方高地
        (-6.5, -5.0, 3.0, 2.4),     # 红方高地
        (4.2, 5.0, 0.8, 2.4),       # 二级台阶 1
        (4.6, 5.0, 0.8, 2.4),       # 二级台阶 2
    ]

    def __init__(self, resolution=0.25, inflate=0.5, map_file=''):
        self.res = float(resolution)
        self.inflate = float(inflate)
        self._from_file = False
        if map_file and os.path.isfile(map_file):
            try:
                self._load_npz(map_file)
                self._from_file = True
            except Exception as exc:  # noqa: BLE001 - 缺 numpy / 文件损坏时回退
                print(f'[FieldMap] 加载 {map_file} 失败（{exc}），回退到内置解析障碍')
        if not self._from_file:
            self.nx = int((self.X_MAX - self.X_MIN) / self.res)
            self.ny = int((self.Y_MAX - self.Y_MIN) / self.res)
            self.grid = self._build()

    # ---------------- 离线栅格（真实场地网格光栅化产物） ----------------
    def _load_npz(self, path):
        """加载 .npz 离线栅格：occ(nx,ny) uint8（已含膨胀）、x0/y0/res。

        栅格在 NED 系下生成：x_ned = mesh_x，y_ned = -mesh_y，
        与 worlds/rmuc_2027_field.sdf（及 rmuc_2025_field.sdf）中
        场地 yaw=+90° 的放置一致。
        """
        import numpy as np
        data = np.load(path)
        occ = data['occ']
        self.X_MIN = float(data['x0'])
        self.Y_MIN = float(data['y0'])
        self.res = float(data['res'])
        self.nx, self.ny = int(occ.shape[0]), int(occ.shape[1])
        self.X_MAX = self.X_MIN + self.nx * self.res
        self.Y_MAX = self.Y_MIN + self.ny * self.res
        self.grid = [[1 if occ[i, j] else 0 for j in range(self.ny)]
                     for i in range(self.nx)]

    # ---------------- 建图 ----------------
    def _build(self):
        g = [[0] * self.ny for _ in range(self.nx)]
        for cx, cy, sx, sy in self.OBSTACLES:
            hx = sx / 2 + self.inflate
            hy = sy / 2 + self.inflate
            i0 = max(0, self._x2i(cx - hx))
            i1 = min(self.nx - 1, self._x2i(cx + hx))
            j0 = max(0, self._y2j(cy - hy))
            j1 = min(self.ny - 1, self._y2j(cy + hy))
            for i in range(i0, i1 + 1):
                for j in range(j0, j1 + 1):
                    g[i][j] = 1
        return g

    def _x2i(self, x):
        return int((x - self.X_MIN) / self.res)

    def _y2j(self, y):
        return int((y - self.Y_MIN) / self.res)

    def _i2x(self, i):
        return self.X_MIN + (i + 0.5) * self.res

    def _j2y(self, j):
        return self.Y_MIN + (j + 0.5) * self.res

    def in_bounds(self, i, j):
        return 0 <= i < self.nx and 0 <= j < self.ny

    def occupied(self, x, y):
        i, j = self._x2i(x), self._y2j(y)
        if not self.in_bounds(i, j):
            return True
        return self.grid[i][j] == 1

    def nearest_free(self, x, y):
        """给定点落在障碍里时，螺旋向外找最近的空闲栅格。"""
        i0, j0 = self._x2i(x), self._y2j(y)
        i0 = min(max(i0, 0), self.nx - 1)
        j0 = min(max(j0, 0), self.ny - 1)
        if self.grid[i0][j0] == 0:
            return self._i2x(i0), self._j2y(j0)
        for r in range(1, max(self.nx, self.ny)):
            for di in range(-r, r + 1):
                for dj in (-r, r):
                    for i, j in ((i0 + di, j0 + dj), (i0 + dj, j0 + di)):
                        if self.in_bounds(i, j) and self.grid[i][j] == 0:
                            return self._i2x(i), self._j2y(j)
        return x, y  # 找不到就原样返回（调用方应视为失败）

    # ---------------- A* ----------------
    def astar(self, start, goal):
        """8 连通 A*，返回 [(x,y), ...] 公共系 NED 路径；失败返回 None。"""
        sx, sy = self.nearest_free(*start)
        gx, gy = self.nearest_free(*goal)
        si, sj = self._x2i(sx), self._y2j(sy)
        gi, gj = self._x2i(gx), self._y2j(gy)

        def h(i, j):
            dx, dy = abs(i - gi), abs(j - gj)
            return max(dx, dy) + 0.4142 * min(dx, dy)  # octile

        DIRS = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]
        openq = [(h(si, sj), 0.0, si, sj)]
        came = {}
        gscore = {(si, sj): 0.0}
        closed = set()
        while openq:
            _, g, i, j = heapq.heappop(openq)
            if (i, j) in closed:
                continue
            if (i, j) == (gi, gj):
                path = [(i, j)]
                while path[-1] in came:
                    path.append(came[path[-1]])
                path.reverse()
                pts = [(self._i2x(a), self._j2y(b)) for a, b in path]
                pts[-1] = (gx, gy)
                return pts
            closed.add((i, j))
            for di, dj, w in DIRS:
                ni, nj = i + di, j + dj
                if not self.in_bounds(ni, nj) or self.grid[ni][nj] == 1 \
                        or (ni, nj) in closed:
                    continue
                # 禁止斜穿障碍角
                if di != 0 and dj != 0 and \
                        (self.grid[i + di][j] == 1 or self.grid[i][j + dj] == 1):
                    continue
                ng = g + w
                if ng < gscore.get((ni, nj), math.inf):
                    gscore[(ni, nj)] = ng
                    came[(ni, nj)] = (i, j)
                    heapq.heappush(openq, (ng + h(ni, nj), ng, ni, nj))
        return None

    # ---------------- 视线拉直平滑 ----------------
    def line_free(self, p0, p1):
        """两点间直线无碰撞（按 res/2 步长采样）。"""
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        dist = math.hypot(dx, dy)
        steps = max(1, int(dist / (self.res / 2)))
        for k in range(steps + 1):
            t = k / steps
            if self.occupied(p0[0] + t * dx, p0[1] + t * dy):
                return False
        return True

    def smooth(self, path):
        """贪心捷径：能直走就删掉中间点。

        警告：不要在此之后做任何"按距离稀疏化"的二次删点——捷径只保证
        相邻保留点之间直线无碰撞，再删点会产生未经检查的新段，
        可能斜穿障碍角（真机即撞墙）。
        """
        if not path or len(path) < 3:
            return path
        out = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1 and not self.line_free(path[i], path[j]):
                j -= 1
            out.append(path[j])
            i = j
        return out

    # ---------------- 导出 ----------------
    def to_occupancy_grid(self):
        """返回 nav_msgs/OccupancyGrid（调用方填 header）。

        注意数据约定：行优先存储、行 = y（data[y*width + x]），
        因此 width=nx、height=ny，外层循环必须是 y——写反会导致
        RViz 里地图转置 90°，与 TF/标记坐标对不上。
        """
        from nav_msgs.msg import OccupancyGrid
        grid = OccupancyGrid()
        grid.info.resolution = self.res
        grid.info.width = self.nx          # 列数 = NED x 方向栅格数
        grid.info.height = self.ny         # 行数 = NED y 方向栅格数
        grid.info.origin.position.x = self.X_MIN
        grid.info.origin.position.y = self.Y_MIN
        grid.info.origin.orientation.w = 1.0
        data = []
        for j in range(self.ny):           # 行（y）外层
            for i in range(self.nx):       # 列（x）内层
                data.append(100 if self.grid[i][j] else 0)
        grid.data = data
        return grid
