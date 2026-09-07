"""
四机集群调度节点（地面站角色，对应框架文档 4.5 节）。

任务流程（全局 FSM）：
  WAIT_TAKEOFF  —— 等全部 UAV 进入 MISSION（起飞完成）
  SEARCH        —— 搜索区域按东向划分为 4 条带，各机割草机式搜索（高度分层防碰）
  CONVERGE      —— 任一机上报目标后，四机汇聚到目标上空（保持各自高度 + 水平错开）
  RETURN        —— 汇聚持续 converge_time 秒后各机返回出生点上空
  LAND          —— 到位后向各机下发 land 指令
  DONE

坐标：公共坐标系 = 各机本地 NED + 出生点偏移（spawn_offsets，与
scripts/start_sim_4uav.sh 的 PX4_GZ_MODEL_POSE 一一对应，注意
PX4 gz_bridge 的换算 NED=(enu_y, enu_x)）。下发给某机的航点会
自动减去该机的出生点偏移，转换到其本地系。
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                       QoSDurabilityPolicy, QoSHistoryPolicy)

from std_msgs.msg import String
from geometry_msgs.msg import Point
from px4_msgs.msg import VehicleLocalPosition
from uav_msgs.msg import DetectionArray


def px4_qos() -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )


class SwarmCoordinator(Node):
    def __init__(self):
        super().__init__('swarm_coordinator')

        # ---- 参数 ----
        self.declare_parameter('num_uavs', 4)
        self.declare_parameter('spawn_spacing', 2.0)        # 兼容旧的一字排开出生点
        # 各机出生点偏移（公共系 NED，扁平化 [x1,y1, x2,y2, ...]），
        # 与 scripts/start_sim_4uav.sh 的 SPAWN_POSES 对应；
        # 长度不足 2*num_uavs 时回退为 (0, (i-1)*spawn_spacing) 一字排开
        self.declare_parameter('spawn_offsets',
                               [9.4, 1.3, 9.4, -1.3, 11.6, 1.3, 11.6, -1.3])
        # 搜索区域（公共系 NED）：x∈[x0,x1], y∈[y0,y1]
        # 默认覆盖 RM2025 赛场中场至红方半场
        self.declare_parameter('search_area', [-10.0, -6.0, 8.0, 6.0])
        self.declare_parameter('lawnmower_step', 2.0)       # 割草机航线间距 m
        self.declare_parameter('base_alt', 2.0)             # 最低搜索高度 m
        self.declare_parameter('alt_layer', 0.5)            # 相邻机高度层差 m
        self.declare_parameter('converge_time', 15.0)       # 汇聚盘旋时间 s
        self.declare_parameter('wp_timeout', 30.0)          # 单航点超时 s（未到达也切下一个）

        self.n = int(self.get_parameter('num_uavs').value)
        self.spacing = float(self.get_parameter('spawn_spacing').value)
        self.area = list(self.get_parameter('search_area').value)
        self.step = float(self.get_parameter('lawnmower_step').value)
        self.base_alt = float(self.get_parameter('base_alt').value)
        self.alt_layer = float(self.get_parameter('alt_layer').value)
        self.converge_time = float(self.get_parameter('converge_time').value)
        self.wp_timeout = float(self.get_parameter('wp_timeout').value)

        # 出生点偏移（公共系 = 本地系 + 偏移）
        flat = list(self.get_parameter('spawn_offsets').value)
        if len(flat) >= 2 * self.n:
            self.spawn_offset = {i: (flat[2 * (i - 1)], flat[2 * (i - 1) + 1])
                                 for i in range(1, self.n + 1)}
        else:
            self.get_logger().warn('spawn_offsets 长度不足，回退为一字排开')
            self.spawn_offset = {i: (0.0, (i - 1) * self.spacing)
                                 for i in range(1, self.n + 1)}
        # 各机搜索高度（NED z，负值），逐层抬升
        self.uav_alt = {i: -(self.base_alt + (i - 1) * self.alt_layer)
                        for i in range(1, self.n + 1)}

        # ---- 接口 ----
        qos = px4_qos()
        self.wp_pubs = {}
        self.cmd_pubs = {}
        self.uav_state = {}       # offboard 节点状态
        self.uav_pos = {}         # 公共系位置 (x, y, z)
        for i in range(1, self.n + 1):
            self.wp_pubs[i] = self.create_publisher(Point, f'/uav{i}/waypoint', 10)
            self.cmd_pubs[i] = self.create_publisher(String, f'/uav{i}/command', 10)
            self.create_subscription(String, f'/uav{i}/state',
                                     self._make_state_cb(i), 10)
            self.create_subscription(
                VehicleLocalPosition, f'/px4_{i}/fmu/out/vehicle_local_position',
                self._make_pos_cb(i), qos)
            self.create_subscription(
                DetectionArray, f'/uav{i}/detections',
                self._make_det_cb(i), 10)
        self.pub_swarm_state = self.create_publisher(String, '/swarm/state', 10)

        # ---- FSM ----
        self.state = 'WAIT_TAKEOFF'
        self.wp_index = {i: 0 for i in range(1, self.n + 1)}
        self.wp_sent_time = {i: None for i in range(1, self.n + 1)}
        self.search_plans = self._build_search_plans()
        self.converge_start = None

        self.timer = self.create_timer(0.2, self._tick)  # 5 Hz 调度
        self.get_logger().info(
            f'集群调度就绪：{self.n} 机，搜索区域 {self.area}，'
            f'出生点 {[self.spawn_offset[i] for i in range(1, self.n + 1)]}')

    # ---------------- 回调 ----------------
    def _make_state_cb(self, i):
        def cb(msg: String):
            self.uav_state[i] = msg.data
        return cb

    def _make_pos_cb(self, i):
        def cb(msg: VehicleLocalPosition):
            ox, oy = self.spawn_offset[i]
            self.uav_pos[i] = (msg.x + ox, msg.y + oy, msg.z)
        return cb

    def _make_det_cb(self, i):
        def cb(msg: DetectionArray):
            if self.state != 'SEARCH':
                return
            for det in msg.detections:
                if det.label == 'target' and det.score > 0.5:
                    ox, oy = self.spawn_offset[i]
                    tx = det.position.x + ox
                    ty = det.position.y + oy
                    self.get_logger().info(
                        f'UAV{i} 发现目标！公共系位置 ({tx:.1f}, {ty:.1f})，集群汇聚')
                    self.target = (tx, ty)
                    self.state = 'CONVERGE'
                    self.converge_start = self.get_clock().now()
                    break
        return cb

    # ---------------- 搜索航线 ----------------
    def _build_search_plans(self):
        """把搜索区域按东向等分为 n 条带，每条带内生成割草机航点（公共系）。"""
        x0, y0, x1, y1 = self.area
        plans = {i: [] for i in range(1, self.n + 1)}
        band = (y1 - y0) / self.n
        for i in range(1, self.n + 1):
            by0 = y0 + (i - 1) * band
            by1 = by0 + band
            ys = []
            y = by0
            while y <= by1 + 1e-6:
                ys.append(y)
                y += self.step
            forward = True
            for y in ys:
                plans[i].append((x1, y) if forward else (x0, y))
                forward = not forward
        return plans

    def _pub_waypoint(self, i, wx, wy):
        """把公共系航点转为 i 号机本地系并下发。"""
        ox, oy = self.spawn_offset[i]
        p = Point()
        p.x = wx - ox
        p.y = wy - oy
        p.z = self.uav_alt[i]
        self.wp_pubs[i].publish(p)
        self.wp_sent_time[i] = self.get_clock().now()

    def _reached(self, i, wx, wy):
        if i not in self.uav_pos:
            return False
        x, y, z = self.uav_pos[i]
        d = math.sqrt((x - wx) ** 2 + (y - wy) ** 2 + (z - self.uav_alt[i]) ** 2)
        return d < 0.4

    def _wp_timed_out(self, i):
        t = self.wp_sent_time.get(i)
        if t is None:
            return True
        return (self.get_clock().now() - t).nanoseconds / 1e9 > self.wp_timeout

    # ---------------- 主循环 ----------------
    def _tick(self):
        if self.state == 'WAIT_TAKEOFF':
            if all(self.uav_state.get(i) == 'MISSION' for i in range(1, self.n + 1)):
                self.state = 'SEARCH'
                self.get_logger().info('全部起飞完成，开始分区搜索')

        elif self.state == 'SEARCH':
            for i in range(1, self.n + 1):
                plan = self.search_plans[i]
                idx = self.wp_index[i]
                if idx >= len(plan):
                    continue  # 该机搜索完毕，原地悬停
                wx, wy = plan[idx]
                if self.wp_sent_time[i] is None:
                    self._pub_waypoint(i, wx, wy)
                elif self._reached(i, wx, wy) or self._wp_timed_out(i):
                    self.wp_index[i] += 1
                    if self.wp_index[i] < len(plan):
                        self._pub_waypoint(i, *plan[self.wp_index[i]])
            # 全部搜完则返航
            if all(self.wp_index[i] >= len(self.search_plans[i])
                   for i in range(1, self.n + 1)):
                self.get_logger().info('搜索完毕未发现目标，返航')
                self.state = 'RETURN'

        elif self.state == 'CONVERGE':
            tx, ty = self.target
            # 各机在目标四周错开 1.5 m，保持各自高度层
            offsets = [(1.5, 0.0), (0.0, 1.5), (-1.5, 0.0), (0.0, -1.5)]
            for i in range(1, self.n + 1):
                dx, dy = offsets[(i - 1) % 4]
                self._pub_waypoint(i, tx + dx, ty + dy)
            if (self.get_clock().now() - self.converge_start).nanoseconds / 1e9 \
                    > self.converge_time:
                self.get_logger().info('汇聚完成，集群返航')
                self.state = 'RETURN'
                for i in range(1, self.n + 1):
                    self.wp_sent_time[i] = None

        elif self.state == 'RETURN':
            all_home = True
            for i in range(1, self.n + 1):
                hx, hy = self.spawn_offset[i]  # 出生点上空
                if self.wp_sent_time[i] is None:
                    self._pub_waypoint(i, hx, hy)
                    all_home = False
                elif not self._reached(i, hx, hy):
                    all_home = False
                    if self._wp_timed_out(i):
                        self._pub_waypoint(i, hx, hy)
            if all_home:
                self.state = 'LAND'
                self.get_logger().info('全部返航到位，开始降落')

        elif self.state == 'LAND':
            cmd = String()
            cmd.data = 'land'
            for i in range(1, self.n + 1):
                self.cmd_pubs[i].publish(cmd)
            if all(self.uav_state.get(i) in ('IDLE', 'DONE', 'LAND')
                   for i in range(1, self.n + 1)):
                self.state = 'DONE'
                self.get_logger().info('全部降落完成，任务结束')

        s = String()
        s.data = self.state
        self.pub_swarm_state.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = SwarmCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
