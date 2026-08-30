"""
机间防碰安全监视（集群级，运行一个实例即可）。

订阅全部 UAV 的本地位置（换算到公共系），两两计算 3D 距离：
  - 距离 < warn_dist：发布 /swarm/collision_warning 告警日志
  - 距离 < crit_dist：向双方下发"拉开"航点（沿连线反向各退 escape_dist）

仿真中靠高度分层（0.5 m 层差）基本不会触发；该节点是实机安全兜底，
也是后续接入 VFH 避障的挂载点。
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                       QoSDurabilityPolicy, QoSHistoryPolicy)

from std_msgs.msg import String
from geometry_msgs.msg import Point
from px4_msgs.msg import VehicleLocalPosition


def px4_qos() -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )


class CollisionMonitor(Node):
    def __init__(self):
        super().__init__('collision_monitor')

        self.declare_parameter('num_uavs', 4)
        self.declare_parameter('spawn_spacing', 2.0)
        self.declare_parameter('warn_dist', 1.5)
        self.declare_parameter('crit_dist', 0.8)
        self.declare_parameter('escape_dist', 1.0)

        self.n = int(self.get_parameter('num_uavs').value)
        spacing = float(self.get_parameter('spawn_spacing').value)
        self.warn_dist = float(self.get_parameter('warn_dist').value)
        self.crit_dist = float(self.get_parameter('crit_dist').value)
        self.escape_dist = float(self.get_parameter('escape_dist').value)
        self.spawn_offset = {i: (0.0, (i - 1) * spacing) for i in range(1, self.n + 1)}

        self.pos = {}
        qos = px4_qos()
        self.wp_pubs = {}
        for i in range(1, self.n + 1):
            self.create_subscription(
                VehicleLocalPosition, f'/px4_{i}/fmu/out/vehicle_local_position',
                self._make_cb(i), qos)
            self.wp_pubs[i] = self.create_publisher(Point, f'/uav{i}/waypoint', 10)
        self.pub_warn = self.create_publisher(String, '/swarm/collision_warning', 10)

        self.last_escape = {}
        self.create_timer(0.1, self._tick)  # 10 Hz
        self.get_logger().info(
            f'防碰监视就绪：{self.n} 机，告警 {self.warn_dist} m，临界 {self.crit_dist} m')

    def _make_cb(self, i):
        def cb(msg: VehicleLocalPosition):
            ox, oy = self.spawn_offset[i]
            self.pos[i] = (msg.x + ox, msg.y + oy, msg.z)
        return cb

    def _escape(self, i, j):
        """沿连线反向把两机拉开（在各自本地系下发航点）。"""
        now = self.get_clock().now()
        for k in (i, j):
            t = self.last_escape.get(k)
            if t is not None and (now - t).nanoseconds / 1e9 < 2.0:
                return  # 2 s 内不重复触发，避免与集群调度抢航点
        xi, yi, zi = self.pos[i]
        xj, yj, zj = self.pos[j]
        dx, dy = xi - xj, yi - yj
        d = max(math.hypot(dx, dy), 1e-3)
        ux, uy = dx / d, dy / d
        for k, sign in ((i, 1.0), (j, -1.0)):
            ox, oy = self.spawn_offset[k]
            x, y, z = self.pos[k]
            p = Point()
            p.x = x + sign * ux * self.escape_dist - ox
            p.y = y + sign * uy * self.escape_dist - oy
            p.z = z
            self.wp_pubs[k].publish(p)
            self.last_escape[k] = now

    def _tick(self):
        ids = sorted(self.pos.keys())
        for a in range(len(ids)):
            for b in range(a + 1, len(ids)):
                i, j = ids[a], ids[b]
                pi, pj = self.pos[i], self.pos[j]
                d = math.dist(pi, pj)
                if d < self.crit_dist:
                    msg = f'CRITICAL: UAV{i}<->UAV{j} 距离 {d:.2f} m，执行拉开'
                    self.get_logger().warn(msg)
                    self._escape(i, j)
                elif d < self.warn_dist:
                    msg = f'WARN: UAV{i}<->UAV{j} 距离 {d:.2f} m'
                    w = String()
                    w.data = msg
                    self.pub_warn.publish(w)


def main(args=None):
    rclpy.init(args=args)
    node = CollisionMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
