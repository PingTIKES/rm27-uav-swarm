"""
RViz 打点导航节点：订阅 RViz "2D Nav Goal" 工具发布的 /goal_pose，
在 RM2025 赛场占据栅格上跑 A* + 视线拉直，把路径拆成航点序列
依次下发给指定无人机的 offboard 节点（/uavN/waypoint）。

话题：
  订阅  /goal_pose                     geometry_msgs/PoseStamped（RViz 打点，map 系）
  订阅  /px4_N/fmu/out/vehicle_local_position（本机位置，换算公共系）
  发布  /uavN/waypoint                 geometry_msgs/Point（本机 NED，z 负为向上）
  发布  /planned_path                  nav_msgs/Path（RViz 显示）
  发布  /field_map                     nav_msgs/OccupancyGrid（latched，RViz 地图）
  发布  /goal_marker                   visualization_msgs/Marker（目标点标记）
  发布  /goal_planner/state            std_msgs/String（状态机）

坐标约定：RViz map 系 = 公共系（x=北，y=东，z 上），与 swarm_coordinator
的公共 NED x/y 完全一致，z 取反显示。spawn_offsets 必须与
scripts/start_sim_4uav.sh 一致。

注意：本节点与 swarm_coordinator 都会给 /uavN/waypoint 发航点，
不要同时运行（二选一）。
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                       QoSDurabilityPolicy, QoSHistoryPolicy)

from std_msgs.msg import String
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from visualization_msgs.msg import Marker
from px4_msgs.msg import VehicleLocalPosition

from uav_planning.field_map import FieldMap


def px4_qos() -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )


class GoalPlanner(Node):
    def __init__(self):
        super().__init__('goal_planner')

        # ---- 参数 ----
        self.declare_parameter('uav_id', 1)                 # 打点控制哪台机
        self.declare_parameter('num_uavs', 4)
        self.declare_parameter('spawn_offsets',
                               [9.4, 1.3, 9.4, -1.3, 11.6, 1.3, 11.6, -1.3])
        self.declare_parameter('cruise_alt', 2.0)           # 巡航高度 m
        self.declare_parameter('map_resolution', 0.25)
        self.declare_parameter('inflate', 0.5)              # 障碍膨胀 m
        self.declare_parameter('reach_tol', 0.45)           # 航点到达判定 m

        self.uav_id = int(self.get_parameter('uav_id').value)
        n = int(self.get_parameter('num_uavs').value)
        flat = list(self.get_parameter('spawn_offsets').value)
        self.spawn_offset = {i: (flat[2 * (i - 1)], flat[2 * (i - 1) + 1])
                             for i in range(1, n + 1)
                             if len(flat) >= 2 * i}
        self.alt = float(self.get_parameter('cruise_alt').value)
        self.reach_tol = float(self.get_parameter('reach_tol').value)

        self.fmap = FieldMap(
            resolution=float(self.get_parameter('map_resolution').value),
            inflate=float(self.get_parameter('inflate').value))

        # ---- 状态 ----
        self.pos = None            # 公共系 (x, y, z_ned)
        self.waypoints = []        # 待执行航点（公共系）
        self.state = 'IDLE'

        # ---- 接口 ----
        uid = self.uav_id
        self.create_subscription(
            PoseStamped, '/goal_pose', self._cb_goal, 10)
        self.create_subscription(
            VehicleLocalPosition, f'/px4_{uid}/fmu/out/vehicle_local_position',
            self._cb_pos, px4_qos())
        self.wp_pub = self.create_publisher(Point, f'/uav{uid}/waypoint', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.marker_pub = self.create_publisher(Marker, '/goal_marker', 10)
        self.state_pub = self.create_publisher(String, '/goal_planner/state', 10)

        # 场地地图（latched：reliable + transient_local，RViz Map 直接显示）
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.map_pub = self.create_publisher(OccupancyGrid, '/field_map', map_qos)
        self._publish_field_map()

        self.create_timer(0.2, self._tick)  # 5 Hz
        self.get_logger().info(
            f'打点导航就绪：控制 uav{uid}，巡航高度 {self.alt} m，'
            f'在 RViz 里用 "2D Nav Goal" 工具点目标')

    # ---------------- 回调 ----------------
    def _cb_pos(self, msg: VehicleLocalPosition):
        ox, oy = self.spawn_offset.get(self.uav_id, (0.0, 0.0))
        self.pos = (msg.x + ox, msg.y + oy, msg.z)

    def _cb_goal(self, msg: PoseStamped):
        gx, gy = msg.pose.position.x, msg.pose.position.y
        if self.pos is None:
            self.get_logger().warn('尚未收到本机位置，无法规划')
            return
        if self.fmap.occupied(gx, gy):
            gx, gy = self.fmap.nearest_free(gx, gy)
            self.get_logger().warn(f'目标在障碍内，已吸附到最近空闲点 ({gx:.1f}, {gy:.1f})')
        start = (self.pos[0], self.pos[1])
        raw = self.fmap.astar(start, (gx, gy))
        if raw is None:
            self.state = 'NO_PATH'
            self.get_logger().warn(f'A* 失败：({start[0]:.1f},{start[1]:.1f}) -> ({gx:.1f},{gy:.1f})')
            return
        self.waypoints = self.fmap.smooth(raw)
        self.state = 'EXECUTING'
        self.get_logger().info(
            f'收到目标 ({gx:.1f}, {gy:.1f})，路径 {len(self.waypoints)} 个航点')
        self._publish_path()
        self._publish_goal_marker(gx, gy)

    # ---------------- 发布 ----------------
    def _publish_field_map(self):
        grid = self.fmap.to_occupancy_grid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'map'
        self.map_pub.publish(grid)

    def _publish_path(self):
        p = Path()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = 'map'
        for x, y in self.waypoints:
            ps = PoseStamped()
            ps.header = p.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = self.alt
            ps.pose.orientation.w = 1.0
            p.poses.append(ps)
        self.path_pub.publish(p)

    def _publish_goal_marker(self, x, y):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'goal'
        m.id = 0
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = self.alt / 2
        m.pose.orientation.w = 1.0
        m.scale.x = 0.3
        m.scale.y = 0.3
        m.scale.z = self.alt
        m.color.r = 0.1
        m.color.g = 0.9
        m.color.b = 0.2
        m.color.a = 0.8
        self.marker_pub.publish(m)

    # ---------------- 主循环 ----------------
    def _tick(self):
        if self.state != 'EXECUTING' or self.pos is None:
            self._pub_state()
            return
        if not self.waypoints:
            self.state = 'REACHED'
            self.get_logger().info('已到达目标点')
            self._pub_state()
            return
        wx, wy = self.waypoints[0]
        ox, oy = self.spawn_offset.get(self.uav_id, (0.0, 0.0))
        p = Point()
        p.x = wx - ox          # 公共系 → 本机 NED
        p.y = wy - oy
        p.z = -self.alt
        self.wp_pub.publish(p)
        # 到达判定（含高度）
        d = math.sqrt((self.pos[0] - wx) ** 2 + (self.pos[1] - wy) ** 2
                      + (self.pos[2] + self.alt) ** 2)
        if d < self.reach_tol:
            self.waypoints.pop(0)
        self._pub_state()

    def _pub_state(self):
        s = String()
        s.data = self.state
        self.state_pub.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = GoalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
