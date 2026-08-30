"""
VFH+ 避障规划器（实机模板，对应框架文档 uav_planning 模块）。

仿真中障碍物由高度分层规避，本节点默认不参与仿真。
实机接入方式：
  订阅：obstacles（sensor_msgs/PointCloud2，来自 stereo_depth_node，320x240@5Hz）
        waypoint_in（geometry_msgs/Point，集群调度下发的原始航点）
  发布：waypoint（geometry_msgs/Point，避障修正后的航点，接 uav_control）

当前实现为"直通 + 简单斥力修正"：把点云按方位分 bin，统计每个 bin 的
最近障碍距离，对原始航点方向施加斥力偏转。可按需升级为完整 VFH+
（直方图阈值化 -> 候选谷 -> 代价函数选向）。
"""

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2


class VfhPlanner(Node):
    def __init__(self):
        super().__init__('vfh_planner')

        self.declare_parameter('num_bins', 36)        # 方位分 bin 数（每 bin 10°）
        self.declare_parameter('safe_dist', 1.5)      # 安全距离 m
        self.declare_parameter('max_repulse', 1.0)    # 最大斥力偏转 m

        self.num_bins = int(self.get_parameter('num_bins').value)
        self.safe_dist = float(self.get_parameter('safe_dist').value)
        self.max_repulse = float(self.get_parameter('max_repulse').value)

        self.create_subscription(PointCloud2, 'obstacles', self._cb_cloud, 10)
        self.create_subscription(Point, 'waypoint_in', self._cb_wp, 10)
        self.pub = self.create_publisher(Point, 'waypoint', 10)

        self.bin_min = [float('inf')] * self.num_bins
        self.get_logger().warn('VFH 规划器为模板实现（直通 + 斥力修正），实机请完善')

    def _cb_cloud(self, msg: PointCloud2):
        """点云 -> 方位直方图。实机用 sensor_msgs_py.point_cloud2.read_points。"""
        # 桩实现：实机替换为
        #   from sensor_msgs_py import point_cloud2
        #   for x, y, z in point_cloud2.read_points(msg, field_names=('x','y','z')):
        #       b = int((math.atan2(y, x) + math.pi) / (2*math.pi) * self.num_bins)
        #       self.bin_min[b] = min(self.bin_min[b], math.hypot(x, y))
        pass

    def _cb_wp(self, msg: Point):
        """对原始航点施加斥力修正后转发。"""
        out = Point()
        out.x, out.y, out.z = msg.x, msg.y, msg.z
        heading = math.atan2(msg.y, msg.x)
        b = int((heading + math.pi) / (2 * math.pi) * self.num_bins) % self.num_bins
        if self.bin_min[b] < self.safe_dist:
            # 前方有障碍：向垂直方向偏转
            shift = self.max_repulse * (1.0 - self.bin_min[b] / self.safe_dist)
            out.x += -math.sin(heading) * shift
            out.y += math.cos(heading) * shift
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = VfhPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
