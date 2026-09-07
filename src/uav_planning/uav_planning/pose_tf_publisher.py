"""
无人机位姿 → TF + RViz 标记 桥接节点（集群一个实例即可）。

订阅全部 UAV 的 /px4_N/fmu/out/vehicle_local_position（本机 NED），
换算到公共系后：
  - 广播 TF：map -> uavN（x=北，y=东，z 上）
  - 发布 MarkerArray：每机一个机身方块 + 机头箭头，按机号着色

航向换算：PX4 NED heading 为"从北顺时针"，map 系（z 上）ROS yaw 为
"从 x 逆时针"，故 yaw_ros = -heading。
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                       QoSDurabilityPolicy, QoSHistoryPolicy)

from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from px4_msgs.msg import VehicleLocalPosition
from tf2_ros import TransformBroadcaster


def px4_qos() -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )


# 每机显示颜色 (r, g, b)
COLORS = {1: (0.9, 0.2, 0.2), 2: (0.2, 0.8, 0.3), 3: (0.2, 0.4, 0.95),
          4: (0.95, 0.8, 0.1)}


class PoseTfPublisher(Node):
    def __init__(self):
        super().__init__('pose_tf_publisher')

        self.declare_parameter('num_uavs', 4)
        self.declare_parameter('spawn_offsets',
                               [9.4, 1.3, 9.4, -1.3, 11.6, 1.3, 11.6, -1.3])
        n = int(self.get_parameter('num_uavs').value)
        flat = list(self.get_parameter('spawn_offsets').value)
        self.spawn_offset = {i: (flat[2 * (i - 1)], flat[2 * (i - 1) + 1])
                             for i in range(1, n + 1)
                             if len(flat) >= 2 * i}
        self.n = n
        self.pose = {}   # i -> (x, y, z_up, yaw_ros) 公共系

        for i in range(1, n + 1):
            self.create_subscription(
                VehicleLocalPosition, f'/px4_{i}/fmu/out/vehicle_local_position',
                self._make_cb(i), px4_qos())

        self.br = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(MarkerArray, '/uav_markers', 10)
        self.create_timer(0.1, self._tick)  # 10 Hz
        self.get_logger().info(f'TF/标记桥接就绪：{n} 机，map -> uavN')

    def _make_cb(self, i):
        def cb(msg: VehicleLocalPosition):
            ox, oy = self.spawn_offset.get(i, (0.0, 0.0))
            self.pose[i] = (msg.x + ox, msg.y + oy, -msg.z, -msg.heading)
        return cb

    def _tick(self):
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        for i, (x, y, z, yaw) in self.pose.items():
            # TF
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'map'
            t.child_frame_id = f'uav{i}'
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = math.sin(yaw / 2)
            t.transform.rotation.w = math.cos(yaw / 2)
            self.br.sendTransform(t)

            r, g, b = COLORS.get(i, (0.7, 0.7, 0.7))
            # 机身
            body = Marker()
            body.header.stamp = now
            body.header.frame_id = 'map'
            body.ns = 'uav_body'
            body.id = i
            body.type = Marker.CUBE
            body.action = Marker.ADD
            body.pose.position.x = x
            body.pose.position.y = y
            body.pose.position.z = z
            body.pose.orientation.z = math.sin(yaw / 2)
            body.pose.orientation.w = math.cos(yaw / 2)
            body.scale.x = 0.45
            body.scale.y = 0.45
            body.scale.z = 0.15
            body.color.r, body.color.g, body.color.b, body.color.a = r, g, b, 0.9
            markers.markers.append(body)
            # 机头箭头
            nose = Marker()
            nose.header = body.header
            nose.ns = 'uav_nose'
            nose.id = i
            nose.type = Marker.ARROW
            nose.action = Marker.ADD
            nose.pose = body.pose
            nose.scale.x = 0.6
            nose.scale.y = 0.08
            nose.scale.z = 0.08
            nose.color.r, nose.color.g, nose.color.b, nose.color.a = r, g, b, 1.0
            markers.markers.append(nose)
            # 机号文字
            text = Marker()
            text.header = body.header
            text.ns = 'uav_label'
            text.id = i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = z + 0.4
            text.pose.orientation.w = 1.0
            text.scale.z = 0.35
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = f'UAV{i}'
            markers.markers.append(text)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = PoseTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
