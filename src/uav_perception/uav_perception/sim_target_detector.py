"""
仿真目标检测器（每机一个实例，命名空间 uavN）。

作用：在仿真中替代真实的 YOLO 检测。当本机飞到目标附近且目标在"视场"内时，
以 detect_rate Hz 发布 DetectionArray（真值 + 高斯噪声），消息格式与实机
yolo_detector 完全一致，因此集群调度无需区分仿真/实机。

目标真值位置通过参数 target_world 给定（公共系 NED），与 Gazebo 世界中
放置的目标物对应；若要更真实，可改为订阅 Gazebo 的 /world/default/pose/info。
"""

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                       QoSDurabilityPolicy, QoSHistoryPolicy)

from px4_msgs.msg import VehicleLocalPosition
from uav_msgs.msg import Detection, DetectionArray


def px4_qos() -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )


class SimTargetDetector(Node):
    def __init__(self):
        super().__init__('sim_target_detector')

        self.declare_parameter('px4_ns', 'px4_1')
        self.declare_parameter('spawn_offset', [0.0, 0.0])   # 本机出生点偏移（公共系）
        self.declare_parameter('target_world', [16.0, 3.0, 0.0])  # 目标真值（公共系 NED）
        self.declare_parameter('detect_range', 8.0)          # 探测距离 m
        self.declare_parameter('detect_rate', 5.0)           # 发布频率 Hz
        self.declare_parameter('pos_noise', 0.3)             # 位置噪声 σ（m）

        self.px4_ns = self.get_parameter('px4_ns').value
        self.offset = list(self.get_parameter('spawn_offset').value)
        self.target = list(self.get_parameter('target_world').value)
        self.detect_range = float(self.get_parameter('detect_range').value)
        self.noise = float(self.get_parameter('pos_noise').value)

        self.local_pos = None
        self.create_subscription(
            VehicleLocalPosition, f'/{self.px4_ns}/fmu/out/vehicle_local_position',
            self._cb_pos, px4_qos())
        self.pub = self.create_publisher(DetectionArray, 'detections', 10)

        rate = float(self.get_parameter('detect_rate').value)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'仿真检测器就绪：目标 {self.target}，探测距离 {self.detect_range} m')

    def _cb_pos(self, msg: VehicleLocalPosition):
        self.local_pos = (msg.x, msg.y, msg.z)

    def _tick(self):
        if self.local_pos is None:
            return
        # 本机公共系位置
        wx = self.local_pos[0] + self.offset[0]
        wy = self.local_pos[1] + self.offset[1]
        wz = self.local_pos[2]
        dist = math.sqrt((wx - self.target[0]) ** 2 +
                         (wy - self.target[1]) ** 2 +
                         (wz - self.target[2]) ** 2)
        if dist > self.detect_range:
            return

        # 目标在本机本地系下的估计位置（真值 - 偏移 + 噪声）
        det = Detection()
        det.header.stamp = self.get_clock().now().to_msg()
        det.label = 'target'
        det.class_id = 0
        det.score = min(0.99, 0.6 + 0.4 * (1.0 - dist / self.detect_range))
        det.x_min, det.y_min, det.x_max, det.y_max = 120, 80, 200, 160
        det.position.x = self.target[0] - self.offset[0] + random.gauss(0, self.noise)
        det.position.y = self.target[1] - self.offset[1] + random.gauss(0, self.noise)
        det.position.z = self.target[2] + random.gauss(0, self.noise * 0.3)

        arr = DetectionArray()
        arr.header.stamp = det.header.stamp
        arr.detections.append(det)
        self.pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = SimTargetDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
