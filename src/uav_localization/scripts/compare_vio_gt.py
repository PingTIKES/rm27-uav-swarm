#!/usr/bin/env python3
"""VIO 精度评估：OpenVINS 里程计 vs 参考轨迹（PX4 SITL 本地位置，仿真中≈真值）。

订阅：
  /<uav_ns>/odomimu                          nav_msgs/Odometry（OpenVINS 输出，z-up）
  /px4_<uav_id>/fmu/out/vehicle_local_position  px4_msgs VehicleLocalPosition（NED）

两组轨迹时间基准不同（gz 仿真时间 vs PX4 时钟），故按消息到达时刻就近配对；
坐标系不同（OpenVINS 起点系、任意航向），先用前 N 对已运动的样本做
最小二乘对齐（2D 旋转 + 3D 平移），再统计误差。

每 report_period 秒打印一次：
  当前误差 | 累积 RMSE(水平/3D) | 参考轨迹里程 | 漂移百分比（水平RMSE/里程）

用法：
  ros2 run uav_localization compare_vio_gt.py --ros-args -p uav_id:=1
注意：先让 OpenVINS 完成初始化并起飞一段距离，对齐才有效（<3m 里程时只对齐平移）。
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                       QoSDurabilityPolicy, QoSHistoryPolicy)

from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition


def px4_qos() -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )


class VioEval(Node):
    def __init__(self):
        super().__init__('compare_vio_gt')
        self.declare_parameter('uav_id', 1)
        self.declare_parameter('report_period', 5.0)   # 打印周期 s
        self.declare_parameter('match_tol', 0.1)       # 配对最大到达时间差 s

        uid = int(self.get_parameter('uav_id').value)
        self.ns = f'uav{uid}'
        self.match_tol = float(self.get_parameter('match_tol').value)
        period = float(self.get_parameter('report_period').value)

        self.vio = []   # (t_arrival, x, y, z)  OpenVINS 系
        self.ref = []   # (t_arrival, x, y, z)  已转 z-up ENU 的参考系
        self.aligned = None  # (yaw, tx, ty, tz)

        self.create_subscription(
            Odometry, f'/{self.ns}/odomimu', self._cb_vio, 50)
        self.create_subscription(
            VehicleLocalPosition,
            f'/px4_{uid}/fmu/out/vehicle_local_position',
            self._cb_ref, px4_qos())
        self.create_timer(period, self._report)
        self.get_logger().info(
            f'VIO 评估就绪：/{self.ns}/odomimu vs /px4_{uid} vehicle_local_position')

    # ---------------- 采样 ----------------
    def _cb_vio(self, msg: Odometry):
        t = self.get_clock().now().nanoseconds / 1e9
        p = msg.pose.pose.position
        self.vio.append((t, p.x, p.y, p.z))

    def _cb_ref(self, msg: VehicleLocalPosition):
        t = self.get_clock().now().nanoseconds / 1e9
        # NED(z-down) -> z-up，便于与 OpenVINS 输出同向比较
        self.ref.append((t, float(msg.x), float(msg.y), float(-msg.z)))

    # ---------------- 配对 ----------------
    def _pairs(self):
        """按到达时刻就近配对，返回 (vio_xyz, ref_xyz) 两个 Nx3 数组。"""
        if not self.vio or not self.ref:
            return None, None
        ref_t = np.array([r[0] for r in self.ref])
        ref_p = np.array([r[1:] for r in self.ref])
        vs, rs = [], []
        for t, x, y, z in self.vio:
            k = int(np.argmin(np.abs(ref_t - t)))
            if abs(ref_t[k] - t) < self.match_tol:
                vs.append((x, y, z))
                rs.append(ref_p[k])
        if len(vs) < 10:
            return None, None
        return np.array(vs), np.array(rs)

    # ---------------- 对齐 ----------------
    def _align(self, v, r):
        """最小二乘求 yaw+平移（参考轨迹水平里程 <3m 时退化为纯平移）。"""
        path = np.linalg.norm(np.diff(r[:, :2], axis=0), axis=1).sum()
        if path < 3.0:
            self.aligned = (0.0, *(r[0] - v[0]))
            return
        vc = v[:, :2] - v[:, :2].mean(0)
        rc = r[:, :2] - r[:, :2].mean(0)
        # yaw = argmin |R(yaw) v - r|
        s = (rc[:, 0] + 1j * rc[:, 1]) * np.conj(vc[:, 0] + 1j * vc[:, 1])
        yaw = math.atan2(s.sum().imag, s.sum().real)
        c, sn = math.cos(yaw), math.sin(yaw)
        R = np.array([[c, -sn, 0.0], [sn, c, 0.0], [0.0, 0.0, 1.0]])
        t = r.mean(0) - R @ v.mean(0)
        self.aligned = (yaw, *t)
        self.get_logger().info(
            f'轨迹已对齐：yaw={math.degrees(yaw):.1f}° '
            f't=({t[0]:.2f},{t[1]:.2f},{t[2]:.2f})')

    def _apply(self, v):
        yaw, tx, ty, tz = self.aligned
        c, sn = math.cos(yaw), math.sin(yaw)
        R = np.array([[c, -sn, 0.0], [sn, c, 0.0], [0.0, 0.0, 1.0]])
        return (R @ v.T).T + np.array([tx, ty, tz])

    # ---------------- 报告 ----------------
    def _report(self):
        v, r = self._pairs()
        if v is None:
            self.get_logger().info('等待数据（检查 /uavN/odomimu 是否在发布）...')
            return
        if self.aligned is None:
            self._align(v, r)
            return  # 下一周期开始统计
        va = self._apply(v)
        err = va - r
        e_xy = np.linalg.norm(err[:, :2], axis=1)
        e_3d = np.linalg.norm(err, axis=1)
        path = np.linalg.norm(np.diff(r[:, :2], axis=0), axis=1).sum()
        rmse_xy = float(np.sqrt((e_xy ** 2).mean()))
        rmse_3d = float(np.sqrt((e_3d ** 2).mean()))
        drift = rmse_xy / path * 100.0 if path > 1.0 else float('nan')
        self.get_logger().info(
            f'样本 {len(v)} | 当前水平误差 {e_xy[-1]:.2f}m | '
            f'RMSE 水平 {rmse_xy:.2f}m / 3D {rmse_3d:.2f}m | '
            f'里程 {path:.0f}m | 漂移 {drift:.2f}%')


def main(args=None):
    rclpy.init(args=args)
    node = VioEval()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
