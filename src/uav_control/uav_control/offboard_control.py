"""
PX4 Offboard 控制节点（每架无人机一个实例）。

职责（对应框架文档 4.1 节）：
  - 持续 10 Hz 发布 OffboardControlMode + TrajectorySetpoint（Offboard 心跳）
  - 自动完成：进入 Offboard -> 解锁 -> 起飞到指定高度 -> 悬停
  - 悬停后跟踪上层（集群调度）通过 ~/waypoint 下发的航点（本机本地 NED 系）
  - 收到 ~/command = "land" 后降落并上锁
  - 通过 ~/state 向集群调度汇报状态

坐标约定：全部使用 PX4 本地 NED 系（北 x / 东 y / 下 z，高度 = -z）。
话题：飞控侧在 px4_ns（默认 px4_1，多机为 px4_1..px4_4）命名空间下；
      上层接口在本节点自身命名空间（launch 中设为 uavN）。
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                       QoSDurabilityPolicy, QoSHistoryPolicy)

from std_msgs.msg import String
from geometry_msgs.msg import Point

from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition, VehicleStatus)


def px4_qos() -> QoSProfile:
    """PX4 uXRCE-DDS 要求的 QoS。"""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )


# PX4 自定义模式 / 命令常量
NAV_STATE_OFFBOARD = 14
ARMING_STATE_ARMED = 2
CMD_DO_SET_MODE = 176          # MAV_CMD_DO_SET_MODE
CMD_COMPONENT_ARM_DISARM = 400  # MAV_CMD_COMPONENT_ARM_DISARM
CMD_NAV_LAND = 21              # MAV_CMD_NAV_LAND
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6.0


class OffboardControl(Node):
    """状态机：INIT -> ARMING -> TAKEOFF -> MISSION(跟踪航点) -> LAND -> DONE"""

    def __init__(self):
        super().__init__('offboard_control')

        # ---- 参数 ----
        self.declare_parameter('px4_ns', 'px4_1')       # 飞控命名空间
        self.declare_parameter('px4_instance', 1)       # SITL 实例号（sysid = instance + 1）
        self.declare_parameter('takeoff_alt', 2.0)      # 起飞高度 m（正数）
        self.declare_parameter('auto_takeoff', True)    # 是否自动起飞（仿真默认开）
        self.declare_parameter('reach_tol', 0.25)       # 航点到达容差 m

        self.px4_ns = self.get_parameter('px4_ns').value
        self.sysid = int(self.get_parameter('px4_instance').value) + 1
        self.takeoff_alt = float(self.get_parameter('takeoff_alt').value)
        self.auto_takeoff = bool(self.get_parameter('auto_takeoff').value)
        self.reach_tol = float(self.get_parameter('reach_tol').value)

        qos = px4_qos()

        # ---- 飞控接口（px4_ns/fmu/...）----
        self.pub_offboard_mode = self.create_publisher(
            OffboardControlMode, f'/{self.px4_ns}/fmu/in/offboard_control_mode', qos)
        self.pub_setpoint = self.create_publisher(
            TrajectorySetpoint, f'/{self.px4_ns}/fmu/in/trajectory_setpoint', qos)
        self.pub_vehicle_cmd = self.create_publisher(
            VehicleCommand, f'/{self.px4_ns}/fmu/in/vehicle_command', qos)

        self.create_subscription(
            VehicleLocalPosition, f'/{self.px4_ns}/fmu/out/vehicle_local_position',
            self._cb_local_pos, qos)
        self.create_subscription(
            VehicleStatus, f'/{self.px4_ns}/fmu/out/vehicle_status',
            self._cb_status, qos)

        # ---- 上层接口（本节点命名空间 uavN 下）----
        self.create_subscription(Point, 'waypoint', self._cb_waypoint, 10)
        self.create_subscription(String, 'command', self._cb_command, 10)
        self.pub_state = self.create_publisher(String, 'state', 10)

        # ---- 内部状态 ----
        self.state = 'INIT'
        self.heartbeat_count = 0
        self.local_pos = VehicleLocalPosition()
        self.status = VehicleStatus()
        self.have_pos = False
        # 当前目标航点（NED），None 表示保持当前位置
        self.target = None

        self.timer = self.create_timer(0.1, self._tick)  # 10 Hz
        self.get_logger().info(
            f'Offboard 节点就绪：px4_ns=/{self.px4_ns}, sysid={self.sysid}, '
            f'起飞高度 {self.takeoff_alt} m')

    # ---------------- 回调 ----------------
    def _cb_local_pos(self, msg: VehicleLocalPosition):
        self.local_pos = msg
        self.have_pos = True

    def _cb_status(self, msg: VehicleStatus):
        self.status = msg

    def _cb_waypoint(self, msg: Point):
        """上层下发航点（本机本地 NED）。"""
        self.target = (msg.x, msg.y, msg.z)

    def _cb_command(self, msg: String):
        if msg.data == 'land' and self.state in ('MISSION', 'TAKEOFF'):
            self.get_logger().info('收到降落指令')
            self.state = 'LAND'

    # ---------------- 飞控指令 ----------------
    def _send_vehicle_command(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = float(p1)
        msg.param2 = float(p2)
        msg.command = command
        msg.target_system = self.sysid
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.pub_vehicle_cmd.publish(msg)

    def _engage_offboard(self):
        self._send_vehicle_command(
            CMD_DO_SET_MODE, p1=1.0, p2=PX4_CUSTOM_MAIN_MODE_OFFBOARD)

    def _arm(self):
        self._send_vehicle_command(CMD_COMPONENT_ARM_DISARM, p1=1.0)

    def _land(self):
        self._send_vehicle_command(CMD_NAV_LAND)

    # ---------------- 心跳与设定点 ----------------
    def _publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.pub_offboard_mode.publish(msg)

    def _publish_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float('nan')   # 不控偏航，保持当前航向
        self.pub_setpoint.publish(msg)

    def _dist_to(self, x, y, z):
        return math.sqrt((self.local_pos.x - x) ** 2 +
                         (self.local_pos.y - y) ** 2 +
                         (self.local_pos.z - z) ** 2)

    # ---------------- 主循环 10 Hz ----------------
    def _tick(self):
        # Offboard 心跳必须始终发布（否则 0.5 s 后飞控退出 Offboard）
        self._publish_offboard_mode()

        if self.state == 'INIT':
            if not self.have_pos:
                return
            # 先原地保持
            self._publish_setpoint(self.local_pos.x, self.local_pos.y, self.local_pos.z)
            self.heartbeat_count += 1
            # 官方推荐：先发约 1 s 心跳再切 Offboard + 解锁
            if self.heartbeat_count >= 10 and self.auto_takeoff:
                self._engage_offboard()
                self._arm()
                self.state = 'ARMING'
                self.get_logger().info('请求 Offboard + 解锁')

        elif self.state == 'ARMING':
            self._publish_setpoint(self.local_pos.x, self.local_pos.y, self.local_pos.z)
            if (self.status.arming_state == ARMING_STATE_ARMED and
                    self.status.nav_state == NAV_STATE_OFFBOARD):
                self.state = 'TAKEOFF'
                self.get_logger().info('已解锁并进入 Offboard，开始起飞')
            else:
                # 未成功则重发
                self._engage_offboard()
                self._arm()

        elif self.state == 'TAKEOFF':
            x, y = self.local_pos.x, self.local_pos.y
            z = -self.takeoff_alt
            self._publish_setpoint(x, y, z)
            if self._dist_to(x, y, z) < self.reach_tol:
                self.state = 'MISSION'
                self.get_logger().info(f'到达起飞高度 {self.takeoff_alt} m，进入任务模式')

        elif self.state == 'MISSION':
            if self.target is not None:
                self._publish_setpoint(*self.target)
            else:
                self._publish_setpoint(self.local_pos.x, self.local_pos.y, self.local_pos.z)

        elif self.state == 'LAND':
            self._land()
            self.state = 'DONE'

        elif self.state == 'DONE':
            if self.status.arming_state != ARMING_STATE_ARMED:
                self.get_logger().info('已降落上锁')
                self.state = 'IDLE'

        # 状态上报（供集群调度聚合）
        s = String()
        s.data = self.state
        self.pub_state.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
