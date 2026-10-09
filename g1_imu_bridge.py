#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
G1 本体 IMU -> sensor_msgs/Imu 桥接节点

背景：
    G1 (unitree_hg) 没有独立的 IMU 话题，IMU 打包在 DDS 话题 rt/lowstate
    里 (unitree_hg/msg/LowState_ 的 imu_state 字段)。FAST-LIO2 需要标准的
    sensor_msgs/Imu，所以必须有这一步桥接。

功能：
    订阅 DDS topic rt/lowstate -> 取出 imu_state -> 发布 sensor_msgs/Imu，
    发布的话题可被 ros2 bag record 录制，也可直接喂给 FAST-LIO2。

运行（在 G1 机载电脑上）：
    python3 g1_imu_bridge.py --ros-args \
        -p interface:=enp2s0 \
        -p dds_topic:=rt/lowstate \
        -p ros_topic:=/g1/imu \
        -p frame_id:=imu_link

依赖：
    - ROS2 (rclpy)
    - unitree_sdk2py (含 cyclonedds==0.10.2)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from sensor_msgs.msg import Imu

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_


class G1ImuBridge(Node):
    def __init__(self):
        super().__init__("g1_imu_bridge")

        # ---------------- 参数 ----------------
        self.declare_parameter("interface", "")        # 网卡名，如 'enp2s0'；空=自动
        self.declare_parameter("domain_id", 0)         # DDS domain，需与机器人一致
        self.declare_parameter("dds_topic", "rt/lowstate")
        self.declare_parameter("ros_topic", "/g1/imu")
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("reliability", "reliable")   # reliable | best_effort
        self.declare_parameter("publish_orientation", True)
        self.declare_parameter("log_rate_hz", 5.0)          # 0=关闭频率打印

        p = self.get_parameter
        self.interface = p("interface").value
        self.domain_id = int(p("domain_id").value)
        dds_topic = p("dds_topic").value
        self.ros_topic = p("ros_topic").value
        self.frame_id = p("frame_id").value
        self.publish_orientation = bool(p("publish_orientation").value)

        rel = str(p("reliability").value).lower()
        reliability = (
            ReliabilityPolicy.RELIABLE
            if rel == "reliable"
            else ReliabilityPolicy.BEST_EFFORT
        )
        # 注意：RELIABLE 发布者与 BEST_EFFORT 订阅者兼容，反之不兼容，
        # 所以默认用 RELIABLE 最保险；depth 给大一点避免高频丢消息。
        qos = QoSProfile(
            depth=1000,
            reliability=reliability,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(Imu, self.ros_topic, qos)

        # ---------------- 初始化 Unitree DDS ----------------
        ChannelFactoryInitialize(self.domain_id, self.interface)
        self.sub = ChannelSubscriber(dds_topic, LowState_)
        self.sub.Init(self.on_lowstate, 10)

        # ---------------- 频率统计 ----------------
        self._count = 0
        self._last_log = self.get_clock().now()
        log_hz = float(p("log_rate_hz").value)
        if log_hz > 0.0:
            self.create_timer(1.0 / log_hz, self._log_rate)

        self.get_logger().info(
            f"IMU bridge started: DDS[{self.interface or 'auto'}] "
            f"{dds_topic} (domain {self.domain_id}) -> ROS2 {self.ros_topic} "
            f"({rel})"
        )

    def on_lowstate(self, msg: LowState_):
        imu = msg.imu_state

        out = Imu()
        # G1 没有硬件时间戳，只有 tick；这里用接收时刻（主机时钟）打标。
        # 只要 LiDAR 驱动也用同一主机时钟打标，两者时间基准就一致。
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id

        if self.publish_orientation:
            # LowState 里 quaternion 顺序为 (w, x, y, z)
            out.orientation.w = float(imu.quaternion[0])
            out.orientation.x = float(imu.quaternion[1])
            out.orientation.y = float(imu.quaternion[2])
            out.orientation.z = float(imu.quaternion[3])
        else:
            # -1 表示该字段无有效数据（FAST-LIO2 本身不使用方向）
            out.orientation_covariance[0] = -1.0

        out.angular_velocity.x = float(imu.gyroscope[0])
        out.angular_velocity.y = float(imu.gyroscope[1])
        out.angular_velocity.z = float(imu.gyroscope[2])

        out.linear_acceleration.x = float(imu.accelerometer[0])
        out.linear_acceleration.y = float(imu.accelerometer[1])
        out.linear_acceleration.z = float(imu.accelerometer[2])

        self.pub.publish(out)
        self._count += 1

    def _log_rate(self):
        now = self.get_clock().now()
        dt = (now - self._last_log).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        hz = self._count / dt
        self.get_logger().info(f"IMU rate ~ {hz:.1f} Hz")
        self._count = 0
        self._last_log = now


def main(args=None):
    rclpy.init(args=args)
    node = G1ImuBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
