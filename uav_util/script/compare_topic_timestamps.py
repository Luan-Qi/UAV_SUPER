#!/usr/bin/env python3
"""
================================================================================
 compare_topic_timestamps.py  —  ROS 双话题时间戳实时对比工具
================================================================================

用途
----
  在无人机 / 机器人系统中，经常需要确认两个传感器话题之间的时间同步情况。
  本脚本同时订阅两个 ROS 话题，持续记录各自最新消息的 header.stamp, 
  然后按固定频率（默认 10 Hz)打印它们之间的时间偏差。

  典型场景:
    - 比较里程计 /Odometry 与视觉位姿 /mavros/vision_pose/pose 的同步误差
    - 比较相机彩色图与深度图的曝光时间戳偏差
    - 比较激光雷达点云与 IMU 数据的时间对齐情况
    - 排查传感器数据融合时的时间戳跳变 / 延迟问题

工作原理
--------
  1. 启动时从 ROS Master 自动探测两个话题的消息类型（也可手动指定）
  2. 分别订阅话题 A 和话题 B, 回调中只做一件事: 更新"最新时间戳"
  3. 定时器按 print_rate 频率触发，读取两个话题各自最新的:
       - header.stamp(消息自身时间戳，代表传感器采样时刻）
       - 回调接收时刻(rospy.Time.now()，代表网络到达时刻）
  4. 打印: stamp 偏差(A-B) / A传输延迟 / B传输延迟 / 消息计数
  5. 如启动后某话题在 msg_timeout 秒内无消息到达，脚本自动退出

支持的参数
----------
  -a, --topic_a  (必填)  话题 A 的名称
  -b, --topic_b  (必填)  话题 B 的名称
  --type_a       (可选)  话题 A 的消息类型，如 sensor_msgs/Image
                         不填则自动从 ROS Master 探测
  --type_b       (可选)  话题 B 的消息类型，如 sensor_msgs/PointCloud2
  --rate         (可选)  打印频率，单位 Hz, 默认 10
  --detect_timeout (可选) 探测话题类型的最大等待秒数，默认 5
  --msg_timeout  (可选)  等待首条消息的最大秒数，超时退出，默认 10
  --wait         (可选)  一直等待不退出（覆盖 msg_timeout)

兼容性
------
  两个话题可以使用不同的消息类型（如 Image 与 Odometry),
  只要消息带有 header.stamp 字段即可。绝大多数 ROS 标准消息类型
  (sensor_msgs/*, nav_msgs/Odometry, geometry_msgs/PoseStamped 等）
  都满足此条件。

输出字段解释
--------------
  每行包含 5 个字段，逐列说明:

  ┌────────────────┬──────────────────────────────────────────────────────┐
  │ 列名           │ 含义                                                 │
  ├────────────────┼──────────────────────────────────────────────────────┤
  │ stamp_diff(µs) │ 话题 A 与 B 最新消息的 header.stamp 差值 (A - B)    │
  │                │ >0 = A 的采样时刻晚于 B (A 比 B 更靠后发生）        │
  │                │ <0 = A 的采样时刻早于 B (A 比 B 更早发生）          │
  │                │ ≈0 = 两个话题时间同步良好                            │
  ├────────────────┼──────────────────────────────────────────────────────┤
  │ delay_A(µs)    │ 话题 A 的传输延迟 = 接收时刻 - 消息内时间戳         │
  │                │ 含义: 从传感器采样到本机收到消息经过了多久            │
  │                │ 越小说明网络/驱动延迟越低                            │
  ├────────────────┼──────────────────────────────────────────────────────┤
  │ delay_B(µs)    │ 话题 B 的传输延迟，含义同上                          │
  ├────────────────┼──────────────────────────────────────────────────────┤
  │ cnt_A          │ 话题 A 累计收到的消息总数                            │
  ├────────────────┼──────────────────────────────────────────────────────┤
  │ cnt_B          │ 话题 B 累计收到的消息总数                            │
  └────────────────┴──────────────────────────────────────────────────────┘

运行示例
--------
  ============================================================
    Topic A:  /Odometry                        (nav_msgs/Odometry)
    Topic B:  /mavros/vision_pose/pose         (geometry_msgs/PoseStamped)
    Print rate:  10.0 Hz
    Msg timeout: 10.0 s
  ============================================================
  stamp_diff(µs) | delay_A(µs) delay_B(µs) |   cnt_A    cnt_B
  ----------------------------------------------------------------
      +1234      |      512        498     |     1234     1230
      +1198      |      501        503     |     1244     1240
        -45      |      512        510     |     1254     1250

Usage:
    # 自动探测话题类型（最简单）
    ./compare_topic_timestamps.py -a /Odometry -b /mavros/vision_pose/pose

    # 手动指定不同消息类型
    ./compare_topic_timestamps.py \
        -a /Odometry -b /mavros/vision_pose/pose \
        --type_a nav_msgs/Odometry --type_b geometry_msgs/PoseStamped

    # 完整参数
    ./compare_topic_timestamps.py \
        -a /Odometry \
        -b /mavros/vision_pose/pose \
        --type_a nav_msgs/Odometry \
        --type_b geometry_msgs/PoseStamped \
        --rate 20 \
        --detect_timeout 5.0 \
        --msg_timeout 10.0
"""

import argparse
import sys
import rospy
import importlib


# ------------------------------------------------------------------
# 自动探测话题类型
# ------------------------------------------------------------------
def detect_topic_type(topic_name, timeout):
    """
    从 ROS Master 查询话题的消息类型。
    如果话题尚未被发布，会等待直到超时。
    返回 "pkg/MsgType" 字符串；失败返回 None。
    """
    resolved = rospy.resolve_name(topic_name)
    rate = rospy.Rate(10)
    waited = 0.0

    while not rospy.is_shutdown():
        published = rospy.get_published_topics()
        for t, ttype in published:
            if t == resolved:
                rospy.loginfo("[auto-detect] %s -> %s", resolved, ttype)
                return ttype
        if waited >= timeout:
            rospy.logerr("[auto-detect] timeout (%.1fs): no publisher for %s",
                         timeout, resolved)
            return None
        if waited == 0.0:
            rospy.loginfo("[auto-detect] waiting for '%s' ...", resolved)
        rate.sleep()
        waited += 0.1

    return None


def import_msg_type(msg_type_str):
    pkg, msg = msg_type_str.split("/")
    mod = importlib.import_module("{}.msg".format(pkg))
    return getattr(mod, msg)


# ------------------------------------------------------------------
# 主类
# ------------------------------------------------------------------
class TimestampComparer:
    def __init__(self, topic_a, topic_b,
                 msg_type_a, msg_type_b,
                 print_rate, msg_timeout):
        self.topic_a = topic_a
        self.topic_b = topic_b
        self.print_rate = print_rate
        self.msg_timeout = msg_timeout

        # ---- 导入消息类型 ----
        self.msg_type_a = import_msg_type(msg_type_a)
        self.msg_type_b = import_msg_type(msg_type_b)

        # ---- 最新时间戳 ----
        self.last_stamp_a = None
        self.last_stamp_b = None
        self.last_recv_a = None
        self.last_recv_b = None

        # ---- 消息计数 ----
        self.count_a = 0
        self.count_b = 0

        # ---- 首条消息到达时间 ----
        self.first_msg_time = rospy.Time.now()

        # ---- 订阅 ----
        self.sub_a = rospy.Subscriber(
            self.topic_a, self.msg_type_a, self._cb_a, queue_size=10)
        self.sub_b = rospy.Subscriber(
            self.topic_b, self.msg_type_b, self._cb_b, queue_size=10)

        # ---- 定时打印 ----
        period = rospy.Duration(1.0 / print_rate)
        self.print_timer = rospy.Timer(period, self._print_diff)

        # ---- 超时检查（每秒一次）----
        self.check_timer = rospy.Timer(rospy.Duration(1.0), self._check_timeout)

        rospy.loginfo("=" * 60)
        rospy.loginfo("  Topic A:  %-35s (%s)", self.topic_a, msg_type_a)
        rospy.loginfo("  Topic B:  %-35s (%s)", self.topic_b, msg_type_b)
        rospy.loginfo("  Print rate:  %.1f Hz", print_rate)
        rospy.loginfo("  Msg timeout: %.1f s", msg_timeout)
        rospy.loginfo("=" * 60)
        rospy.loginfo(
            "%-14s | %10s %10s | %8s %8s" % (
                "stamp_diff(us)",
                "delay_A(us)", "delay_B(us)",
                "cnt_A", "cnt_B"
            )
        )
        rospy.loginfo("-" * 63)

    @staticmethod
    def _extract_stamp(msg):
        if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
            return msg.header.stamp
        return rospy.Time.now()

    # ---- 回调 ----
    def _cb_a(self, msg):
        if self.last_stamp_a is None:
            self.first_msg_time = rospy.Time.now()
        self.last_stamp_a = self._extract_stamp(msg)
        self.last_recv_a = rospy.Time.now()
        self.count_a += 1

    def _cb_b(self, msg):
        if self.last_stamp_b is None:
            self.first_msg_time = rospy.Time.now()
        self.last_stamp_b = self._extract_stamp(msg)
        self.last_recv_b = rospy.Time.now()
        self.count_b += 1

    # ---- 超时退出 ----
    def _check_timeout(self, event):
        """如果某个话题一条消息都没收到且超过阈值，退出脚本。"""
        if self.last_stamp_a is not None and self.last_stamp_b is not None:
            return  # 两个话题都收到过消息

        now = rospy.Time.now()
        elapsed = (now - self.first_msg_time).to_sec()

        missing = []
        if self.last_stamp_a is None:
            missing.append(self.topic_a)
        if self.last_stamp_b is None:
            missing.append(self.topic_b)

        if elapsed > self.msg_timeout:
            rospy.logerr(
                "[timeout] %.1fs elapsed, still no message on: %s",
                elapsed, ", ".join(missing))
            rospy.logerr("[timeout] exiting.")
            rospy.signal_shutdown("msg_timeout")
        else:
            rospy.loginfo(
                "[waiting] %.1fs / %.1fs — no message on: %s",
                elapsed, self.msg_timeout, ", ".join(missing))

    # ---- 定时打印 ----
    def _print_diff(self, event):
        if self.last_stamp_a is None or self.last_stamp_b is None:
            return

        # stamp_diff = A.header.stamp - B.header.stamp，单位微秒（正=A晚于B）
        diff_us = (self.last_stamp_a - self.last_stamp_b).to_nsec() / 1000.0
        # 传输延迟 = 回调接收时刻 - 消息内时间戳，单位微秒
        delay_a_us = ((self.last_recv_a - self.last_stamp_a).to_nsec() / 1000.0
                      if self.last_recv_a else 0)
        delay_b_us = ((self.last_recv_b - self.last_stamp_b).to_nsec() / 1000.0
                      if self.last_recv_b else 0)

        # 输出: stamp_diff(us) | delay_A | delay_B | cnt_A | cnt_B
        rospy.loginfo(
            "%+12.1f | %10.1f %10.1f | %6d %6d",
            diff_us,
            delay_a_us,
            delay_b_us,
            self.count_a,
            self.count_b,
        )


# ============================================================================
# Main
# ============================================================================
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Compare timestamps between two ROS topics in real-time.")
    parser.add_argument("-a", "--topic_a", required=True,
                        help="First topic name")
    parser.add_argument("-b", "--topic_b", required=True,
                        help="Second topic name")
    parser.add_argument("--type_a", default=None,
                        help="Message type for topic A (e.g. sensor_msgs/Image). "
                             "Auto-detect if omitted.")
    parser.add_argument("--type_b", default=None,
                        help="Message type for topic B (e.g. sensor_msgs/PointCloud2). "
                             "Auto-detect if omitted.")
    parser.add_argument("--rate", type=float, default=10.0,
                        help="Print rate in Hz (default: 10)")
    parser.add_argument("--detect_timeout", type=float, default=5.0,
                        help="Max seconds to wait for topic type detection (default: 5)")
    parser.add_argument("--msg_timeout", type=float, default=10.0,
                        help="Max seconds to wait for first message before exiting (default: 10)")
    parser.add_argument("--wait", action="store_true", default=False,
                        help="Keep waiting indefinitely even if no messages arrive")

    # 过滤掉 ROS 自己加的 remapping 参数
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node("compare_topic_timestamps")

    # ---- 解析消息类型（手动指定 > 自动探测）----
    msg_type_a = args.type_a
    msg_type_b = args.type_b

    if msg_type_a is None:
        msg_type_a = detect_topic_type(args.topic_a, args.detect_timeout)
    if msg_type_b is None:
        msg_type_b = detect_topic_type(args.topic_b, args.detect_timeout)

    if msg_type_a is None:
        rospy.logerr("Cannot determine message type for topic A: %s", args.topic_a)
        rospy.logerr("Please specify manually: --type_a pkg/MsgType")
        sys.exit(1)
    if msg_type_b is None:
        rospy.logerr("Cannot determine message type for topic B: %s", args.topic_b)
        rospy.logerr("Please specify manually: --type_b pkg/MsgType")
        sys.exit(1)

    msg_timeout = 0 if args.wait else args.msg_timeout

    TimestampComparer(
        topic_a=args.topic_a,
        topic_b=args.topic_b,
        msg_type_a=msg_type_a,
        msg_type_b=msg_type_b,
        print_rate=args.rate,
        msg_timeout=msg_timeout,
    )
    rospy.spin()
