#!/usr/bin/env python3
"""
mocap_map_odom — publishes map->odom from an OptiTrack (VRPN) pose,
replacing AMCL as Nav2's global localizer.

    map->odom = (map->base) . (odom->base)^-1
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def compose(a, b):
    ax, ay, ath = a
    bx, by, bth = b
    ca, sa = math.cos(ath), math.sin(ath)
    return (ax + ca * bx - sa * by, ay + sa * bx + ca * by, ath + bth)


def inverse(a):
    ax, ay, ath = a
    ca, sa = math.cos(ath), math.sin(ath)
    return (-(ca * ax + sa * ay), (sa * ax - ca * ay), -ath)


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class MocapMapOdom(Node):
    def __init__(self):
        super().__init__('mocap_map_odom')
        self.declare_parameter('mocap_topic', '/vrpn_mocap/Limo/pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('reg_x', 0.0)
        self.declare_parameter('reg_y', 0.0)
        self.declare_parameter('reg_yaw', 0.0)
        self.declare_parameter('jump_threshold', 0.0)

        self.mocap_topic = self.get_parameter('mocap_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        rate = float(self.get_parameter('publish_rate').value)
        self.reg = (float(self.get_parameter('reg_x').value),
                    float(self.get_parameter('reg_y').value),
                    float(self.get_parameter('reg_yaw').value))
        self.jump_threshold = float(self.get_parameter('jump_threshold').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST
        self.latest_map_base = None
        self.prev_map_base = None
        self.create_subscription(PoseStamped, self.mocap_topic, self.mocap_cb, qos)
        self.create_timer(1.0 / rate, self.publish_map_odom)

        self.get_logger().info(
            f"mocap_map_odom: {self.mocap_topic} -> "
            f"{self.map_frame}->{self.odom_frame} (base={self.base_frame}), "
            f"reg={self.reg}, rate={rate} Hz")

    def mocap_cb(self, msg):
        p = msg.pose.position
        q = msg.pose.orientation
        world_base = (p.x, p.y, yaw_from_quat(q.x, q.y, q.z, q.w))
        map_base = compose(self.reg, world_base)
        if self.jump_threshold > 0.0 and self.prev_map_base is not None:
            d = math.hypot(map_base[0] - self.prev_map_base[0],
                           map_base[1] - self.prev_map_base[1])
            if d > self.jump_threshold:
                self.get_logger().warn(f"Rejected mocap jump of {d:.2f} m")
                return
        self.latest_map_base = map_base
        self.prev_map_base = map_base

    def publish_map_odom(self):
        if self.latest_map_base is None:
            return
        try:
            t = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, Time())
        except Exception as e:
            self.get_logger().warn(
                f"waiting for {self.odom_frame}->{self.base_frame}: {e}",
                throttle_duration_sec=2.0)
            return
        ot = t.transform.translation
        oq = t.transform.rotation
        odom_base = (ot.x, ot.y, yaw_from_quat(oq.x, oq.y, oq.z, oq.w))
        mx, my, myaw = compose(self.latest_map_base, inverse(odom_base))
        myaw = normalize_angle(myaw)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.odom_frame
        tf.transform.translation.x = mx
        tf.transform.translation.y = my
        tf.transform.translation.z = 0.0
        tf.transform.rotation.z = math.sin(myaw / 2.0)
        tf.transform.rotation.w = math.cos(myaw / 2.0)
        self.tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = MocapMapOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
