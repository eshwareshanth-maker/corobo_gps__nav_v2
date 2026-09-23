#!/usr/bin/env python3
"""
serial_base -- ROS 2 <-> Arduino bridge for the PHYSICAL corobo.
Replaces the Gazebo diff-drive plugin: in /cmd_vel, out /odom.

Protocol (115200 baud):
  Pi  -> MCU :  v <left_mps> <right_mps>\n
  Pi  -> MCU :  r\n
  MCU -> Pi  :  o <left_ticks> <right_ticks> <dt_ms>\n
"""

import math
import threading

import rclpy
from rclpy.node import Node

import serial

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class SerialBase(Node):
    def __init__(self):
        super().__init__('serial_base')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_radius', 0.10)
        self.declare_parameter('wheel_separation', 0.40)
        self.declare_parameter('ticks_per_rev', 1320)
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('cmd_timeout', 0.6)

        p = self.get_parameter
        self.R = p('wheel_radius').value
        self.L = p('wheel_separation').value
        self.tpr = float(p('ticks_per_rev').value)
        self.m_per_tick = 2.0 * math.pi * self.R / self.tpr

        self.ser = serial.Serial(p('port').value, p('baud').value, timeout=0.1)
        self.get_logger().info(f"opened {p('port').value}")

        self.x = self.y = self.th = 0.0
        self.vx = self.vth = 0.0
        self.prev_l = self.prev_r = None
        self.last_cmd_time = self.get_clock().now()
        self.lock = threading.Lock()

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.pub_odom = self.create_publisher(Odometry, 'odom', 20)
        self.tfb = TransformBroadcaster(self) if p('publish_tf').value else None

        self.create_timer(0.05, self.watchdog)
        threading.Thread(target=self.reader, daemon=True).start()

    def on_cmd(self, msg):
        v, w = msg.linear.x, msg.angular.z
        vl = v - w * self.L / 2.0
        vr = v + w * self.L / 2.0
        with self.lock:
            self.ser.write(f'v {vl:.4f} {vr:.4f}\n'.encode())
        self.last_cmd_time = self.get_clock().now()

    def watchdog(self):
        dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if dt > self.get_parameter('cmd_timeout').value:
            with self.lock:
                self.ser.write(b'v 0 0\n')

    def reader(self):
        buf = b''
        while rclpy.ok():
            try:
                buf += self.ser.read(self.ser.in_waiting or 1)
            except serial.SerialException as e:
                self.get_logger().error(f'serial error: {e}')
                return
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                self.handle(line.decode(errors='ignore').strip())

    def handle(self, line):
        if not line.startswith('o '):
            if line:
                self.get_logger().debug(f'mcu: {line}')
            return
        try:
            _, l_s, r_s, dt_s = line.split()
            lt, rt, dt_ms = int(l_s), int(r_s), int(dt_s)
        except ValueError:
            return
        dt = dt_ms / 1000.0
        if dt <= 0.0:
            return

        if self.prev_l is None:
            self.prev_l, self.prev_r = lt, rt
            return

        dl = (lt - self.prev_l) * self.m_per_tick
        dr = (rt - self.prev_r) * self.m_per_tick
        self.prev_l, self.prev_r = lt, rt

        d = (dl + dr) / 2.0
        dth = (dr - dl) / self.L

        self.x += d * math.cos(self.th + dth / 2.0)
        self.y += d * math.sin(self.th + dth / 2.0)
        self.th = math.atan2(math.sin(self.th + dth), math.cos(self.th + dth))

        self.vx = d / dt
        self.vth = dth / dt
        self.publish()

    def publish(self):
        now = self.get_clock().now().to_msg()
        odom_frame = self.get_parameter('odom_frame').value
        base_frame = self.get_parameter('base_frame').value
        qz, qw = math.sin(self.th / 2.0), math.cos(self.th / 2.0)

        o = Odometry()
        o.header.stamp = now
        o.header.frame_id = odom_frame
        o.child_frame_id = base_frame
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.orientation.z = qz
        o.pose.pose.orientation.w = qw
        o.twist.twist.linear.x = self.vx
        o.twist.twist.angular.z = self.vth
        o.pose.covariance[0] = 0.05
        o.pose.covariance[7] = 0.05
        o.pose.covariance[35] = 0.10
        o.twist.covariance[0] = 0.02
        o.twist.covariance[35] = 0.04
        self.pub_odom.publish(o)

        if self.tfb:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = odom_frame
            t.child_frame_id = base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tfb.sendTransform(t)


def main():
    rclpy.init()
    node = SerialBase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.ser.write(b'v 0 0\n')
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
