#!/usr/bin/env python3
"""
latlon_odom -- publishes the FUSED robot pose expressed in latitude/longitude.

in :  /gps/filtered     (NavSatFix)  fused lat/lon from navsat_transform
      /odometry/global  (Odometry)   fused pose from ekf_global
out:  /odometry/latlon      GeoPoseStamped
      /corobo/heading_deg   Float64, 0 = North, clockwise
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geographic_msgs.msg import GeoPoseStamped


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class LatLonOdom(Node):
    def __init__(self):
        super().__init__('latlon_odom')

        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('log_every_n', 20)

        self._fix = None
        self._odom = None
        self._count = 0

        self.create_subscription(NavSatFix, 'gps/filtered',
                                 self._on_fix, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odometry/global',
                                 self._on_odom, 10)

        self.pub_geo = self.create_publisher(GeoPoseStamped, 'odometry/latlon', 10)
        self.pub_hdg = self.create_publisher(Float64, 'corobo/heading_deg', 10)

        rate = self.get_parameter('publish_rate').value
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info('latlon_odom up -> /odometry/latlon')

    def _on_fix(self, msg):
        self._fix = msg

    def _on_odom(self, msg):
        self._odom = msg

    def _tick(self):
        if self._fix is None or self._odom is None:
            return

        msg = GeoPoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'wgs84'
        msg.pose.position.latitude = self._fix.latitude
        msg.pose.position.longitude = self._fix.longitude
        msg.pose.position.altitude = self._fix.altitude
        msg.pose.orientation = self._odom.pose.pose.orientation
        self.pub_geo.publish(msg)

        yaw = yaw_from_quat(self._odom.pose.pose.orientation)
        heading = (90.0 - math.degrees(yaw)) % 360.0
        self.pub_hdg.publish(Float64(data=heading))

        n = self.get_parameter('log_every_n').value
        self._count += 1
        if n and self._count % n == 0:
            self.get_logger().info(
                f'lat={msg.pose.position.latitude:.7f} '
                f'lon={msg.pose.position.longitude:.7f} '
                f'hdg={heading:6.2f} deg')


def main():
    rclpy.init()
    node = LatLonOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
