#!/usr/bin/env python3
"""
latlon_goal -- send corobo to a WGS84 lat/lon using Nav2.

A) one-shot:
   ros2 run corobo latlon_goal.py --ros-args -p lat:=13.0102 -p lon:=80.2354

B) goal server:
   ros2 run corobo latlon_goal.py --ros-args -p listen:=true
   ros2 topic pub --once /goal_latlon geographic_msgs/msg/GeoPoseStamped \
     "{pose: {position: {latitude: 13.0102, longitude: 80.2354}}}"
"""

import math
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from robot_localization.srv import FromLL
from geographic_msgs.msg import GeoPoint, GeoPoseStamped
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class LatLonGoal(Node):
    def __init__(self):
        super().__init__('latlon_goal')

        self.declare_parameter('lat', 0.0)
        self.declare_parameter('lon', 0.0)
        self.declare_parameter('alt', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('listen', False)
        self.declare_parameter('fromll_service', '/fromLL')

        srv_name = self.get_parameter('fromll_service').value
        self.cli = self.create_client(FromLL, srv_name)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info(f'waiting for {srv_name} ...')
        while not self.cli.wait_for_service(timeout_sec=2.0) and rclpy.ok():
            self.get_logger().warn(
                f'{srv_name} not up yet -- is navsat_transform running?')

        self.get_logger().info('waiting for /navigate_to_pose ...')
        self.nav.wait_for_server()

        if self.get_parameter('listen').value:
            self.create_subscription(GeoPoseStamped, 'goal_latlon',
                                     self._on_goal_msg, 10)
            self.get_logger().info('listening on /goal_latlon')
        else:
            lat = self.get_parameter('lat').value
            lon = self.get_parameter('lon').value
            alt = self.get_parameter('alt').value
            yaw = self.get_parameter('yaw').value
            if lat == 0.0 and lon == 0.0:
                self.get_logger().error(
                    'set lat and lon, e.g. --ros-args -p lat:=13.01 -p lon:=80.235')
                sys.exit(1)
            self.send(lat, lon, alt, yaw)

    def _on_goal_msg(self, msg):
        self.send(msg.pose.position.latitude,
                  msg.pose.position.longitude,
                  msg.pose.position.altitude,
                  0.0,
                  orientation=msg.pose.orientation)

    def send(self, lat, lon, alt=0.0, yaw=0.0, orientation=None):
        req = FromLL.Request()
        req.ll_point = GeoPoint(latitude=float(lat),
                                longitude=float(lon),
                                altitude=float(alt))
        self.get_logger().info(f'converting ({lat:.7f}, {lon:.7f}) -> map frame')
        fut = self.cli.call_async(req)
        fut.add_done_callback(
            lambda f: self._after_convert(f, yaw, orientation))

    def _after_convert(self, fut, yaw, orientation):
        try:
            point = fut.result().map_point
        except Exception as e:
            self.get_logger().error(f'/fromLL failed: {e}')
            return

        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = point.x
        ps.pose.position.y = point.y
        ps.pose.position.z = 0.0
        if orientation is not None and any(
                [orientation.x, orientation.y, orientation.z, orientation.w]):
            ps.pose.orientation = orientation
        else:
            qx, qy, qz, qw = quat_from_yaw(yaw)
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
        goal.pose = ps

        self.get_logger().info(
            f'map goal x={point.x:.2f} y={point.y:.2f} -- sending to Nav2')
        send_fut = self.nav.send_goal_async(goal, feedback_callback=self._fb)
        send_fut.add_done_callback(self._on_accepted)

    def _on_accepted(self, fut):
        handle = fut.result()
        if not handle.accepted:
            self.get_logger().error('Nav2 rejected the goal')
            return
        self.get_logger().info('goal accepted')
        handle.get_result_async().add_done_callback(self._on_result)

    def _fb(self, fb):
        d = fb.feedback.distance_remaining
        self.get_logger().info(f'{d:.2f} m to go', throttle_duration_sec=2.0)

    def _on_result(self, fut):
        self.get_logger().info(f'navigation finished: {fut.result().status}')
        if not self.get_parameter('listen').value:
            rclpy.try_shutdown()


def main():
    rclpy.init()
    node = LatLonGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
