#!/usr/bin/env python3
"""
arm_tracker_controller
=======================
Subscribes to /drone_pos (std_msgs/String: LEFT/RIGHT/UP/DOWN/STOP/LOST)
and drives the 2-DOF arm to follow it:

    LEFT   -> base_joint rotates left  (cmd_vel_basejoint,  angular.z = +speed)
    RIGHT  -> base_joint rotates right (cmd_vel_basejoint,  angular.z = -speed)
    UP     -> elbow_joint tilts up     (cmd_vel_elbowjoint, angular.z = +speed)
    DOWN   -> elbow_joint tilts down   (cmd_vel_elbowjoint, angular.z = -speed)
    STOP   -> both motors stop
    LOST   -> both motors stop (no target -- don't keep moving blind)

No class -- plain function-based rclpy node, matching the style of
base_joint_driver.py / elbow_joint_driver.py.

Run directly with python3 (source the workspace first so rclpy resolves):
    cd ~/corobo_ws && source install/setup.bash
    python3 ~/corobo_ws/src/corobo/scripts/arm_tracker_controller.py
"""

import rclpy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


def main():
    rclpy.init()
    node = rclpy.create_node('arm_tracker_controller')

    node.declare_parameter('base_speed', 2.0)   # rad/s
    node.declare_parameter('elbow_speed', 1.5)  # rad/s
    node.declare_parameter('drone_pos_topic', 'drone_pos')

    base_speed = node.get_parameter('base_speed').value
    elbow_speed = node.get_parameter('elbow_speed').value
    topic = node.get_parameter('drone_pos_topic').value

    base_pub = node.create_publisher(Twist, 'cmd_vel_basejoint', 10)
    elbow_pub = node.create_publisher(Twist, 'cmd_vel_elbowjoint', 10)

    def stop_base():
        base_pub.publish(Twist())

    def stop_elbow():
        elbow_pub.publish(Twist())

    def stop_both():
        stop_base()
        stop_elbow()

    def drive_base(speed):
        msg = Twist()
        msg.angular.z = -speed
        base_pub.publish(msg)

    def drive_elbow(speed):
        msg = Twist()
        msg.angular.z = -speed
        elbow_pub.publish(msg)

    def on_drone_pos(msg):
        label = msg.data.strip().upper()

        if label == 'LEFT':
            stop_elbow()
            drive_base(base_speed)
            node.get_logger().info(f'LEFT  -> base rotating left  ({base_speed} rad/s)')

        elif label == 'RIGHT':
            stop_elbow()
            drive_base(-base_speed)
            node.get_logger().info(f'RIGHT -> base rotating right ({-base_speed} rad/s)')

        elif label == 'UP':
            stop_base()
            drive_elbow(elbow_speed)
            node.get_logger().info(f'UP    -> elbow tilting up    ({elbow_speed} rad/s)')

        elif label == 'DOWN':
            stop_base()
            drive_elbow(-elbow_speed)
            node.get_logger().info(f'DOWN  -> elbow tilting down  ({-elbow_speed} rad/s)')

        elif label == 'STOP':
            stop_both()
            node.get_logger().info('STOP  -> both motors stopped')

        elif label == 'LOST':
            stop_both()
            node.get_logger().info('LOST  -> target lost, both motors stopped')

        else:
            node.get_logger().warn(f'unrecognized drone_pos label: "{label}"')

    node.create_subscription(String, topic, on_drone_pos, 10)
    node.get_logger().info(f'arm_tracker_controller up -- listening on /{topic}')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_both()
        node.get_logger().info('shutting down, motors stopped')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
