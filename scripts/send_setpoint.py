#!/usr/bin/env python3
"""Send one target pose ("go to this position") on the setpoint topic.

Normally run through ``./scripts/njord goto X Y [HEADING_DEG]`` while
``./scripts/njord lab <course>`` is running; it executes inside the lab's
autonomy container, so it shares the ROS domain and the simulation clock.
Any ROS 2 Jazzy shell on the same ROS_DOMAIN_ID works as well.

Arguments: X and Y in metres in the map frame (ENU: x east, y north, origin
at the world datum) and optionally the heading in degrees counter-clockwise
from east (0 east, 90 north). Without a heading the boat is asked to face
along the line from its current estimated position (/njord/odometry) to the
target.

Publishes one geometry_msgs/PoseStamped on constants.SETPOINT_TOPIC
(reliable, transient local, depth 1), stamped with the current simulation
time, after waiting up to --wait seconds for /clock and for at least one
subscriber (the controller or the lab evaluator). It stays alive one more
second so the message is delivered. A new target replaces the active one.

Exit: 0 when published, 1 when /clock or the needed odometry never arrived.
"""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from njord_sim.constants import MAP_FRAME, SETPOINT_TOPIC
from njord_sim.geometry import yaw_from_quaternion

SETPOINT_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('x', type=float, help='target x (east) in m, map frame')
    parser.add_argument('y', type=float, help='target y (north) in m, map frame')
    parser.add_argument('heading_deg', type=float, nargs='?',
                        help='target heading in degrees counter-clockwise from east (default: along the leg)')
    parser.add_argument('--wait', type=float, default=10.0, help='seconds to wait for /clock and a subscriber')
    args = parser.parse_args()
    values = [args.x, args.y] + ([args.heading_deg] if args.heading_deg is not None else [])
    if not all(math.isfinite(v) for v in values):
        parser.error('target values must be finite numbers')

    rclpy.init()
    node = Node('send_setpoint', parameter_overrides=[Parameter('use_sim_time', value=True)])
    publisher = node.create_publisher(PoseStamped, SETPOINT_TOPIC, SETPOINT_QOS)
    position = []
    node.create_subscription(Odometry, '/njord/odometry',
                             lambda m: position.__setitem__(slice(None), [m.pose.pose.position.x,
                                                                          m.pose.pose.position.y,
                                                                          yaw_from_quaternion(m.pose.pose.orientation)]),
                             qos_profile_sensor_data)
    deadline = time.monotonic() + args.wait
    try:
        while time.monotonic() < deadline and (
                node.get_clock().now().nanoseconds == 0 or publisher.get_subscription_count() == 0
                or (args.heading_deg is None and not position)):
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.get_clock().now().nanoseconds == 0:
            print('No /clock received; is the simulator running in this ROS domain?', file=sys.stderr)
            return 1
        if args.heading_deg is None:
            if not position:
                print('No /njord/odometry received; give HEADING_DEG explicitly.', file=sys.stderr)
                return 1
            heading = math.atan2(args.y - position[1], args.x - position[0])
        else:
            heading = math.radians(args.heading_deg)
        if publisher.get_subscription_count() == 0:
            print(f'Warning: nobody subscribes to {SETPOINT_TOPIC} yet; publishing anyway.', file=sys.stderr)
        message = PoseStamped()
        message.header.frame_id, message.header.stamp = MAP_FRAME, node.get_clock().now().to_msg()
        message.pose.position.x, message.pose.position.y = args.x, args.y
        message.pose.orientation.z, message.pose.orientation.w = math.sin(heading / 2), math.cos(heading / 2)
        publisher.publish(message)
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        print(f'Sent {SETPOINT_TOPIC}: x={args.x:.2f} m, y={args.y:.2f} m, '
              f'heading={math.degrees(heading):.1f} deg (ENU, from east)')
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
