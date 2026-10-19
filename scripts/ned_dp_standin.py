#!/usr/bin/env python3
"""Stand-in for Control & Autonomy's nodes in simulator mode (not their code).

Control & Autonomy (C&A) works in NED and a forward/right/down (FRD) body
frame with its own message types, and converts at the boundary of its nodes
(docs/control-autonomy.md). This node does the same conversion on the
simulator's ROS-standard topics, runs a plain PD dynamic-positioning law in
NED/FRD and allocates the wrench to the thrusters with the vessel file's
layout expressed in FRD. It answers one question before C&A's own nodes are
connected: does a controller that follows the agreed conversion drive the
simulated boat to the target and hold it? It is also the reference for that
conversion. It is not a model of C&A's controller and says nothing about it.

Run it as the external controller of a setpoint course, with the same
VESSEL_CONFIG as the run (a fully actuated vessel such as munin_v0):

    VESSEL_CONFIG=/config/vessels/munin_v0.yaml docker compose -p njord-ca \\
        run --rm autonomy python3 /opt/njord/scripts/ned_dp_standin.py
    STATE_SOURCE=truth CONTROLLER=external VESSEL_CONFIG=/config/vessels/munin_v0.yaml \\
        ./scripts/njord demo hold_position

Subscribes:
    ``/njord/odometry`` (``nav_msgs/Odometry``, best effort): pose of
        ``base_link`` in ``map`` (ENU) and body twist (forward/left/up).
    constants.SETPOINT_TOPIC (``geometry_msgs/PoseStamped``, reliable,
        volatile): target position and heading in ``map``. The latest one
        holds; a repeat with the same stamp and pose changes nothing.

Publishes:
    one ``std_msgs/Float64`` per thruster of the vessel file, on its
        ``/<name>/command`` topic: force in N along the thruster's direction,
        at ``setpoint_control.control_hz`` of simulation time.

Conversion (the one sent to C&A):
    x = y_enu (north), y = x_enu (east), psi = pi/2 - yaw_enu (clockwise from
    north, wrapped to [-pi, pi]); u = v_x, v = -v_y, r = -w_z. A thruster at
    (x, y) pushing at angle a counter-clockwise from forward is at (x, -y)
    pushing at -a in FRD. Thruster forces are scalars along each thruster and
    do not change.

Settings: the thrusters come from the vessel file (``--vessel`` or
VESSEL_CONFIG), the gains (``kp_position``, ``kd_position``, ``kp_yaw``,
``kd_yaw``), rate and odometry age limit from ``setpoint_control`` in
algorithms.yaml (``--algorithms`` or ALGORITHMS_CONFIG) and the force cap from
``guidance.max_thrust``; nothing is copied here.

Failure behaviour: zero force on every step without a target, without
odometry, or with odometry older than ``stale_after_s`` (or from the future)
in simulation time. A target or odometry message in another frame or with
non-finite values is ignored. Zero force does not stop the boat; it drifts.
"""
import argparse
import math
import os
import sys

import numpy as np

from njord_sim.configuration import (DEFAULT_PROFILE, _read, _resolve_algorithms, config_path, fully_actuated,
                                     thruster_table, validate_vessel, vessel_key)
from njord_sim.constants import BASE_FRAME, MAP_FRAME, SETPOINT_TOPIC


def wrap(angle):
    """Angle in rad wrapped to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def ned_pose(x_enu, y_enu, yaw_enu):
    """(x north m, y east m, psi rad clockwise from north) of a map ENU pose."""
    return y_enu, x_enu, wrap(math.pi / 2 - yaw_enu)


def frd_velocity(forward, left, yaw_rate_ccw):
    """(u, v, r): surge, sway to starboard (m/s) and clockwise yaw rate (rad/s)."""
    return forward, -left, -yaw_rate_ccw


def frd_thrusters(table, center_of_mass):
    """Thrusters as (x forward m, y starboard m, angle rad clockwise from forward).

    ``table`` is ``configuration.thruster_table`` (body forward/left/up);
    positions are taken relative to ``center_of_mass``, the point the wrench
    refers to.
    """
    return [(t['position_m'][0] - center_of_mass[0], -(t['position_m'][1] - center_of_mass[1]),
             -math.atan2(t['axis'][1], t['axis'][0])) for t in table]


def allocation(thrusters):
    """Matrix turning an FRD wrench (X N, Y N, N N*m) into thruster forces (N).

    The pseudo-inverse of the configuration matrix whose column for a
    thruster at (lx, ly) with angle a is (cos a, sin a, lx sin a - ly cos a),
    which gives the minimum-norm forces.
    """
    columns = [(math.cos(a), math.sin(a), lx * math.sin(a) - ly * math.cos(a)) for lx, ly, a in thrusters]
    return np.linalg.pinv(np.array(columns).T)


def dp_wrench(state, velocity, target, gains):
    """PD station-keeping wrench (X N, Y N, N N*m) in FRD.

    ``state`` and ``target`` are (x, y, psi) in NED, ``velocity`` (u, v, r) in
    FRD. The NED position error is rotated into the body frame.
    """
    north, east = target[0] - state[0], target[1] - state[1]
    c, s = math.cos(state[2]), math.sin(state[2])
    forward, starboard = c * north + s * east, -s * north + c * east
    return (gains['kp_position'] * forward - gains['kd_position'] * velocity[0],
            gains['kp_position'] * starboard - gains['kd_position'] * velocity[1],
            gains['kp_yaw'] * wrap(target[2] - state[2]) - gains['kd_yaw'] * velocity[2])


def thruster_forces(wrench, matrix, limits):
    """Forces in N for ``wrench``, all scaled by one factor to stay within ``limits``."""
    forces = matrix @ np.array(wrench)
    excess = max(abs(force) / limit for force, limit in zip(forces, limits))
    return [float(force / max(1.0, excess)) for force in forces]


def load(vessel_file, algorithms_file):
    """(thruster table, FRD allocation matrix, force limits N, setpoint_control settings)."""
    vessel = validate_vessel(_read(vessel_file))
    if not fully_actuated(vessel):
        raise ValueError(f'{vessel_file}: the stand-in needs a fully actuated vessel (surge, sway and yaw '
                         'independent), e.g. /config/vessels/munin_v0.yaml')
    algorithms = _resolve_algorithms(_read(algorithms_file), DEFAULT_PROFILE, vessel_key(vessel))
    table = thruster_table(vessel)
    cap = algorithms['guidance']['max_thrust']
    limits = [min(t['forward_limit_n'], t['reverse_limit_n'], cap) for t in table]
    matrix = allocation(frd_thrusters(table, vessel.get('center_of_mass_m', [0.0, 0.0, 0.0])))
    return table, matrix, limits, algorithms['setpoint_control']


def run(vessel_file, algorithms_file):
    """Spin the stand-in node until interrupted."""
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from std_msgs.msg import Float64

    from njord_sim.geometry import stamp_seconds, yaw_from_quaternion

    table, matrix, limits, settings = load(vessel_file, algorithms_file)

    class NedDpStandIn(Node):
        def __init__(self):
            super().__init__('ned_dp_standin', parameter_overrides=[Parameter('use_sim_time', value=True)])
            self.state = self.velocity = self.odom_stamp = None
            self.target = self.target_key = None
            self.driving = False
            self.thrusters = [self.create_publisher(Float64, t['topic'], 1) for t in table]
            self.create_subscription(Odometry, '/njord/odometry', self.on_odom, qos_profile_sensor_data)
            self.create_subscription(PoseStamped, SETPOINT_TOPIC, self.on_setpoint, QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE))
            self.create_timer(1.0 / settings['control_hz'], self.step)
            self.get_logger().info(f'Stand-in controller for {len(table)} thrusters; waiting for {SETPOINT_TOPIC}')

        def on_odom(self, msg):
            p, q, twist = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
            values = (p.x, p.y, q.x, q.y, q.z, q.w, twist.linear.x, twist.linear.y, twist.angular.z)
            if (msg.header.frame_id != MAP_FRAME or msg.child_frame_id != BASE_FRAME
                    or not all(math.isfinite(v) for v in values)):
                return
            self.state = ned_pose(p.x, p.y, yaw_from_quaternion(q))
            self.velocity = frd_velocity(twist.linear.x, twist.linear.y, twist.angular.z)
            self.odom_stamp = stamp_seconds(msg.header.stamp)

        def on_setpoint(self, msg):
            p, q = msg.pose.position, msg.pose.orientation
            if msg.header.frame_id != MAP_FRAME or not all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w)):
                return
            key = (stamp_seconds(msg.header.stamp), p.x, p.y, q.z, q.w)
            if key == self.target_key:
                return  # the evaluator's repeat of the active target
            self.target_key, self.target = key, ned_pose(p.x, p.y, yaw_from_quaternion(q))
            self.get_logger().info('New setpoint in NED: x=%.2f m (north), y=%.2f m (east), psi=%.1f deg'
                                   % (self.target[0], self.target[1], math.degrees(self.target[2])))

        def step(self):
            now = self.get_clock().now().nanoseconds * 1e-9
            fresh = self.odom_stamp is not None and 0.0 <= now - self.odom_stamp <= settings['stale_after_s']
            forces = [0.0] * len(table)
            if fresh and self.target is not None:
                forces = thruster_forces(dp_wrench(self.state, self.velocity, self.target, settings), matrix, limits)
            if (fresh and self.target is not None) != self.driving:
                self.driving = not self.driving
                self.get_logger().info('Sending thrust' if self.driving else
                                       'Zero thrust: no target or no fresh /njord/odometry')
            for publisher, force in zip(self.thrusters, forces):
                publisher.publish(Float64(data=force))

    rclpy.init()
    node = NedDpStandIn()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--vessel', default=os.environ.get('VESSEL_CONFIG'),
                        help='vessel file (default: VESSEL_CONFIG)')
    parser.add_argument('--algorithms', default=os.environ.get('ALGORITHMS_CONFIG') or config_path('algorithms.yaml'),
                        help='algorithms file (default: ALGORITHMS_CONFIG, else the packaged algorithms.yaml)')
    args = parser.parse_args()
    if not args.vessel:
        parser.error('give --vessel or set VESSEL_CONFIG')
    try:
        run(args.vessel, args.algorithms)
    except ValueError as error:
        sys.exit(str(error))


if __name__ == '__main__':
    main()
