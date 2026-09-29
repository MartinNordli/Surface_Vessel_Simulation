"""Explicit truth mode: simulator ground truth as the navigation state.

Started by dstar_demo.launch.py instead of the two robot_localization EKFs
when ``state_source:=truth`` (Compose ``STATE_SOURCE=truth``). A controller
under test then receives the exact simulated pose and velocity on the same
topic, type and frames as the estimate, so it runs unchanged in both modes.
Truth mode is recorded in autonomy_config.json and run_metrics.json; results
from it say nothing about estimator performance.

Subscribes:
    ``/sim/ground_truth/odometry`` (``nav_msgs/Odometry``, sensor QoS):
        Gazebo odometry of ``base_link`` in ``map``; the twist is in the
        body frame.

Publishes:
    ``/njord/odometry`` (``nav_msgs/Odometry``): the same message, unchanged:
        same header stamp (simulation time of the pose), frames, pose, twist
        and covariance. It is never re-stamped.
    TF ``odom`` -> ``base_link`` from each relayed pose, and a static
        identity ``map`` -> ``odom``, so ``map`` -> ``base_link`` is the
        ground truth pose (the EKFs, which own these transforms in estimate
        mode, are not running).

Failure behaviour:
    A message with the wrong frames, a non-finite value or a non-unit
    quaternion is dropped. Nothing is republished to fill a gap, so
    ``/njord/odometry`` goes stale, the sensor adapter reports navigation
    ERROR and the command guard zeroes thrust.
"""
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from njord_sim.constants import BASE_FRAME, GROUND_TRUTH_TOPIC
from njord_sim.geometry import valid_odometry


class TruthRelay(Node):
    """Republish ground-truth odometry as ``/njord/odometry`` with its TF."""

    def __init__(self):
        super().__init__('truth_relay')
        self.declare_parameters('', [
            ('input_topic', GROUND_TRUTH_TOPIC), ('output_topic', '/njord/odometry'),
            ('map_frame', 'map'), ('odom_frame', 'odom'), ('base_frame', BASE_FRAME),
        ])
        self.p = lambda name: self.get_parameter(name).value
        self.pub = self.create_publisher(Odometry, self.p('output_topic'), 10)
        self.tf = TransformBroadcaster(self)
        # map and odom coincide in truth mode: there is no drifting estimate.
        identity = TransformStamped()
        identity.header.stamp = self.get_clock().now().to_msg()
        identity.header.frame_id, identity.child_frame_id = self.p('map_frame'), self.p('odom_frame')
        identity.transform.rotation.w = 1.0
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform(identity)
        self.create_subscription(Odometry, self.p('input_topic'), self.relay, qos_profile_sensor_data)

    def relay(self, msg):
        """Forward one valid ground-truth message and its odom -> base TF."""
        if not valid_odometry(msg, self.p('map_frame'), self.p('base_frame')):
            return
        self.pub.publish(msg)
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp  # acquisition time, not now
        transform.header.frame_id, transform.child_frame_id = self.p('odom_frame'), self.p('base_frame')
        p = msg.pose.pose.position
        transform.transform.translation.x, transform.transform.translation.y = p.x, p.y
        transform.transform.translation.z = p.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.tf.sendTransform(transform)


def main():
    rclpy.init()
    node = TruthRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
