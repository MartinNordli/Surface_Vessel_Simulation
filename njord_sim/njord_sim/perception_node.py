"""Fuse two calibrated RGB cameras and lidar at their original TF timestamps.

ROS node ``perception``: reference red/green buoy detector. Each camera image
is segmented by colour, lidar points are projected into it to get range, and
the resulting buoy positions are tracked per camera (see ``perception_core``).

Subscribes (best-effort sensor QoS):
    ``image_topics`` (sensor_msgs/Image): undistorted RGB images, one per
        camera, in the camera optical frame (x right, y down, z forward).
        Default: front left and right WAM-V cameras, ``.../image_raw``.
    ``camera_info_topics`` (sensor_msgs/CameraInfo): intrinsics per camera,
        in the same order as ``image_topics``.
    ``points_topic`` (sensor_msgs/PointCloud2): 3D lidar in the lidar frame.

Publishes:
    ``buoys_topic`` (vision_msgs/Detection3DArray, default ``/njord/buoys``):
        one array per processed image, in ``map_frame`` and stamped with the
        image acquisition time. It is published even when empty, because the
        mission node uses the array stamp as proof that perception is alive.
        Each detection has id ``"<camera index>/<track id>"``, the tracked
        buoy centre in metres, a fixed nominal 0.5 x 0.5 x 1.0 m box (not a
        measured size) and one hypothesis with ``class_id`` ``red``/``green``
        and the blob fill-ratio score.

Parameters:
    ``map_frame``: output frame (default ``map``, ENU).
    ``sync_tolerance_s``: max |image stamp - cloud stamp| for fusion [s].
    ``input_max_age_s``: images/clouds older than this (or stamped in the
        future) relative to simulation time are ignored [s].
    ``min_blob_area``: minimum blob area [pixels].
    ``min_observations``, ``track_ttl_s``: ``BuoyTracker`` confirmation count
        and track lifetime [s].

Timing and failure behaviour:
    All ages use the ``/clock`` simulation time. Every measurement is
    transformed with TF looked up at its own acquisition stamp, never at
    "now", and outputs keep the image stamp; nothing is re-stamped. If an
    input is stale, out of order, not synchronised, has no TF at its stamp,
    has invalid or distorted calibration, or cannot be decoded, that image
    is skipped and nothing is published for it. Downstream consumers then
    see the output go stale instead of receiving old data with a fresh stamp.

The ``map`` transform comes from the GPS/IMU estimator, so buoy positions
carry its localisation error. No ground truth is used.
"""
from collections import deque
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from njord_sim.geometry import stamp_seconds, transform_from_ros
from njord_sim.perception_core import associate_lidar, BuoyTracker, detect_blobs


class Perception(Node):
    """Camera/lidar buoy detector with one ``BuoyTracker`` per camera."""

    def __init__(self):
        super().__init__("perception")
        roots = ["/wamv/sensors/cameras/front_left_camera_sensor",
                 "/wamv/sensors/cameras/front_right_camera_sensor"]
        self.declare_parameters("", [
            ("image_topics", [root+"/image_raw" for root in roots]),
            ("camera_info_topics", [root+"/camera_info" for root in roots]),
            ("points_topic", "/wamv/sensors/lidars/lidar_wamv_sensor/points"),
            ("buoys_topic", "/njord/buoys"), ("map_frame", "map"),
            ("sync_tolerance_s", 0.12), ("input_max_age_s", 0.5),
            ("min_blob_area", 20), ("min_observations", 3), ("track_ttl_s", 2.0),
        ])
        self.p = lambda name: self.get_parameter(name).value
        images, infos = self.p("image_topics"), self.p("camera_info_topics")
        if len(images) != len(infos):
            raise ValueError("image_topics and camera_info_topics must have equal length")
        # Latest CameraInfo per camera index.
        self.info = {}
        # Recent lidar clouds as (acquisition stamp [s], (N, 3) points in map).
        self.clouds = deque(maxlen=8)
        self.trackers = {i: BuoyTracker(ttl=self.p("track_ttl_s"),
                                        minimum_observations=self.p("min_observations"))
                         for i in range(len(images))}
        # Stamp [s] of the last processed image per camera, to drop duplicates.
        self.last_images = {}
        self.bridge = CvBridge()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.pub = self.create_publisher(Detection3DArray, self.p("buoys_topic"), 5)
        self.create_subscription(PointCloud2, self.p("points_topic"), self.on_cloud,
                                 qos_profile_sensor_data)
        for index, (image, info) in enumerate(zip(images, infos)):
            # ``i=index`` binds the camera index now, not when the lambda runs.
            self.create_subscription(CameraInfo, info,
                                     lambda msg, i=index: self.info.__setitem__(i, msg),
                                     qos_profile_sensor_data)
            self.create_subscription(Image, image, lambda msg, i=index: self.on_image(msg, i),
                                     qos_profile_sensor_data)

    def on_cloud(self, msg):
        """Store a fresh lidar cloud, transformed into the map frame.

        The cloud is transformed with TF at its own acquisition stamp and kept
        in ``map``. Because the world frame does not move with the boat, it
        can later be re-projected into a camera using the camera pose at the
        image stamp, which compensates vessel motion between the two stamps.
        Clouds without a frame, older than ``input_max_age_s``, stamped in the
        future, or without TF at their stamp are dropped.
        """
        if not msg.header.frame_id:
            return
        stamp = stamp_seconds(msg.header.stamp)
        now = self.get_clock().now().nanoseconds*1e-9
        if not 0 <= now-stamp <= self.p("input_max_age_s"):
            return
        try:
            transform = self.tf.lookup_transform(self.p("map_frame"), msg.header.frame_id,
                                                 Time.from_msg(msg.header.stamp))
        except TransformException:
            return
        raw = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        raw = np.asarray(raw).reshape(-1, 3)
        raw = raw[np.isfinite(raw).all(axis=1)]
        self.clouds.append((stamp, transform_from_ros(raw, transform)))

    def on_image(self, msg, index):
        """Detect, fuse and track buoys in one image of camera ``index``.

        The image is skipped (nothing published) when it is stale or from the
        future, when no CameraInfo or cloud has arrived yet, when it is not
        newer than the previous image of this camera, when no cloud lies
        within ``sync_tolerance_s``, when the calibration is invalid or has
        distortion, or when TF at the image stamp or image decoding fails.
        """
        stamp = stamp_seconds(msg.header.stamp)
        now = self.get_clock().now().nanoseconds*1e-9
        if (not 0 <= now-stamp <= self.p("input_max_age_s") or index not in self.info
                or not self.clouds):
            return
        previous = self.last_images.get(index)
        if previous is not None and stamp <= previous:
            # A jump back of more than 1 s means the simulation was reset:
            # start this camera's tracks from scratch and process the image.
            # Otherwise it is a duplicate or out-of-order image: drop it.
            if stamp < previous-1.0:  # simulation reset
                self.trackers[index] = BuoyTracker(ttl=self.p("track_ttl_s"),
                                                   minimum_observations=self.p("min_observations"))
            else:
                return
        self.last_images[index] = stamp
        # Use the cloud closest in time; lidar and camera are not triggered
        # together, so a small offset is tolerated.
        cloud_stamp, world = min(self.clouds, key=lambda item: abs(item[0]-stamp))
        if abs(cloud_stamp-stamp) > self.p("sync_tolerance_s"):
            return
        info = self.info[index]
        # The image and its calibration must refer to the same optical frame,
        # and the focal lengths fx = k[0], fy = k[4] must be positive.
        if (not info.header.frame_id or msg.header.frame_id != info.header.frame_id
                or info.k[0] <= 0 or info.k[4] <= 0):
            return
        # This baseline expects the undistorted Gazebo camera model.
        if any(abs(value) > 1e-9 for value in info.d):
            self.get_logger().warn("Camera distortion requires rectified images",
                                   throttle_duration_sec=5.0)
            return
        try:
            # map -> camera optical frame at the image acquisition time.
            transform = self.tf.lookup_transform(info.header.frame_id, self.p("map_frame"),
                                                 Time.from_msg(msg.header.stamp))
            optical = transform_from_ros(world, transform)
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except (TransformException, CvBridgeError, ValueError, RuntimeError):
            return
        observations = associate_lidar(detect_blobs(bgr, self.p("min_blob_area")),
                                       optical, world, info.k)
        tracks = self.trackers[index].update(observations, stamp)
        output = Detection3DArray()
        # Keep the image acquisition stamp: consumers judge freshness by it.
        output.header.stamp, output.header.frame_id = msg.header.stamp, self.p("map_frame")
        for track in tracks:
            detection = Detection3D()
            detection.header = output.header
            detection.id = str(index)+"/"+track["id"]
            (detection.bbox.center.position.x, detection.bbox.center.position.y,
             detection.bbox.center.position.z) = map(float, track["position"])
            detection.bbox.center.orientation.w = 1.0
            # Fixed nominal buoy size; not measured from the data.
            detection.bbox.size.x = detection.bbox.size.y = 0.5
            detection.bbox.size.z = 1.0
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = track["color"]
            hypothesis.hypothesis.score = float(track["score"])
            hypothesis.pose.pose = detection.bbox.center
            detection.results.append(hypothesis)
            output.detections.append(detection)
        self.pub.publish(output)


def main():
    rclpy.init()
    node = Perception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
