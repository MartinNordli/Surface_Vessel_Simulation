"""Generate the real upstream WAM-V and validate sensor/actuator contracts.

These integration checks run in the ROS image. A CPU-only environment without
Jazzy, Gazebo or the VRX ament packages skips this class explicitly.
"""
import math
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"njord_sim"))


class VesselGenerationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            import sdformat14
            import yaml
            from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
        except ImportError as error:
            raise unittest.SkipTest(f"ROS/Gazebo integration dependencies unavailable: {error}")
        if not shutil.which("xacro") or not shutil.which("gz"):
            raise unittest.SkipTest("xacro and gz commands required")
        try:
            for package in ("wamv_description", "wamv_gazebo", "vrx_gazebo", "njord_sim"):
                get_package_share_directory(package)
        except PackageNotFoundError as error:
            raise unittest.SkipTest(f"VRX/ROS ament environment unavailable: {error}")
        from njord_sim.vessel import generate
        cls.temp = tempfile.TemporaryDirectory(prefix="njord-vessel-test-")
        cls.addClassCleanup(cls.temp.cleanup)
        cls.out = Path(cls.temp.name)
        urdf, cls.config = generate(cls.out)
        cls.urdf = ET.fromstring(urdf)
        cls.xml = ET.parse(cls.out/"wamv.sdf").getroot()
        cls.model = cls.xml.find("model")
        cls.sensors = {sensor.get("name"): sensor for sensor in cls.model.findall(".//sensor")}
        cls.bridges = yaml.safe_load((cls.out/"bridges.yaml").read_text())
        # sdformat14 raises on errors; a successful load returns None.
        cls.sdf = sdformat14.Root()
        cls.sdf.load(str(cls.out/"wamv.sdf"))

    def test_sdformat_model_and_physical_hull(self):
        self.assertEqual(self.sdf.model().name(), "wamv")
        self.assertGreater(self.sdf.model().link_count(), 3)
        plugins = self.model.findall("plugin")
        names = [plugin.get("name") for plugin in plugins]
        self.assertEqual(names.count("vrx::Surface"), 2)
        self.assertIn("vrx::SimpleHydrodynamics", names)
        self.assertEqual(names.count("gz::sim::systems::Thruster"), 2)
        self.assertIn("njord::ActuatorWatchdog", names)
        self.assertFalse(any("PosePublisher" in name or "DetachableJoint" in name for name in names))
        base = self.model.find("link[@name='wamv/base_link']")
        self.assertGreater(float(base.findtext("inertial/mass")), 100.)
        self.assertTrue(base.findall("collision"))

    def test_both_cameras_have_matching_optical_frames_and_ros_bridges(self):
        links = {link.get("name") for link in self.urdf.findall("link")}
        for side in ("left", "right"):
            name = f"front_{side}_camera_sensor"
            frame = f"wamv/front_{side}_camera_link_optical"
            root = f"/wamv/sensors/cameras/{name}"
            sensor = self.sensors[name]
            self.assertIn(frame, links)
            self.assertEqual(sensor.findtext("gz_frame_id"), frame)
            self.assertEqual(sensor.findtext("camera/optical_frame_id"), frame)
            self.assertEqual(sensor.findtext("camera/image/width"), str(self.config["camera_width"]))
            self.assertEqual(sensor.findtext("camera/image/height"), str(self.config["camera_height"]))
            self.assertEqual(sensor.findtext("topic"), root+"/image_raw")
            self.assertEqual(sensor.findtext("camera/camera_info_topic"), root+"/camera_info")
            for suffix, ros_type in (("image_raw", "sensor_msgs/msg/Image"), ("camera_info", "sensor_msgs/msg/CameraInfo")):
                self.assert_bridge(root+"/"+suffix, root+"/"+suffix, ros_type, "GZ_TO_ROS")
            # Optical +z must point forward and optical +x right in the camera body.
            joint = next(j for j in self.urdf.findall("joint") if j.find("child").get("link") == frame)
            roll, pitch, yaw = map(float, joint.find("origin").get("rpy").split())
            self.assertAlmostEqual(roll, -math.pi/2)
            self.assertAlmostEqual(pitch, 0.)
            self.assertAlmostEqual(yaw, -math.pi/2)

    def assert_bridge(self, gazebo, ros, ros_type, direction):
        matches = [bridge for bridge in self.bridges if bridge["ros_topic_name"] == ros]
        self.assertEqual(len(matches), 1, ros)
        bridge = matches[0]
        self.assertEqual(bridge["gz_topic_name"], gazebo)
        self.assertEqual(bridge["ros_type_name"], ros_type)
        self.assertEqual(bridge["direction"], direction)

    def test_lidar_geometry_and_scan_pointcloud_contract(self):
        sensor = self.sensors["lidar_wamv_sensor"]
        self.assertEqual(sensor.findtext("gz_frame_id"), "wamv/lidar_wamv_link")
        ray = sensor.find("ray")
        if ray is None:
            ray = sensor.find("lidar")
        self.assertEqual(int(ray.findtext("scan/horizontal/samples")), self.config["lidar_samples"])
        self.assertEqual(int(ray.findtext("scan/vertical/samples")), self.config["lidar_vertical_samples"])
        self.assertAlmostEqual(float(ray.findtext("range/max")), self.config["lidar_range"])
        self.assertAlmostEqual(float(ray.findtext("scan/horizontal/min_angle")), -math.pi)
        self.assertAlmostEqual(float(ray.findtext("scan/horizontal/max_angle")), math.pi)
        prefix = "/wamv/sensors/lidars/lidar_wamv_sensor"
        self.assert_bridge(prefix+"/scan", prefix+"/scan", "sensor_msgs/msg/LaserScan", "GZ_TO_ROS")
        self.assert_bridge(prefix+"/scan/points", prefix+"/points", "sensor_msgs/msg/PointCloud2", "GZ_TO_ROS")

    def test_sdformat_disables_degree_noise_before_metric_adapter(self):
        model = self.sdf.model()
        gps = None
        for i in range(model.link_count()):
            link = model.link_by_index(i)
            for j in range(link.sensor_count()):
                sensor = link.sensor_by_index(j)
                if sensor.name() == "navsat":
                    gps = sensor.nav_sat_sensor()
        self.assertIsNotNone(gps)
        self.assertAlmostEqual(gps.horizontal_position_noise().std_dev(), 0.0)
        self.assertAlmostEqual(gps.vertical_position_noise().std_dev(), 0.0)

    def test_only_guarded_actuator_input_is_bridged_to_gazebo(self):
        commands = [bridge for bridge in self.bridges if bridge["direction"] == "ROS_TO_GZ"]
        self.assertEqual(len(commands), 1)
        self.assertEqual(commands[0]["ros_topic_name"], "/njord/actuator_forces")
        self.assertEqual(commands[0]["ros_type_name"], "geometry_msgs/msg/Twist")
        self.assertFalse(any(bridge["ros_topic_name"] in ("/tf", "/tf_static") for bridge in self.bridges))


if __name__ == "__main__":
    unittest.main()
