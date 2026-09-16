"""Analytic CPU/model checks; these do not establish marine fidelity."""

from pathlib import Path
import copy
import math
import shutil
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "njord_sim"))
from njord_sim.physics_core import (
    actuator_response,
    damping_wrench,
    thruster_wrench,
    wind_coefficients,
)
from njord_sim.njord_model import generate


class PhysicsTests(unittest.TestCase):
    def test_force_arm_and_asymmetric_com(self):
        force, torque = thruster_wrench(100, (-1, 0.6, -0.1), (1, 0, 0), (0.1, 0.2, 0))
        self.assertEqual(force, (100, 0, 0))
        self.assertAlmostEqual(torque[2], -40)
        self.assertAlmostEqual(torque[1], -10)

    def test_expiry_and_response(self):
        self.assertAlmostEqual(
            actuator_response(0, 100, 1, 1, 50, 20), 50 * (1 - math.exp(-1))
        )
        self.assertAlmostEqual(
            actuator_response(100, 100, 1, 1, 50, 20, False), 100 * math.exp(-1)
        )
        self.assertEqual(actuator_response(0, float("nan"), 1, 0, 50, 20), 0)
        self.assertEqual(actuator_response(0, -100, 1, 0, 50, 20), -20)

    def test_damping_dissipates(self):
        v = (-2, 3, -4, 0.3, -0.4, 0.5)
        w = damping_wrench(v, [1] * 6, [2] * 6)
        self.assertLess(sum(a * b for a, b in zip(v, w)), 0)

    def test_periodic_wind(self):
        rows = [
            {"angle_deg": a, "cx": x, "cy": 0, "cn": 0} for a, x in [(0, 1), (180, -1)]
        ]
        self.assertEqual(wind_coefficients(-90, rows), wind_coefficients(270, rows))
        self.assertAlmostEqual(
            wind_coefficients(359, rows)[0], wind_coefficients(1, rows)[0]
        )

    def test_cpp_hydrostatics(self):
        if not shutil.which("c++"):
            self.skipTest("C++ compiler unavailable")
        with tempfile.TemporaryDirectory() as d:
            binary = str(Path(d) / "hydro")
            subprocess.run(
                [
                    "c++",
                    "-std=c++17",
                    "-I",
                    str(ROOT / "njord_gz_plugins/include"),
                    str(ROOT / "njord_gz_plugins/test/hydrostatics_test.cc"),
                    "-o",
                    binary,
                ],
                check=True,
                capture_output=True,
            )
            subprocess.run([binary], check=True, capture_output=True)

    def test_multiple_convex_buoyancy_volumes(self):
        from njord_sim.mesh_geometry import geometry_mesh

        vessel = yaml.safe_load((ROOT / "njord_sim/config/njord_v1.yaml").read_text())
        box = copy.deepcopy(vessel["geometry"]["buoyancy"])
        box["size_m"] = [3, 0.3, 0.6]
        vertices, faces = geometry_mesh(box)
        hulls = []
        for y in (-0.7, 0.7):
            hulls.append(
                {
                    "type": "mesh",
                    "vertices": vertices,
                    "faces": faces,
                    "pose": [0, y, 0, 0, 0, 0],
                    "uri": "/unused-test-resource.obj",
                }
            )
        vessel["geometry"]["buoyancy"] = hulls
        resolved = {
            "vessel": vessel,
            "scenario": {
                "environment": {
                    "water_density_kg_m3": 1000,
                    "water_level_m": 0,
                    "wind_velocity_enu": [0, 0, 0],
                    "current_velocity_enu": [0, 0, 0],
                }
            },
        }
        with tempfile.TemporaryDirectory() as directory:
            generate(directory, resolved)
            root = ET.parse(Path(directory) / "wamv.sdf")
            volumes = root.findall("model/plugin/buoyancy_volume")
            self.assertEqual(len(volumes), 2)
            for volume, y in zip(volumes, (-0.7, 0.7)):
                points = [
                    list(map(float, v.text.split())) for v in volume.findall("vertex")
                ]
                self.assertAlmostEqual(sum(p[1] for p in points) / len(points), y)
                self.assertEqual(len(volume.findall("triangle")), 12)

    def test_generated_model(self):
        vessel = yaml.safe_load((ROOT / "njord_sim/config/njord_v1.yaml").read_text())
        vessel["center_of_mass_m"] = [0.1, 0.2, 0.05]
        resolved = {
            "vessel": vessel,
            "scenario": {
                "environment": {
                    "water_density_kg_m3": 1000,
                    "water_level_m": 0,
                    "wind_velocity_enu": [0, 0, 0],
                    "current_velocity_enu": [0.5, 0, 0],
                }
            },
        }
        with tempfile.TemporaryDirectory() as d:
            urdf, config = generate(d, resolved)
            root = ET.parse(Path(d) / "wamv.sdf")
            model = root.find("model")
            link = model.find("link")
            self.assertEqual(float(link.findtext("inertial/mass")), 180)
            self.assertEqual(link.findtext("inertial/pose"), "0.1 0.2 0.05 0 0 0")
            self.assertEqual(len(model.findall("plugin")), 3)
            hydro = model.find("plugin[@name='gz::sim::systems::Hydrodynamics']")
            self.assertEqual(hydro.findtext("disable_added_mass"), "true")
            self.assertEqual(hydro.findtext("xU"), "-40.0")
            self.assertIsNotNone(link.find("inertial/fluid_added_mass/xx"))
            self.assertEqual(len(link.findall("sensor")), 5)
            self.assertIn("wamv/front_left_camera_link_optical", urdf)
            self.assertEqual(config["thruster_separation_m"], 1.2)
            if shutil.which("gz"):
                result = subprocess.run(
                    ["gz", "sdf", "-k", str(Path(d) / "wamv.sdf")],
                    capture_output=True,
                    text=True,
                )
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                printed = subprocess.run(
                    ["gz", "sdf", "-p", str(Path(d) / "wamv.sdf")],
                    capture_output=True,
                    text=True,
                    check=True,
                )
                parsed = ET.fromstring(printed.stdout)
                self.assertEqual(
                    float(parsed.findtext("model/link/inertial/fluid_added_mass/xx")),
                    20.0,
                )
                self.assertNotIn("attribute", printed.stderr)


if __name__ == "__main__":
    unittest.main()
