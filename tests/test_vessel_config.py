"""CPU validation of runtime sensor configuration; does not test rendering."""
from pathlib import Path
import sys
import tempfile
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'njord_sim'))
from njord_sim.vessel import load_config

DEFAULTS = ROOT/'njord_sim/config/vessel.yaml'


class VesselConfigTests(unittest.TestCase):
    def load_override(self, override):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'override.yaml'
            path.write_text(yaml.safe_dump(override))
            return load_config(path, DEFAULTS)

    def test_partial_profile_inherits_noise_and_physical_limits(self):
        config = load_config(ROOT/'njord_sim/config/sensors_low_bandwidth.yaml', DEFAULTS)
        defaults = load_config(defaults_file=DEFAULTS)
        self.assertEqual(config['camera_width'], 320)
        self.assertEqual(config['imu_rate'], 50.)
        self.assertEqual(config['max_thrust_n'], defaults['max_thrust_n'])
        self.assertEqual(config['gps_horizontal_noise_m'], defaults['gps_horizontal_noise_m'])

    def test_rejects_typo_and_non_mapping(self):
        for value in ({'camera_rates': 10.}, [], None, 12):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.load_override(value)

    def test_rejects_nonphysical_and_ambiguous_values(self):
        for key, value in [('camera_width', 320.5), ('camera_height', True),
                           ('camera_rate', 0.), ('imu_rate', float('nan')),
                           ('gps_rate', float('inf')), ('lidar_samples', -1),
                           ('camera_noise_stddev', -0.1), ('gps_rate', '10'),
                           ('camera_horizontal_fov_rad', 3.2), ('max_thrust_n', 0.)]:
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                self.load_override({key: value})

    def test_zero_noise_is_a_valid_repeatable_configuration(self):
        config = self.load_override({'camera_noise_stddev': 0, 'gps_horizontal_noise_m': 0})
        self.assertEqual(config['camera_noise_stddev'], 0.)
        self.assertIsInstance(config['gps_horizontal_noise_m'], float)

    def test_resolved_config_can_be_loaded_without_change(self):
        defaults = load_config(defaults_file=DEFAULTS)
        self.assertEqual(self.load_override(defaults), defaults)


if __name__ == '__main__':
    unittest.main()
