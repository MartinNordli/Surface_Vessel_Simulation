"""Configuration-to-generator contracts independent of ROS/Gazebo runtime."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET
import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'njord_sim'))
from njord_sim.configuration import (resolve_configuration, autonomy_parameters, sensor_timing,
    validate_reference_timing, convert_sensor_schema, _read, validate_vessel)
from njord_sim.constants import (BASE_FRAME, CAMERAS, IMU_TOPIC, LIDAR_FRAME, LIDAR_POINTS_TOPIC, camera_frame,
                                 camera_topic)
from njord_sim.vessel import configure_thrusters, remove_sensor_noise
from njord_sim.visual_geometry import export_visual_geometry
ROOT = Path(__file__).resolve().parents[1]


class Contracts(unittest.TestCase):
    def test_text_configuration_uses_the_shared_ros_names(self):
        config = ROOT/'njord_sim/config'
        localization = yaml.safe_load((config/'localization.yaml').read_text())
        for ekf in ('ekf_local', 'ekf_global'):
            parameters = localization[ekf]['ros__parameters']
            self.assertEqual(parameters['base_link_frame'], BASE_FRAME)
            self.assertEqual(parameters['imu0'], IMU_TOPIC)
        rviz = (config/'njord.rviz').read_text()
        for name in (BASE_FRAME, LIDAR_FRAME, LIDAR_POINTS_TOPIC,
                     *(camera_frame(camera, optical=True) for camera in CAMERAS),
                     *(camera_topic(camera, 'image_raw') for camera in CAMERAS)):
            self.assertIn(name, rviz)
        for path in sorted(config.rglob('*')):
            if path.suffix in ('.yaml', '.rviz', '.xacro'):
                with self.subTest(path=path.name):
                    # Covers /wamv/... topics and wamv/... frames, not vessels/wamv.yaml.
                    self.assertNotIn('wamv/', path.read_text())

    def resolved(self):
        return resolve_configuration(ROOT/'njord_sim/config/vessels/wamv.yaml', ROOT/'scenarios/reference.yaml', ROOT/'njord_sim/config/algorithms.yaml')

    def test_range_and_covariance_configuration(self):
        r = self.resolved()
        for distance in (20., 120.):
            r['vessel']['settings']['lidar_range'] = distance
            p = autonomy_parameters(r)
            self.assertEqual(p['mapper']['max_range_m'], distance)
            self.assertEqual(p['sensor_adapter']['angular_velocity_noise_rad_s'], .009)
            self.assertEqual(p['mapper']['self_filter_margin_m'], .02)

    def test_legacy_sensor_migration_explicit(self):
        old = _read(ROOT/'njord_sim/config/vessels/wamv.yaml')
        old['schema_version'] = 1
        for key in ('imu_angular_velocity_noise_rad_s', 'imu_linear_acceleration_noise_m_s2'):
            del old['settings'][key]
        converted = convert_sensor_schema(old)
        self.assertEqual(converted['schema_version'], 2)
        self.assertEqual(converted['settings']['imu_angular_velocity_noise_rad_s'], .009)
        self.assertEqual(validate_vessel(old), converted)
        self.assertNotIn('imu_angular_velocity_noise_rad_s', old['settings'])

    def test_timing_quantization_and_reference_only_limits(self):
        r = self.resolved()
        periods = sensor_timing(r['vessel']['settings'], .004)
        self.assertAlmostEqual(periods['imu'], .012)
        p = autonomy_parameters(r)
        components = dict(autonomy='reference', mapping='reference', perception='reference', controller='reference')
        validate_reference_timing(p, periods, r['algorithms']['navigation'], components)
        slow = dict(r['vessel']['settings'], lidar_rate=1.)
        periods = sensor_timing(slow, .004)  # valid simulator source
        with self.assertRaisesRegex(ValueError, 'requires'):
            validate_reference_timing(p, periods, r['algorithms']['navigation'], components)
        validate_reference_timing(p, periods, r['algorithms']['navigation'], {'autonomy':'external'})
        with self.assertRaisesRegex(ValueError, 'physics_step'):
            sensor_timing(dict(slow, imu_rate=1000), .004)

    def model(self):
        m = ET.Element('model')
        for side in ('left', 'right'):
            joint = side+'_engine_propeller_joint'
            ET.SubElement(m,'link',name=side)
            j = ET.SubElement(m,'joint',name=joint)
            ET.SubElement(j,'child').text = side
            p = ET.SubElement(m,'plugin',name='gz::sim::systems::Thruster',filename='gz-sim-thruster-system')
            for k,v in dict(namespace='wamv',joint_name=joint,topic='thrusters/'+side+'/thrust',velocity_control='true').items():
                ET.SubElement(p,k).text=v
        return m

    def test_force_limits_and_upstream_drift(self):
        for limit in (125.,500.,3000.):
            m = self.model()
            configure_thrusters(m,limit)
            for p in m.findall('plugin'):
                self.assertEqual(float(p.findtext('max_thrust_cmd')),limit)
                self.assertEqual(float(p.findtext('min_thrust_cmd')),-limit)
                self.assertEqual(p.findtext('use_angvel_cmd'),'false')
        m = self.model()
        ET.SubElement(m.find('plugin'),'use_angvel_cmd').text='true'
        with self.assertRaises(ValueError): configure_thrusters(m,500)
        m = self.model();m.remove(m.find('joint'))
        with self.assertRaises(ValueError): configure_thrusters(m,500)

    def test_upstream_noise_bias_removed(self):
        sensor = ET.fromstring('<sensor><imu><angular_velocity><x><noise type="gaussian"><stddev>.1</stddev><bias_stddev>1</bias_stddev></noise></x></angular_velocity></imu></sensor>')
        remove_sensor_noise(sensor)
        self.assertEqual(sensor.findall('.//noise'),[])

    def test_visible_components_and_frames_preserved(self):
        m = ET.fromstring('<model><link name="base_link"><visual name="left"><pose>0 1 0 0 0 0</pose><geometry><box><size>4 .4 .4</size></box></geometry></visual><visual name="right"><pose>0 -1 0 0 0 0</pose><geometry><box><size>4 .4 .4</size></box></geometry></visual></link><link name="propeller"><visual name="blade"><geometry><box><size>.1 .2 .3</size></box></geometry></visual></link></model>')
        with tempfile.TemporaryDirectory() as tmp:
            export_visual_geometry(m,tmp)
            d = json.loads((Path(tmp)/'self_geometry.json').read_text())
        self.assertEqual(len(d['surfaces']),3)
        self.assertEqual(d['surfaces'][2]['frame'],'propeller')
        self.assertTrue(all(v[1]>.7 for v in d['surfaces'][0]['vertices']))
        self.assertTrue(all(v[1]<-.7 for v in d['surfaces'][1]['vertices']))

if __name__ == '__main__': unittest.main()
