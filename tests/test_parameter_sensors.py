"""Fixed-seed SI sensor statistics, covariance and stream independence."""
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'njord_sim'))
from njord_sim.sensor_noise_core import SensorNoise, covariance, noisy_fix, noisy_orientation


class SensorNoiseTests(unittest.TestCase):
    def test_ten_thousand_samples_per_axis_and_channel(self):
        noise = SensorNoise(2309)
        settings = {'orientation': (.015,)*3, 'gps': (.3,.3,.5),
                    'angular_velocity': (.009,)*3, 'linear_acceleration': (.021,)*3}
        for channel, stds in settings.items():
            with self.subTest(channel=channel):
                errors = np.asarray([noise.sample(channel, stds) for _ in range(10000)])
                if channel == 'orientation':
                    q = np.asarray([noisy_orientation((0,0,0,1), e) for e in errors])
                    angles = 2*np.arctan2(np.linalg.norm(q[:,:3], axis=1), q[:,3])
                    errors = q[:,:3]*angles[:,None]/np.linalg.norm(q[:,:3],axis=1)[:,None]
                elif channel == 'gps':
                    positions = np.asarray([noisy_fix(63.,10.,0.,e) for e in errors])
                    # Independent WGS84 ECEF projection, then north/east/up.
                    lat, lon = np.deg2rad(positions[:,:2]).T
                    e2 = (1/298.257223563)*(2-1/298.257223563)
                    n = 6378137/np.sqrt(1-e2*np.sin(lat)**2)
                    xyz = np.column_stack(((n+positions[:,2])*np.cos(lat)*np.cos(lon),
                                           (n+positions[:,2])*np.cos(lat)*np.sin(lon),
                                           (n*(1-e2)+positions[:,2])*np.sin(lat)))
                    lat0, lon0 = np.deg2rad([63.,10.])
                    n0 = 6378137/np.sqrt(1-e2*np.sin(lat0)**2)
                    baseline = [n0*np.cos(lat0)*np.cos(lon0), n0*np.cos(lat0)*np.sin(lon0), n0*(1-e2)*np.sin(lat0)]
                    axes = [[-np.sin(lat0)*np.cos(lon0), -np.sin(lat0)*np.sin(lon0), np.cos(lat0)],
                            [-np.sin(lon0), np.cos(lon0), 0],
                            [np.cos(lat0)*np.cos(lon0), np.cos(lat0)*np.sin(lon0), np.sin(lat0)]]
                    errors = (xyz-baseline)@np.asarray(axes).T
                self.assertTrue(np.all(np.abs(errors.mean(axis=0)) < 5*np.asarray(stds)/100))
                np.testing.assert_allclose(errors.std(axis=0), stds, rtol=.05)
                np.testing.assert_allclose(np.asarray(covariance(stds)).reshape(3,3), np.diag(np.square(stds)))

    def test_zero_noise_and_channel_rng_independence(self):
        channels = ['gps','orientation','angular_velocity','linear_acceleration']
        for target in channels:
            baseline, perturbed = SensorNoise(7), SensorNoise(7)
            for channel in channels:
                if channel != target:
                    for _ in range(113):
                        perturbed.sample(channel, (1,1,1))
            self.assertEqual(baseline.sample(target, (1,1,1)), perturbed.sample(target, (1,1,1)))
            self.assertEqual(perturbed.sample(target, (0,0,0)), (0,0,0))
        self.assertEqual(covariance((0,0,0)), [0.]*9)
        self.assertEqual(noisy_orientation((0,0,0,1), (0,0,0)), (0,0,0,1))
        self.assertEqual(noisy_fix(63.,10.,3.,(0,0,0)), (63.,10.,3.))


try:
    from test_sensor_runtime import HAS_ROS, local_node, SensorAdapter, Imu, NavSatFix
    from diagnostic_msgs.msg import DiagnosticStatus
except ImportError:
    HAS_ROS = False


@unittest.skipUnless(HAS_ROS, 'ROS unavailable')
class AdapterWiringTests(unittest.TestCase):
    def test_headers_raw_covariances_and_all_imu_channels(self):
        overrides = {'orientation_noise_rad': .02, 'angular_velocity_noise_rad_s': .04,
                     'linear_acceleration_noise_m_s2': .08}
        with local_node(SensorAdapter, overrides) as (node, clock, pubs):
            raw = Imu()
            raw.header.frame_id = 'imu_link'
            raw.orientation.w = 1.
            raw.angular_velocity.x = 2.
            raw.linear_acceleration.z = 9.81
            errors = []
            for i in range(10000):
                clock.seconds = 10.+i*.01
                raw.header.stamp = clock.now().to_msg()
                node.imu(raw)
                msg = pubs['/sensors/imu/data'].messages[-1]
                self.assertEqual(msg.header, raw.header)
                errors.append([msg.angular_velocity.x-2., msg.angular_velocity.y, msg.angular_velocity.z,
                               msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z-9.81])
            errors = np.asarray(errors)
            stds = np.array([.04]*3+[.08]*3)
            np.testing.assert_allclose(errors.std(axis=0), stds, rtol=.05)
            self.assertTrue((np.abs(errors.mean(axis=0)) < 5*stds/100).all())
            self.assertEqual(raw.angular_velocity.x, 2.)
            self.assertEqual(raw.linear_acceleration.z, 9.81)
            self.assertEqual(raw.orientation.w, 1.)
            np.testing.assert_allclose(msg.orientation_covariance, covariance((.02,)*3))
            np.testing.assert_allclose(msg.angular_velocity_covariance, covariance((.04,)*3))
            np.testing.assert_allclose(msg.linear_acceleration_covariance, covariance((.08,)*3))

    def test_probe_recovers_noise_about_nonidentity_orientation(self):
        import copy
        import importlib.util
        import math
        path = Path(__file__).resolve().parents[1]/'validation/sensor_parameter_probe.py'
        spec = importlib.util.spec_from_file_location('sensor_parameter_probe',path)
        probe = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(probe)
        raw = Imu()
        raw.orientation.x, raw.orientation.w = math.sin(.3), math.cos(.3)
        raw.angular_velocity.x, raw.linear_acceleration.z = 1.2, 9.81
        noise, collected = SensorNoise(567), []
        for _ in range(10000):
            errors = (noise.sample('orientation',(.015,)*3)+noise.sample('angular_velocity',(.009,)*3)+
                      noise.sample('linear_acceleration',(.021,)*3))
            msg = copy.deepcopy(raw)
            msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w = noisy_orientation(
                (raw.orientation.x,0,0,raw.orientation.w),errors[:3])
            for offset, field in ((3,'angular_velocity'),(6,'linear_acceleration')):
                for axis,error in zip(('x','y','z'),errors[offset:offset+3]):
                    vector = getattr(msg,field)
                    setattr(vector,axis,getattr(vector,axis)+error)
            recovered = probe.injected_errors('imu',raw,msg)
            np.testing.assert_allclose(recovered,errors,atol=1e-14)
            collected.append(recovered)
        self.assertTrue(probe.statistics(collected,[.015]*3+[.009]*3+[.021]*3,
                                         ['rad']*3+['rad/s']*3+['m/s^2']*3)['passed'])

    def test_duplicate_stale_clock_reset_and_clock_stall(self):
        from unittest.mock import patch
        with local_node(SensorAdapter) as (node, clock, pubs):
            raw = Imu()
            raw.orientation.w = 1.
            raw.header.stamp = clock.now().to_msg()
            node.imu(raw)
            receipt = node.received['imu']
            node.imu(raw)
            self.assertEqual(len(pubs['/sensors/imu/data'].messages), 1)
            self.assertEqual(node.received['imu'], receipt)
            clock.seconds += 1.
            node.imu(raw)
            self.assertEqual(len(pubs['/sensors/imu/data'].messages), 1)
            clock.seconds = 0.
            raw.header.stamp = clock.now().to_msg()
            node.imu(raw)
            self.assertEqual(len(pubs['/sensors/imu/data'].messages), 2)
            for name in ('gps', 'gps_projected', 'estimate'):
                node.received[name] = node.received['imu']
            node.status()
            self.assertEqual(pubs['/njord/navigation_status'].messages[-1].status[0].level, DiagnosticStatus.OK)
            with patch('njord_sim.sensor_adapter_node.time.monotonic', return_value=node.clock_progress_wall+1.):
                node.status()
                self.assertEqual(pubs['/njord/navigation_status'].messages[-1].status[0].level, DiagnosticStatus.ERROR)


if __name__ == '__main__':
    unittest.main()
