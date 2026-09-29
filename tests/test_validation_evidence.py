"""Synthetic negative controls for evidence checks; no live simulator claim."""
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'validation'))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'njord_sim'))
from runtime_checks import StreamWindow, image_varies, valid_odometry


class RuntimeEvidenceTests(unittest.TestCase):
    def test_solid_red_and_padding_are_not_spatial_texture(self):
        pixels = np.tile([255, 0, 0], (4, 4, 1)).astype('uint8')
        msg = NS(encoding='rgb8', width=4, height=4, step=12, data=pixels.tobytes())
        self.assertFalse(image_varies(msg))
        pixels[0, 0] = [0, 0, 0]
        msg.data = pixels.tobytes()
        self.assertTrue(image_varies(msg))

    def test_all_pose_twist_fields_and_quaternion_are_checked(self):
        vector = lambda: NS(x=0., y=0., z=0.)
        msg = NS(header=NS(frame_id='map'), child_frame_id='base_link',
                 pose=NS(pose=NS(position=vector(), orientation=NS(x=0.,y=0.,z=0.,w=1.))),
                 twist=NS(twist=NS(linear=vector(), angular=vector())))
        self.assertTrue(valid_odometry(msg))
        for obj in (msg.pose.pose.position, msg.twist.twist.linear, msg.twist.twist.angular):
            for axis in ('x','y','z'):
                setattr(obj, axis, float('nan'))
                self.assertFalse(valid_odometry(msg))
                setattr(obj, axis, 0.)
        msg.pose.pose.orientation.w = 0.
        self.assertFalse(valid_odometry(msg))

    def test_contiguous_overlapping_ten_seconds_and_reset(self):
        window = StreamWindow()
        for i in range(41):
            now = 1+i*.25
            for name in window.names:
                window.observe(name, now, now, True)
            self.assertEqual(window.passed(now), i == 40)
        self.assertFalse(window.passed(12.))
        window.observe('left', 11., 11., True)  # Repeated acquisition time.
        self.assertFalse(window.passed(11.))
        window.observe('left', 11.25, 11.25, False)  # Missing acquisition TF.
        self.assertNotIn('left', window.windows)

if __name__ == '__main__':
    unittest.main()

class PhysicalOracleTests(unittest.TestCase):
    def test_force_moment_and_onset_acceleration(self):
        from physical_acceptance import wrench, diagonal_acceleration, response, acceptance
        load = wrench([{'position_m':[0.,1.,0.], 'yaw_deg':0.},
                       {'position_m':[0.,-1.,0.], 'yaw_deg':90.}], [10.,20.], [0.,0.,0.])
        np.testing.assert_allclose(load['force_n'], [10.,20.,0.], atol=1e-12)
        np.testing.assert_allclose(load['moment_nm'], [0.,0.,-10.], atol=1e-12)
        np.testing.assert_allclose(diagonal_acceleration(load,10.,[2.,3.,4.],[10.,10.,0.,0.,0.,6.]), [.5,1.,0.,0.,0.,-1.])
        self.assertAlmostEqual(response(10.,0.,1.,1.),10.*(1.-np.exp(-1.)))
        self.assertTrue(acceptance(10.19,10.,'N')['passed'])
        self.assertFalse(acceptance(10.21,10.,'N')['passed'])
        self.assertFalse(acceptance(float('nan'),10.,'N')['passed'])


class CalibrationTests(unittest.TestCase):
    def test_template_fails_without_measurements(self):
        import json
        from calibration_report import report
        root = Path(__file__).resolve().parents[1] / 'validation/calibration'
        result = report(json.loads((root/'dataset_manifest.template.json').read_text()), root)
        self.assertFalse(result['passed'])
        self.assertIn('incomplete holdout channel comparisons', result['failures'])
