"""Surface filtering preserves nearby external obstacles and unobserved water."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'njord_sim'))
from njord_sim.mesh_geometry import geometry_mesh
from njord_sim.self_geometry import SelfGeometry
from njord_sim.mapping_core import OccupancyMapper


def box(size, pose):
    vertices, faces = geometry_mesh({'type': 'box', 'size_m': size, 'pose': pose})
    return {'vertices': vertices, 'faces': faces}


class SelfGeometryTests(unittest.TestCase):
    def setUp(self):
        self.artifact = {'version': 1, 'frame': 'base_link', 'surfaces': [
            box([4., .4, .6], [0., -1., 0.]), box([4., .4, .6], [0., 1., 0.]),
            box([.2, 2., .2], [0., 0., .7])]}
        self.geometry = SelfGeometry(self.artifact, .05)

    def test_separate_surfaces_preserve_obstacles_between_above_and_beyond_hulls(self):
        points = [[1., 1.2, 0.], [1., -1., .32], [0., 0., .8],
                  [1., 0., 0.], [1., 1., .7], [2.2, 1., 0.], [1., 1.3, 0.]]
        self.assertEqual(self.geometry.contains_returns(points).tolist(),
                         [True, True, True, False, False, False, False])

    def test_roll_pitch_yaw_mount_transform_keeps_surface_decision(self):
        from njord_sim.geometry import transform_points, rotation_matrix
        # Independent composed Euler rotation to build a mounted lidar view.
        roll, pitch, yaw = .3, -.2, .8
        cr, sr, cp, sp, cy, sy = np.cos(roll/2), np.sin(roll/2), np.cos(pitch/2), np.sin(pitch/2), np.cos(yaw/2), np.sin(yaw/2)
        q = [sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
             cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy]
        translation = np.array([.7, -.3, 1.8])
        body = np.array([[1.,1.2,0.], [1.,0.,0.], [1.,1.,.7]])
        lidar = (body-translation)@rotation_matrix(q)
        self.assertEqual(self.geometry.contains_returns(transform_points(lidar, translation, q)).tolist(),
                         [True, False, False])

    def test_triangle_edges_and_margin(self):
        geometry = SelfGeometry({'version': 1, 'frame': 'base_link', 'surfaces': [
            {'vertices': [[0, 0, 0], [1, 0, 0], [0, 1, 0]], 'faces': [[0, 1, 2]]}]}, .05)
        self.assertEqual(geometry.contains_returns([[.2,.2,.049], [.2,.2,.051],
                                                   [.52,.52,0], [.6,.6,0]]).tolist(),
                         [True, False, True, False])

    def test_self_hits_do_not_clear_map_and_duplicate_stamp_cannot_add_evidence(self):
        mapper = OccupancyMapper(resolution=.1, size_m=10., origin=(-5.,-5.), inflation_m=0)
        raw = np.array([[1.,1.2,0.]])
        remaining = raw[~self.geometry.contains_returns(raw)]
        if len(remaining):
            mapper.update([0,0,0], remaining, [True]*len(remaining), 1.)
        self.assertTrue((mapper.grid(1.) == -1).all())
        self.assertTrue(mapper.update([0,0,0], [[1,0,0]], [True], 1.))
        before = mapper.grid(1.)
        self.assertFalse(mapper.update([0,0,0], [[2,0,0]], [False], 1.))
        np.testing.assert_array_equal(before, mapper.grid(1.))

    def test_export_visual_mesh_scale_pose_and_moving_frame(self):
        import xml.etree.ElementTree as ET
        from unittest.mock import patch
        from njord_sim.visual_geometry import export_visual_geometry
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            mesh = path/'vessel.dae'
            mesh.write_text('test fixture')
            model = ET.fromstring(f'''<model name="vessel"><link name="propeller_link">
                <visual name="blade"><pose>1 2 3 0 0 1.5707963267948966</pose>
                <geometry><mesh><uri>{mesh}</uri><scale>2 3 4</scale></mesh></geometry>
                </visual></link></model>''')
            imported = {'vertices': [[0,0,0],[1,0,0],[0,1,0]], 'faces': [[0,1,2]]}
            with patch('njord_sim.visual_geometry.subprocess.check_output', return_value=json.dumps(imported)):
                export_visual_geometry(model, path)
            artifact = json.loads((path/'self_geometry.json').read_text())
            surface = artifact['surfaces'][0]
            self.assertEqual(surface['frame'], 'propeller_link')
            # Scale in mesh axes BEFORE visual rotation/translation.
            np.testing.assert_allclose(surface['vertices'], [[1,2,3],[1,4,3],[-2,2,3]], atol=1e-14)
            geometry = SelfGeometry.load_frames(path/'self_geometry.json', .02)
            self.assertTrue(geometry['propeller_link'].contains_returns([[0,2.5,3]])[0])

    def test_invalid_missing_and_moving_surface_frames(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'geometry.json'
            with self.assertRaises(ValueError):
                SelfGeometry.load_frames(path, .05)
            self.artifact['surfaces'][-1]['frame'] = 'propeller_link'
            path.write_text(json.dumps(self.artifact))
            self.assertEqual(set(SelfGeometry.load_frames(path, .05)),
                             {'base_link', 'propeller_link'})
            self.artifact['surfaces'] = []
            path.write_text(json.dumps(self.artifact))
            with self.assertRaises(ValueError):
                SelfGeometry.load_frames(path, .05)


if __name__ == '__main__':
    unittest.main()
