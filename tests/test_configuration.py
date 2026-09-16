"""Configuration failures must occur before Gazebo or ROS starts."""
import copy
from pathlib import Path
import sys
import tempfile
import unittest
import yaml

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'njord_sim'))
from njord_sim.configuration import (resolve_configuration, validate_vessel,
    convert_legacy_scenario, autonomy_parameters, legacy_vessel_settings)

ROOT=Path(__file__).resolve().parents[1]
CONFIG=ROOT/'njord_sim/config'

class ConfigurationTests(unittest.TestCase):
    def resolve(self, **kwargs):
        return resolve_configuration(CONFIG/'njord_v1.yaml', ROOT/'scenarios/reference.yaml', CONFIG/'algorithms.yaml',**kwargs)

    def test_complete_reproducible_resolution(self):
        a=self.resolve(seed=42); b=self.resolve(seed=42)
        self.assertEqual(a,b)
        self.assertEqual(a['vessel']['calibration']['status'],'uncalibrated')
        self.assertEqual(a['scenario']['hull'],{'length_m':3.,'beam_m':1.5})
        self.assertEqual(len(a['resources']),3)
        public=autonomy_parameters(a)
        self.assertNotIn('scenario',public)
        self.assertTrue(public['guidance']['physical_allocation'])
        self.assertEqual(public['guidance']['thruster_positions'],[-1.2,.6,-.1,-1.2,-.6,-.1])
        self.assertEqual(legacy_vessel_settings(a)['max_thrust_n'],500.)

    def test_unknown_and_missing_physics(self):
        original=yaml.safe_load((CONFIG/'njord_v1.yaml').read_text())
        for key,value in [('typo',1),('mass_kg',float('nan')),('mass_kg',True),('mass_kg',0.)]:
            bad=copy.deepcopy(original);bad[key]=value
            with self.assertRaises(ValueError): validate_vessel(bad)
        bad=copy.deepcopy(original);del bad['hydrodynamics']
        with self.assertRaises(ValueError):validate_vessel(bad)

    def test_inertia_triangle_and_positive_definite(self):
        for values in ([1.,1.,3.],[1.,1.,-1.]):
            bad=yaml.safe_load((CONFIG/'njord_v1.yaml').read_text())
            bad['inertia_kg_m2'].update(dict(zip(('ixx','iyy','izz'),values)))
            with self.assertRaisesRegex(ValueError,'inertia'):validate_vessel(bad)

    def test_negative_damping_invalid_axes_open_mesh_rejected(self):
        original=yaml.safe_load((CONFIG/'njord_v1.yaml').read_text())
        cases=[]
        bad=copy.deepcopy(original);bad['hydrodynamics']['linear_damping'][0]=-1;cases.append(bad)
        bad=copy.deepcopy(original);bad['thrusters'][0]['axis']=[0,0,0];cases.append(bad)
        bad=copy.deepcopy(original);bad['geometry']['buoyancy']['type']='mesh';cases.append(bad)
        for bad in cases:
            with self.assertRaises(ValueError):validate_vessel(bad)

    def test_insufficient_displacement_and_operating_limit(self):
        for key,value,message in [('mass_kg',10000.,'displacement'),('thrusters',None,'capacity')]:
            vessel=yaml.safe_load((CONFIG/'njord_v1.yaml').read_text())
            if key=='thrusters':vessel[key][0]['forward_limit_n']=100.
            else:vessel[key]=value
            with tempfile.TemporaryDirectory() as d:
                p=Path(d)/'vessel.yaml';p.write_text(yaml.safe_dump(vessel))
                with self.assertRaisesRegex(ValueError,message):resolve_configuration(p,ROOT/'scenarios/reference.yaml',CONFIG/'algorithms.yaml')

    def test_waves_rejected_for_njord(self):
        with self.assertRaisesRegex(ValueError,'waves'):self.resolve(environment='moderate')

    def test_named_wamv_reference_and_legacy_conversion(self):
        named=resolve_configuration(CONFIG/'wamv_reference.yaml',ROOT/'scenarios/reference.yaml',CONFIG/'algorithms.yaml',environment='moderate')
        legacy=resolve_configuration(CONFIG/'vessel.yaml',ROOT/'scenarios/reference.yaml',CONFIG/'algorithms.yaml',environment='moderate',legacy_vessel=True)
        self.assertEqual(named['vessel'],legacy['vessel'])
        self.assertEqual(named['scenario'],legacy['scenario'])
        with self.assertRaises(ValueError):resolve_configuration(CONFIG/'vessel.yaml',ROOT/'scenarios/reference.yaml',CONFIG/'algorithms.yaml')

    def test_enu_current_and_direction_conflict(self):
        scenario=convert_legacy_scenario(yaml.safe_load((ROOT/'scenarios/reference.yaml').read_text()))
        scenario.pop('hull')
        scenario['environments']['calm'].update(current_speed_mps=2.,current_direction_to_deg_enu=90.)
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'scenario.yaml';p.write_text(yaml.safe_dump(scenario))
            result=resolve_configuration(CONFIG/'njord_v1.yaml',p,CONFIG/'algorithms.yaml')
            self.assertAlmostEqual(result['scenario']['environment']['current_velocity_enu'][1],2.)
        legacy=yaml.safe_load((ROOT/'scenarios/reference.yaml').read_text())
        legacy['environments']['calm']['wind_direction_to_deg_enu']=45.
        with self.assertRaises(ValueError):convert_legacy_scenario(legacy)

    def test_public_sensor_authority_and_operating_guard(self):
        result=self.resolve(seed=43)
        public=autonomy_parameters(result)
        self.assertEqual(public['sensor_adapter']['seed'],43)
        self.assertEqual(public['sensor_adapter']['gps_xy_std_m'],result['vessel']['sensors']['settings']['gps_horizontal_noise_m'])
        reference=resolve_configuration(CONFIG/'wamv_reference.yaml',ROOT/'scenarios/reference.yaml',CONFIG/'algorithms.yaml')
        reference['algorithms']['guidance']['max_thrust']=123.
        params=autonomy_parameters(reference)
        self.assertEqual(params['command_guard']['max_thrust'],123.)
        self.assertEqual(params['command_guard']['forward_limits'],[500.,500.])

    def test_versioned_hull_conflict_rejected_legacy_replaced(self):
        scenario=convert_legacy_scenario(yaml.safe_load((ROOT/'scenarios/reference.yaml').read_text()))
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'scenario.yaml';p.write_text(yaml.safe_dump(scenario))
            with self.assertRaisesRegex(ValueError,'hull conflicts'):
                resolve_configuration(CONFIG/'njord_v1.yaml',p,CONFIG/'algorithms.yaml')
            scenario['hull']={'length_m':3.,'beam_m':1.5}
            p.write_text(yaml.safe_dump(scenario))
            resolved=resolve_configuration(CONFIG/'njord_v1.yaml',p,CONFIG/'algorithms.yaml')
            self.assertEqual(resolved['scenario']['hull'],scenario['hull'])
        self.assertEqual(self.resolve()['scenario']['hull'],{'length_m':3.,'beam_m':1.5})

    def test_lidar_max_exceeds_fixed_minimum(self):
        for value in (0.1,0.2):
            vessel=yaml.safe_load((CONFIG/'njord_v1.yaml').read_text())
            vessel['sensors']['settings']['lidar_range']=value
            with self.assertRaisesRegex(ValueError,'physical minimum'):validate_vessel(vessel)

    def test_duplicate_yaml_keys(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'bad.yaml';p.write_text('schema_version: 1\nschema_version: 1\n')
            with self.assertRaisesRegex(ValueError,'duplicate'):resolve_configuration(p,ROOT/'scenarios/reference.yaml',CONFIG/'algorithms.yaml')

    def test_seed_rejects_truncation_and_boolean(self):
        for seed in (True,1.5,0):
            with self.assertRaises(ValueError):self.resolve(seed=seed)

class MeshConfigurationTests(unittest.TestCase):
    def setUp(self):
        self.vessel=yaml.safe_load((CONFIG/'njord_v1.yaml').read_text())

    @staticmethod
    def write_obj(path,vertices,faces):
        path.write_text(''.join('v '+' '.join(map(str,v))+'\n' for v in vertices)+
                        ''.join('f '+' '.join(str(i+1) for i in f)+'\n' for f in faces))

    def test_mesh_roundtrip_checksum_footprint_and_volume(self):
        from njord_sim.mesh_geometry import geometry_mesh
        vertices,faces=geometry_mesh(self.vessel['geometry']['collision'])
        with tempfile.TemporaryDirectory() as d:
            path=Path(d);mesh=path/'hull.obj'
            self.write_obj(mesh,vertices,faces)
            for k in ('visual','collision','buoyancy'):
                self.vessel['geometry'][k]={'type':'mesh','uri':'hull.obj','pose':[0.,0.,0.,0.,0.,0.]}
            source=path/'vessel.yaml';source.write_text(yaml.safe_dump(self.vessel))
            result=resolve_configuration(source,ROOT/'scenarios/reference.yaml',CONFIG/'algorithms.yaml')
            self.assertIn(str(mesh),result['resources'])
            self.assertAlmostEqual(result['vessel']['geometry']['buoyancy']['volume_m3'],2.7)
            self.assertEqual(result['vessel']['hull'],{'length_m':3.,'beam_m':1.5})
            self.assertEqual(result['vessel']['geometry']['collision']['vertices'],vertices)

    def test_analytic_tetrahedron_volume(self):
        from njord_sim.mesh_geometry import validate_convex_mesh
        vertices=[[0.,0.,0.],[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]]
        faces=[[0,2,1],[0,1,3],[0,3,2],[1,2,3]]
        self.assertAlmostEqual(validate_convex_mesh(vertices,faces),1/6)
        translated=[[v[0]+10000.,v[1]-10000.,v[2]+5.] for v in vertices]
        self.assertAlmostEqual(validate_convex_mesh(translated,faces),1/6)

    def test_open_inverted_concave_and_duplicate_mesh_rejected(self):
        from njord_sim.mesh_geometry import validate_convex_mesh,geometry_mesh
        vertices,faces=geometry_mesh(self.vessel['geometry']['collision'])
        bad_vertices=copy.deepcopy(vertices);bad_vertices[6]=[0.,0.,0.]
        cases=[(vertices,faces[:-1]),(vertices,[f[::-1] for f in faces]),
               (bad_vertices,faces),(vertices,faces+[faces[0]])]
        for v,f in cases:
            with self.assertRaises(ValueError):validate_convex_mesh(v,f)

    def test_coplanar_overlap_detector(self):
        from njord_sim.mesh_geometry import _overlap_area
        a=[[0.,0.,0.],[2.,0.,0.],[0.,2.,0.]]
        b=[[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]]
        self.assertAlmostEqual(_overlap_area(a,b,[0.,0.,1.]),0.5)
        b=[[0.,0.,0.],[-1.,0.,0.],[0.,-1.,0.]]
        self.assertEqual(_overlap_area(a,b,[0.,0.,1.]),0.)

    def test_separate_hulls_and_overlapping_volumes(self):
        first={'type':'box','size_m':[3.,0.4,0.6],'pose':[0.,-0.6,0.,0.,0.,0.]}
        second=copy.deepcopy(first);second['pose'][1]=0.6
        self.vessel['geometry']['buoyancy']=[first,second]
        validate_vessel(copy.deepcopy(self.vessel))
        second['pose'][1]=-0.3
        with self.assertRaisesRegex(ValueError,'overlap'):validate_vessel(self.vessel)

    def test_missing_resource_rejected(self):
        self.vessel['geometry']['buoyancy']={'type':'mesh','uri':'/nonexistent/hull.obj','pose':[0.]*6}
        with self.assertRaisesRegex(ValueError,'missing'):validate_vessel(self.vessel)

    def test_wamv_rejects_ignored_geometry_and_environment(self):
        vessel=yaml.safe_load((CONFIG/'wamv_reference.yaml').read_text())
        vessel['legacy']['thruster_separation_m']=3.
        with self.assertRaisesRegex(ValueError,'pinned'):validate_vessel(vessel)
        for field,value in [('current_speed_mps',1.),('water_density_kg_m3',1025.),('water_level_m',1.)]:
            scenario=convert_legacy_scenario(yaml.safe_load((ROOT/'scenarios/reference.yaml').read_text()))
            scenario['environments']['calm'][field]=value
            with tempfile.TemporaryDirectory() as d:
                p=Path(d)/'scenario.yaml';p.write_text(yaml.safe_dump(scenario))
                with self.assertRaisesRegex(ValueError,'does not support'):
                    resolve_configuration(CONFIG/'wamv_reference.yaml',p,CONFIG/'algorithms.yaml')

    def test_splayed_axes_and_singular_allocation(self):
        import math
        a=math.radians(5)
        self.vessel['thrusters'][0]['axis']=[math.cos(a),math.sin(a),0.]
        validate_vessel(copy.deepcopy(self.vessel))
        for t in self.vessel['thrusters']:
            x,y,_=t['position_m'];n=math.hypot(x,y)
            t['axis']=[-x/n,-y/n,0.]
        with self.assertRaisesRegex(ValueError,'independently control'):validate_vessel(self.vessel)


if __name__=='__main__':unittest.main()
