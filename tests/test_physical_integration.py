"""Public configuration and physical allocation contract regressions."""
import copy
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'njord_sim'))
from njord_sim.control_core import allocate_thrusters
from njord_sim.run_manifest import write_manifest, publish_ready, wait_ready, verify_public_handoff, freeze_resources, sha256
from njord_sim.configuration import resolve_configuration
from njord_sim.scenario import world_xml
import xml.etree.ElementTree as ET

class AllocationTests(unittest.TestCase):
    def test_asymmetric_arms_reproduce_requested_force_and_moment(self):
        positions=[-1., .8, 0., -.7, -.4, 0.]
        left,right=allocate_thrusters(90., 12., positions,[1.,0.,0.]*2,[500.,400.],[200.,300.])
        self.assertAlmostEqual(left+right,90.)
        self.assertAlmostEqual(-.8*left+.4*right,12.)

    def test_directional_limits_scale_wrench_without_exceeding_capacity(self):
        left,right=allocate_thrusters(-600.,0.,[0.,1.,0.,0.,-1.,0.],[1.,0.,0.]*2,[500.,500.],[100.,200.])
        self.assertEqual((left,right),(-100.,-100.))

    def test_singular_and_nonfinite_geometry_rejected(self):
        for positions in ([0.]*6,[float('nan')]*6):
            with self.assertRaises(ValueError):
                allocate_thrusters(1.,0.,positions,[1.,0.,0.]*2,[500.]*2,[500.]*2)

class ManifestTests(unittest.TestCase):
    def test_resource_snapshot_is_used_and_changed_source_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory)
            mesh = out/'input.obj'
            mesh.write_text('immutable mesh bytes')
            resolved = {'resources': {str(mesh): sha256(mesh)},
                        'vessel': {'geometry': {'visual': {'uri': str(mesh)}}}}
            freeze_resources(out, resolved)
            frozen = Path(resolved['vessel']['geometry']['visual']['uri'])
            self.assertNotEqual(frozen, mesh)
            self.assertEqual(frozen.read_bytes(), mesh.read_bytes())
            mesh.write_text('changed after resolution')
            with self.assertRaises(ValueError):
                freeze_resources(out, resolved)

    def test_njord_world_gravity_and_water_plane_match_force_model(self):
        root = Path(__file__).resolve().parents[1]
        resolved = resolve_configuration(root/'njord_sim/config/njord_v1.yaml',
            root/'scenarios/reference.yaml', root/'njord_sim/config/algorithms.yaml')
        scenario = resolved['scenario']
        scenario['environment']['water_level_m'] = 2.0
        world = ET.fromstring(world_xml(scenario, 'njord')).find('world')
        self.assertEqual(world.findtext('gravity'), '0 0 -9.81')
        self.assertEqual(float(world.findtext('include/pose').split()[2]), 2.0)
        self.assertEqual(float(world.findtext('model/pose').split()[2]), 2.5)
        self.assertIsNone(world.find("plugin[@name='vrx::USVWind']"))

    def test_public_handoff_is_bound_to_artifacts_and_excludes_truth(self):
        with tempfile.TemporaryDirectory() as directory:
            out=Path(directory)
            source=out/'source.yaml';source.write_text('schema_version: 1')
            (out/'vessel_config.yaml').write_text('max_thrust_n: 500')
            scenario={'seed':7,'gates':[{'red':[99,12],'green':[99,0]}]}
            scenario_path = out/'resolved_scenario.json'
            scenario_path.write_text(json.dumps(scenario))
            digest=write_manifest(out,'run1',{'scenario':scenario}, {'vessel':source}, {'guidance':{'max_speed':1.}})
            publish_ready(out,'run1',scenario,digest)
            metadata=wait_ready(out,'run1',timeout=0)
            self.assertEqual(metadata['manifest_sha256'],digest)
            self.assertNotIn('gates',(out/'public_parameters.json').read_text())
            original = scenario_path.read_bytes()
            scenario_path.write_text('{}')
            with self.assertRaises(ValueError):
                verify_public_handoff(out,metadata)
            scenario_path.write_bytes(original)
            (out/'vessel_config.yaml').write_text('max_thrust_n: 999')
            with self.assertRaises(ValueError):
                verify_public_handoff(out,metadata)

if __name__=='__main__':unittest.main()
