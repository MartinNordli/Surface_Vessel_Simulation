"""Independent synthetic oracles: never claims Gazebo or measured boat fidelity."""
import copy
import importlib.util
import math
from pathlib import Path
import sys
import unittest
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'validation'))
from hull_acceptance import compare, hydrostatic_equilibrium, oscillator, oscillator_parameters
from physical_acceptance import wrench, response
spec=importlib.util.spec_from_file_location('campaign_acceptance',ROOT/'scripts/dynamics_campaign.py')
campaign=importlib.util.module_from_spec(spec);spec.loader.exec_module(campaign)


class HullAcceptanceTests(unittest.TestCase):
    def setUp(self):
        self.vessel=yaml.safe_load((ROOT/'njord_sim/config/vessels/njord_v1.yaml').read_text())
        self.environment=yaml.safe_load((ROOT/'scenarios/dynamics.yaml').read_text())['environments']['calm']
        self.vessel['wind']['reference_area_m2']=[0.,0.]

    def test_integrated_mass_added_mass_damping_rejects_wrong_response(self):
        effective=self.vessel['mass_kg']+self.vessel['hydrodynamics']['added_mass'][0]
        self.vessel['hydrodynamics']['linear_damping'][0]=40.
        self.vessel['hydrodynamics']['quadratic_damping'][0]=0.
        rows=[];loads=[]
        for i in range(602):
            t=i*.02
            speed=2.*(1-math.exp(-40*t/effective))
            rows.append((t,0.,0.,speed,0.,0.,0.,0.,speed,0.,0.,0.,0.,0.))
            loads.append({'time_s':t,'force_n':[80.,0.,0.],'moment_nm':[0.,0.,0.]})
        report={'complete':True,'samples':rows,'wrench_samples':loads}
        self.assertTrue(compare(report,self.vessel,self.environment)['passed'])
        wrong=copy.deepcopy(self.vessel);wrong['mass_kg']*=2
        self.assertFalse(compare(report,wrong,self.environment)['passed'])
        self.assertFalse(compare({'complete':True},self.vessel,self.environment)['passed'])

    def test_box_draft_density_and_mass(self):
        shape=self.vessel['geometry']['buoyancy']
        l,b,h=shape['size_m']
        z=h/2-self.vessel['mass_kg']/(l*b*self.environment['water_density_kg_m3'])
        report={'complete':True,'metrics':{'mean_z_m':z,'mean_roll_rad':0.,'mean_pitch_rad':0.}}
        self.assertTrue(hydrostatic_equilibrium(report,self.vessel,self.environment)['passed'])
        report['metrics']['mean_z_m']+=.03
        self.assertFalse(hydrostatic_equilibrium(report,self.vessel,self.environment)['passed'])

    def test_three_oscillators_against_closed_form_and_wrong_inertia(self):
        for axis,index,column,velocity in [('heave',2,5,9),('roll',3,6,10),('pitch',4,7,11)]:
            with self.subTest(axis=axis):
                vessel=copy.deepcopy(self.vessel)
                vessel['hydrodynamics']['quadratic_damping'][index]=0.
                parameters=oscillator_parameters(vessel,self.environment,axis)
                mass=parameters['effective_mass']
                alpha=parameters['linear_damping']/(2*mass)
                omega=math.sqrt(parameters['stiffness']/mass-alpha*alpha)
                rows=[];applied=[]
                for i in range(1201):
                    t=i*.01
                    q=parameters['excitation']*math.exp(-alpha*t)*(math.cos(omega*t)+alpha/omega*math.sin(omega*t))
                    dq=-parameters['excitation']*math.exp(-alpha*t)*(omega*omega+alpha*alpha)/omega*math.sin(omega*t)
                    row=[t+.01,0.,0.,0.,0.,parameters['equilibrium_z_m'],0.,0.,0.,0.,0.,0.,0.,0.]
                    row[column]+=q;row[velocity]=dq
                    rows.append(row)
                    applied.append({'time_s':t+.01,'forces_n':[0.]*len(vessel['thrusters'])})
                report={'complete':True,'samples':rows,'applied_samples':applied}
                self.assertTrue(oscillator(report,vessel,self.environment,axis)['passed'])
                wrong=copy.deepcopy(vessel)
                if axis=='heave':
                    wrong['hydrodynamics']['added_mass'][index]*=2
                else:
                    wrong['inertia_kg_m2']['ixx' if axis=='roll' else 'iyy']*=2
                self.assertFalse(oscillator(report,wrong,self.environment,axis)['passed'])
                self.assertFalse(oscillator(dict(report,applied_samples=[]),vessel,self.environment,axis)['passed'])
                coupled=copy.deepcopy(report);coupled['samples'][10][8]=.1
                self.assertFalse(oscillator(coupled,vessel,self.environment,axis)['passed'])

    def test_oscillator_rejects_environment_and_unsupported_geometry(self):
        environment=dict(self.environment,wind_speed_mps=1.)
        with self.assertRaisesRegex(ValueError,'still water'):
            oscillator_parameters(self.vessel,environment,'roll')
        vessel=copy.deepcopy(self.vessel);vessel['center_of_mass_m'][0]=.1
        with self.assertRaisesRegex(ValueError,'zero COM'):
            oscillator_parameters(vessel,self.environment,'pitch')

    def test_one_parameter_variant_and_signed_axis_allocation(self):
        scenario={'environments':{'calm':self.environment}}
        variants=campaign.physical_variants(self.vessel,scenario,'vessel.mass_kg',[135.,225.],'calm')
        for _,v,s,change in variants[1:]:
            expected=copy.deepcopy(self.vessel);expected['mass_kg']=change['after']
            self.assertEqual(v,expected);self.assertEqual(s,scenario)
        vessel=yaml.safe_load((ROOT/'njord_sim/config/vessels/munin_v0.yaml').read_text())
        vectors=campaign.excitation_vectors(vessel,100.)
        for name,forces in vectors.items():
            if name.startswith('thruster'):
                self.assertEqual(sum(abs(f)>0 for f in forces),1)
                continue
            net=wrench(vessel['thrusters'],forces,vessel['center_of_mass_m'])
            actual=[net['force_n'][0],net['force_n'][1],net['moment_nm'][2]]
            axis,sign=name.split('-',1);index=('surge','sway','yaw').index(axis)
            self.assertGreater(actual[index]*int(sign),0.)
            self.assertTrue(all(abs(v)<1e-8 for i,v in enumerate(actual) if i!=index))

    def test_actuator_acceptance_rejects_missing_and_truncated_measurements(self):
        self.assertFalse(campaign.actuator_acceptance({},self.vessel,[50.,50.])['passed'])
        bad={'applied_samples':[{'time_s':1.,'forces_n':[0.], 'targets_n':[50.]}],
             'wrench_samples':[]}
        self.assertFalse(campaign.actuator_acceptance(bad,self.vessel,[50.,50.])['passed'])

if __name__=='__main__': unittest.main()
