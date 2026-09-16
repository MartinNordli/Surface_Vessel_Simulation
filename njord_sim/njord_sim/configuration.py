"""Versioned SI configuration shared by launch, model generation and evaluation.

Mass inertia is about center_of_mass_m, with axes parallel to forward/left/up
body axes. Damping and added mass are nonnegative diagonal six-DOF magnitudes.
Geometry accepts analytical boxes and pre-scaled, closed convex triangular OBJ
resources. Arbitrary concave CAD and automatic geometry scaling are rejected.
"""
import copy
import hashlib
import json
import math
from pathlib import Path
import random
import yaml

from .scenario_core import validate_scenario
from .mesh_geometry import (load_obj, geometry_vertices, geometry_volume,
                            validate_disjoint_volumes)

SENSOR_KEYS = set('camera_width camera_height camera_rate camera_horizontal_fov_rad camera_noise_stddev lidar_range lidar_samples lidar_vertical_samples lidar_rate lidar_noise_stddev gps_rate imu_rate gps_horizontal_noise_m gps_vertical_noise_m imu_orientation_noise_rad'.split())
GUIDANCE_KEYS = set('lookahead_m kp_yaw kd_yaw kp_surge max_speed max_thrust goal_tolerance_m control_hz stale_after_s braking_deceleration_mps2 reaction_time_s stopping_margin_m'.split())
ENV_KEYS = set('wind_speed_mps wind_direction_to_deg_enu wind_variance_gain wave_gain wave_period_s wave_direction_rad wave_steepness current_speed_mps current_direction_to_deg_enu water_density_kg_m3 water_level_m physics_step_s'.split())


def _keys(data, required, optional=(), where='configuration'):
    if not isinstance(data, dict):
        raise ValueError(f'{where} must be a mapping')
    missing, unknown = set(required) - data.keys(), data.keys() - set(required) - set(optional)
    if missing or unknown:
        raise ValueError(f'{where}: missing {sorted(missing)}, unknown {sorted(unknown, key=str)}')


def _number(value, name, minimum=None, positive=False):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise ValueError(f'{name} must be a finite SI number')
    if minimum is not None and value < minimum or positive and value <= 0:
        raise ValueError(f'{name} outside allowed physical range')
    return value


def _vector(values, n, name, minimum=None):
    if not isinstance(values, list) or len(values) != n:
        raise ValueError(f'{name} requires {n} values')
    for value in values:
        _number(value, name, minimum)


def _version(data):
    if type(data.get('schema_version')) is not int or data['schema_version'] != 1:
        raise ValueError('schema_version must be integer 1; use explicit legacy conversion')


def _read(path):
    # Reject duplicate YAML keys rather than accepting an ambiguous experiment.
    class Loader(yaml.SafeLoader):
        pass
    def mapping(loader, node):
        result = {}
        for key, value in node.value:
            key = loader.construct_object(key)
            if key in result:
                raise ValueError(f'duplicate YAML key: {key}')
            result[key] = loader.construct_object(value)
        return result
    Loader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, mapping)
    data = yaml.load(Path(path).read_text(), Loader=Loader)
    if not isinstance(data, dict):
        raise ValueError('configuration must be a YAML mapping')
    return data


def convert_legacy_vessel(data):
    """Explicit legacy WAM-V conversion; never invent Njord mass properties."""
    _keys(data, SENSOR_KEYS | {'max_thrust_n', 'thruster_separation_m'})
    return {'schema_version': 1, 'profile': 'wamv_reference', 'legacy': copy.deepcopy(data)}


def convert_legacy_scenario(data):
    """Normalize legacy directions and document old environment defaults."""
    result = copy.deepcopy(data)
    if 'schema_version' in result:
        raise ValueError('legacy conversion does not accept an already versioned scenario')
    result['schema_version'] = 1
    for env in result.get('environments', {}).values():
        if 'wind_direction_deg' in env and 'wind_direction_to_deg_enu' in env:
            raise ValueError('conflicting legacy and versioned wind direction')
        if 'wind_direction_deg' in env:
            env['wind_direction_to_deg_enu'] = env.pop('wind_direction_deg')
        else:
            env.setdefault('wind_direction_to_deg_enu', 0.0)
        for k, v in dict(current_speed_mps=0., current_direction_to_deg_enu=0.,
                         water_density_kg_m3=1000., water_level_m=0., physics_step_s=0.004).items():
            env.setdefault(k, v)
    return result


def validate_vessel(vessel, resource_base=None, resource_files=None):
    _version(vessel)
    if vessel.get('profile') == 'wamv_reference':
        _keys(vessel, {'schema_version', 'profile', 'legacy'})
        _keys(vessel['legacy'], SENSOR_KEYS | {'max_thrust_n', 'thruster_separation_m'})
        _validate_sensors({k: vessel['legacy'][k] for k in SENSOR_KEYS})
        for k in ('max_thrust_n', 'thruster_separation_m'):
            _number(vessel['legacy'][k], k, positive=True)
        if not math.isclose(vessel['legacy']['thruster_separation_m'], 2.05427, abs_tol=1e-10, rel_tol=0):
            raise ValueError('WAM-V thruster_separation_m must match pinned VRX geometry 2.05427')
        return vessel
    _keys(vessel, set('schema_version profile name revision calibration mass_kg center_of_mass_m inertia_kg_m2 geometry hydrodynamics wind thrusters sensors'.split()))
    if vessel['profile'] != 'njord':
        raise ValueError('unknown vessel profile')
    for key in ('name', 'revision'):
        if not isinstance(vessel[key], str) or not vessel[key]:
            raise ValueError(f'{key} must be nonempty')
    _keys(vessel['calibration'], {'status', 'valid_speed_mps'})
    if vessel['calibration']['status'] != 'uncalibrated':
        raise ValueError('only uncalibrated infrastructure profiles are supported pending measurement review')
    _vector(vessel['calibration']['valid_speed_mps'], 2, 'valid_speed_mps', 0)
    if vessel['calibration']['valid_speed_mps'][0] >= vessel['calibration']['valid_speed_mps'][1]:
        raise ValueError('invalid calibration speed interval')
    _number(vessel['mass_kg'], 'mass_kg', positive=True)
    _vector(vessel['center_of_mass_m'], 3, 'center_of_mass_m')
    inertia = vessel['inertia_kg_m2']
    _keys(inertia, {'ixx','iyy','izz','ixy','ixz','iyz'})
    for value in inertia.values():
        _number(value, 'inertia')
    # Positive definiteness of I and covariance C=tr(I)/2 identity-I
    # gives positive principal moments and all principal triangle inequalities.
    a,b,c,d,e,f = (inertia[k] for k in ('ixx','iyy','izz','ixy','ixz','iyz'))
    def psd(x,y,z,xy,xz,yz, strict=False):
        vals = [x,y,z,x*y-xy*xy,x*z-xz*xz,y*z-yz*yz,
                x*y*z+2*xy*xz*yz-x*yz*yz-y*xz*xz-z*xy*xy]
        return all(v > 0 if strict else v >= -1e-10 for v in vals)
    if not psd(a,b,c,d,e,f,True) or not psd((b+c-a)/2,(a+c-b)/2,(a+b-c)/2,-d,-e,-f):
        raise ValueError('inertia must be positive definite and satisfy principal moment triangle inequalities')
    _keys(vessel['geometry'], {'visual','collision','buoyancy'})
    for name, definition in vessel['geometry'].items():
        geometries = definition if isinstance(definition, list) and name == 'buoyancy' else [definition]
        if not geometries:
            raise ValueError('buoyancy requires at least one volume')
        for geom in geometries:
            if not isinstance(geom, dict):
                raise ValueError(f'{name} geometry must be a mapping')
            if geom.get('type') == 'box':
                _keys(geom, {'type','size_m','pose'}, where=name)
                _vector(geom['size_m'],3,name)
                if min(geom['size_m']) <= 0:
                    raise ValueError('box size must be positive')
            elif geom.get('type') == 'mesh':
                _keys(geom, {'type','uri','pose'}, where=name)
                if not isinstance(geom['uri'],str) or not geom['uri']:
                    raise ValueError('mesh uri must be a local OBJ path')
                path=Path(geom['uri']).expanduser()
                if not path.is_absolute(): path=Path(resource_base or '.')/path
                path=path.resolve()
                if path.suffix.lower() != '.obj' or not path.is_file():
                    raise ValueError(f'missing or unsupported mesh resource: {path}')
                vertices,faces,volume=load_obj(path)
                geom.update(uri=str(path),vertices=vertices,faces=faces,volume_m3=volume)
                if resource_files is not None: resource_files.append(path)
            else:
                raise ValueError('geometry type must be box or convex triangular OBJ mesh')
            _vector(geom['pose'],6,name)
            if any(geom['pose'][3:]):
                raise ValueError('geometry rotation must be prepared into mesh vertices; pose rotation must be zero')
    buoyancy=vessel['geometry']['buoyancy']
    validate_disjoint_volumes(buoyancy if isinstance(buoyancy,list) else [buoyancy])
    _keys(vessel['hydrodynamics'], {'linear_damping','quadratic_damping','added_mass'})
    for key,value in vessel['hydrodynamics'].items():
        _vector(value,6,key,0)
    _keys(vessel['wind'], {'reference_area_m2','reference_length_m','coefficients'})
    _vector(vessel['wind']['reference_area_m2'],2,'wind.reference_area_m2',0)
    _number(vessel['wind']['reference_length_m'],'wind.reference_length_m',positive=True)
    table = vessel['wind']['coefficients']
    if not isinstance(table,list) or len(table)<2:
        raise ValueError('wind coefficients need at least two periodic angle samples')
    angles=[]
    for row in table:
        _keys(row,{'angle_deg','cx','cy','cn'})
        for v in row.values(): _number(v,'wind coefficient')
        if not 0 <= row['angle_deg'] < 360: raise ValueError('wind angle must be in [0,360)')
        angles.append(row['angle_deg'])
    if angles != sorted(set(angles)): raise ValueError('wind angle samples must be unique and ordered')
    thrusters = vessel['thrusters']
    if not isinstance(thrusters,list) or len(thrusters)!=2: raise ValueError('exactly two fixed thrusters required')
    for t in thrusters:
        _keys(t,{'name','position_m','axis','forward_limit_n','reverse_limit_n','response_time_s'})
        _vector(t['position_m'],3,'thruster position')
        _vector(t['axis'],3,'thruster axis')
        if t['axis'][0] <= 0 or abs(t['axis'][2]) > 1e-12 or not math.isclose(sum(v*v for v in t['axis']),1.,abs_tol=1e-9):
            raise ValueError('thruster axes must be planar forward unit vectors')
        for key in ('forward_limit_n','reverse_limit_n'): _number(t[key],key,positive=True)
        _number(t['response_time_s'],'response_time_s',0)
    if [t['name'] for t in thrusters] != ['left','right']: raise ValueError('thrusters must be ordered left, right')
    if thrusters[0]['position_m'][1] <= thrusters[1]['position_m'][1]: raise ValueError('left thruster must be port of right thruster')
    columns=[]
    for t in thrusters:
        x,y,_=[p-c for p,c in zip(t['position_m'],vessel['center_of_mass_m'])]
        ax,ay,_=t['axis']
        columns.append((ax,x*ay-y*ax))
    if abs(columns[0][0]*columns[1][1]-columns[1][0]*columns[0][1]) < 1e-9:
        raise ValueError('thruster geometry cannot independently control surge and yaw')
    _keys(vessel['sensors'],{'settings','poses'})
    _validate_sensors(vessel['sensors']['settings'])
    if vessel['sensors']['settings']['lidar_range'] <= 0.2:
        raise ValueError('Njord lidar_range must exceed the physical minimum range 0.2 m')
    _keys(vessel['sensors']['poses'], {'camera','camera_right','lidar','gps','imu'})
    for value in vessel['sensors']['poses'].values(): _vector(value,6,'sensor pose')
    return vessel


def _validate_sensors(settings):
    _keys(settings,SENSOR_KEYS)
    counts={'camera_width','camera_height','lidar_samples','lidar_vertical_samples'}
    for key,value in settings.items():
        _number(value,key,0,positive='noise' not in key)
        if key in counts and type(value) is not int: raise ValueError(f'{key} must be an integer')
    if settings['camera_horizontal_fov_rad'] >= math.pi: raise ValueError('camera FOV must be below pi')


def _resolve_scenario(data, seed, environment):
    if 'schema_version' not in data: data=convert_legacy_scenario(data)
    _version(data)
    _keys(data,{'schema_version','name','start','timeout_s','gates','environments'}, {'seed','hull','gate_y_jitter_m','obstacles'})
    _vector(data['start'],6,'start')
    _number(data['timeout_s'],'timeout_s',positive=True)
    _number(data.get('gate_y_jitter_m',0),'gate_y_jitter_m',0)
    if not isinstance(data['environments'],dict) or not data['environments']: raise ValueError('environments must be nonempty')
    for env in data['environments'].values():
        _keys(env,ENV_KEYS,where='environment')
        for k,v in env.items(): _number(v,k)
        for k in ('wind_speed_mps','wind_variance_gain','wave_gain','wave_steepness','current_speed_mps'): _number(env[k],k,0)
        for k in ('wave_period_s','water_density_kg_m3','physics_step_s'): _number(env[k],k,positive=True)
        for kind in ('wind','current'):
            angle=math.radians(env[f'{kind}_direction_to_deg_enu'])
            speed=env[f'{kind}_speed_mps']
            env[f'{kind}_velocity_enu']=[speed*math.cos(angle),speed*math.sin(angle),0.]
        env['wind_direction_deg']=env['wind_direction_to_deg_enu'] # legacy world adapter
    if not isinstance(data['gates'],list): raise ValueError('gates must be a list')
    for gate in data['gates']:
        _keys(gate,{'name','red','green','radius_m'})
        for k in ('red','green'): _vector(gate[k],2,k)
        _number(gate['radius_m'],'radius_m',positive=True)
    for obstacle in data.get('obstacles',[]):
        _keys(obstacle,{'name','position','radius_m'},{'color'})
        _vector(obstacle['position'],2,'position')
        _number(obstacle['radius_m'],'radius_m',positive=True)
    if 'hull' in data:
        _keys(data['hull'],{'length_m','beam_m'})
        for v in data['hull'].values(): _number(v,'hull',positive=True)
    data['seed']=data.get('seed',1) if seed is None else seed
    if type(data['seed']) is not int or data['seed'] <= 0: raise ValueError('seed must be a positive integer')
    name=environment or 'calm'
    if name not in data['environments']: raise ValueError(f'unknown environment {name}')
    data['environment_name']=name
    data['environment']=copy.deepcopy(data['environments'][name])
    rng=random.Random(data['seed'])
    for gate in data['gates']:
        offset=rng.uniform(-data.get('gate_y_jitter_m',0),data.get('gate_y_jitter_m',0))
        gate['red'][1]+=offset; gate['green'][1]+=offset
    data['resolved']=True
    return data


def resolve_configuration(vessel_file, scenario_file, algorithms_file, seed=None, environment=None, legacy_vessel=False):
    vessel_data=_read(vessel_file)
    source_files=[vessel_file,scenario_file,algorithms_file]
    if legacy_vessel:
        if "schema_version" in vessel_data: raise ValueError("legacy_vessel conflicts with versioned vessel")
        from .vessel import load_config
        defaults=Path(__file__).resolve().parents[1]/"config/vessel.yaml"
        if not defaults.exists():
            from ament_index_python.packages import get_package_share_directory
            defaults=Path(get_package_share_directory("njord_sim"))/"config/vessel.yaml"
        vessel_data=convert_legacy_vessel(load_config(vessel_file,defaults))
        source_files.append(defaults)
    vessel=validate_vessel(vessel_data, Path(vessel_file).resolve().parent, source_files)
    scenario_data=_read(scenario_file)
    versioned_scenario='schema_version' in scenario_data
    scenario=_resolve_scenario(scenario_data,seed,environment)
    algorithms=_read(algorithms_file)
    _version(algorithms)
    _keys(algorithms,{'schema_version','guidance','planner','mapping'})
    _keys(algorithms['guidance'],GUIDANCE_KEYS)
    _keys(algorithms['planner'],{'stale_after_s','publish_hz'})
    _keys(algorithms['mapping'],{'safety_margin_m'})
    for group in ('guidance','planner','mapping'):
        for k,v in algorithms[group].items(): _number(v,k,0,positive=k not in {'kp_yaw','kd_yaw','kp_surge','reaction_time_s','stopping_margin_m','safety_margin_m'})
    if vessel['profile']=='njord':
        env=scenario['environment']
        if env['wave_gain'] or env['wave_steepness']: raise ValueError('Njord flat-water profile rejects nonzero waves')
        if env['wind_variance_gain']: raise ValueError('Njord initial wind adapter supports constant wind only')
        buoyancy=vessel['geometry']['buoyancy']
        volumes=buoyancy if isinstance(buoyancy,list) else [buoyancy]
        if sum(geometry_volume(g) for g in volumes)*env['water_density_kg_m3'] <= vessel['mass_kg']:
            raise ValueError('insufficient maximum displacement volume')
        collision=vessel['geometry']['collision']
        # Symmetric envelope includes any offset from body origin.
        vertices=geometry_vertices(collision)
        vessel['hull']={'length_m':2*max(abs(v[0]) for v in vertices),
                        'beam_m':2*max(abs(v[1]) for v in vertices)}
        if versioned_scenario and 'hull' in scenario and scenario['hull'] != vessel['hull']:
            raise ValueError('versioned scenario hull conflicts with authoritative vessel collision geometry')
        scenario['hull']=copy.deepcopy(vessel['hull'])
        limits=[t[k] for t in vessel['thrusters'] for k in ('forward_limit_n','reverse_limit_n')]
    else:
        env=scenario['environment']
        for key,default in {'current_speed_mps':0.,'water_density_kg_m3':1000.,'water_level_m':0.}.items():
            if env[key] != default:
                raise ValueError(f'WAM-V reference does not support overriding {key}')
        reference_hull = {'length_m': 6., 'beam_m': 3.3}
        if 'hull' in scenario and scenario['hull'] != reference_hull:
            raise ValueError('WAM-V scoring hull must match the pinned reference envelope')
        scenario['hull'] = reference_hull
        limits=[vessel['legacy']['max_thrust_n']]
    if algorithms['guidance']['max_thrust'] > min(limits): raise ValueError('algorithm max_thrust exceeds physical actuator capacity')
    validate_scenario(scenario)
    resources={str(Path(p).resolve()):hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in source_files}
    resolved={'schema_version':1,'vessel':vessel,'scenario':scenario,'algorithms':algorithms,'resources':resources}
    json.dumps(resolved,allow_nan=False)
    return resolved


def autonomy_parameters(resolved):
    """Public physical and tuning projection: contains no scenario truth."""
    vessel=resolved['vessel']; result=copy.deepcopy(resolved['algorithms']); result.pop('schema_version')
    if vessel['profile']=='njord':
        ts=vessel['thrusters']
        result['guidance'].update(physical_allocation=True,
                                  thruster_positions=[v-c for t in ts for v,c in zip(t['position_m'],vessel['center_of_mass_m'])],
                                  thruster_axes=[v for t in ts for v in t['axis']],
                                  thruster_forward_limits=[t['forward_limit_n'] for t in ts],
                                  thruster_reverse_limits=[t['reverse_limit_n'] for t in ts])
        result['command_guard']={'forward_limits':[t['forward_limit_n'] for t in ts], 'reverse_limits':[t['reverse_limit_n'] for t in ts], 'max_thrust':result['guidance']['max_thrust']}
        hull=vessel['hull']
    else:
        result['guidance']['thruster_separation_m']=vessel['legacy']['thruster_separation_m']
        maximum=vessel['legacy']['max_thrust_n']
        result['command_guard']={'forward_limits':[maximum,maximum],
                                 'reverse_limits':[maximum,maximum],
                                 'max_thrust':result['guidance']['max_thrust']}
        hull={'length_m':6.,'beam_m':3.3}
    settings=legacy_vessel_settings(resolved)
    result['sensor_adapter']={'seed':resolved['scenario']['seed'],
                              'orientation_noise_rad':settings['imu_orientation_noise_rad'],
                              'gps_xy_std_m':settings['gps_horizontal_noise_m'],
                              'gps_z_std_m':settings['gps_vertical_noise_m']}
    result['mapper']={'inflation_m':math.hypot(hull['length_m'],hull['beam_m'])/2+result.pop('mapping')['safety_margin_m']}
    return result


def legacy_vessel_settings(resolved):
    """Compatibility projection for sensor adapters; physics stays authoritative."""
    vessel=resolved['vessel']
    if vessel['profile']=='wamv_reference': return copy.deepcopy(vessel['legacy'])
    settings=copy.deepcopy(vessel['sensors']['settings'])
    settings['max_thrust_n']=min(t[k] for t in vessel['thrusters'] for k in ('forward_limit_n','reverse_limit_n'))
    settings['thruster_separation_m']=vessel['thrusters'][0]['position_m'][1]-vessel['thrusters'][1]['position_m'][1]
    return settings
