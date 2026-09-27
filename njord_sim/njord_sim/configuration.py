"""Load, validate and resolve the three YAML inputs of a simulator run.

A run is defined by exactly three files, each with a single owner:

- vessel      (VESSEL_CONFIG)      what the boat is: geometry, mass, actuators, sensors
- scenario    (SCENARIO)           the world: course, start pose, seed, environments
- algorithms  (ALGORITHMS_CONFIG)  reference autonomy tuning and speed profiles

``resolve_configuration`` validates all three before Gazebo starts and returns
one JSON-serializable dictionary. Model generation, launch, the evaluator and
the autonomy parameters are all derived from that single result, so a value is
never read from two places.

Conventions: SI units; world ENU; body axes forward/left/up. Mass inertia is
about ``center_of_mass_m`` with body-parallel axes. Damping and added mass are
nonnegative diagonal six-DOF magnitudes. Geometry accepts analytic boxes and
pre-scaled, closed, convex triangular OBJ meshes; concave CAD and automatic
scaling are rejected.
"""
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import random
import re

import yaml

from .constants import COMMAND_TIMEOUT_S, THRUSTER_COMMAND_TOPIC, WAMV_HULL, WAMV_THRUSTERS
from .control_core import allocation_matrix, independent_rows
from .mesh_geometry import geometry_vertices, geometry_volume, load_obj, validate_disjoint_volumes
from .scenario_core import validate_scenario

# Sensor settings shared by every vessel profile.
SENSOR_KEYS = {
    'camera_width', 'camera_height', 'camera_rate', 'camera_horizontal_fov_rad',
    'camera_noise_stddev', 'lidar_range', 'lidar_samples', 'lidar_vertical_samples',
    'lidar_rate', 'lidar_noise_stddev', 'gps_rate', 'imu_rate', 'gps_horizontal_noise_m',
    'gps_vertical_noise_m', 'imu_orientation_noise_rad',
}
# The WAM-V profile exposes the sensors plus its thrust limit (vessels/wamv.yaml).
WAMV_SETTING_KEYS = SENSOR_KEYS | {'max_thrust_n'}
GUIDANCE_KEYS = {
    'lookahead_m', 'kp_yaw', 'kd_yaw', 'kp_surge', 'max_thrust', 'goal_tolerance_m',
    'control_hz', 'stale_after_s', 'braking_deceleration_mps2', 'reaction_time_s',
    'stopping_margin_m',
}
# Algorithm values that may legitimately be zero; everything else must be > 0.
NONNEGATIVE_ALGORITHM_KEYS = {
    'kp_yaw', 'kd_yaw', 'kp_surge', 'reaction_time_s', 'stopping_margin_m', 'safety_margin_m',
}
ENVIRONMENT_KEYS = {
    'wind_speed_mps', 'wind_direction_to_deg_enu', 'wind_variance_gain', 'wave_gain',
    'wave_period_s', 'wave_direction_rad', 'wave_steepness', 'current_speed_mps',
    'current_direction_to_deg_enu', 'water_density_kg_m3', 'water_level_m', 'physics_step_s',
}
DEFAULT_PROFILE = 'fast'


# --------------------------------------------------------------------------
# Small validation helpers. Every failure raises ValueError with a message
# naming the offending key, before any process is started.
# --------------------------------------------------------------------------

def _keys(data, required, optional=(), where='configuration'):
    """Require a mapping with exactly the required keys plus optional ones."""
    if not isinstance(data, dict):
        raise ValueError(f'{where} must be a mapping')
    missing = set(required) - data.keys()
    unknown = data.keys() - set(required) - set(optional)
    if missing or unknown:
        raise ValueError(f'{where}: missing {sorted(missing)}, unknown {sorted(unknown, key=str)}')


def _number(value, name, minimum=None, positive=False):
    """Accept a finite int/float (not bool), optionally bounded below."""
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise ValueError(f'{name} must be a finite SI number')
    if minimum is not None and value < minimum or positive and value <= 0:
        raise ValueError(f'{name} outside allowed physical range')
    return value


def _vector(values, n, name, minimum=None):
    """Accept a list of exactly n finite numbers."""
    if not isinstance(values, list) or len(values) != n:
        raise ValueError(f'{name} requires {n} values')
    for value in values:
        _number(value, name, minimum)


def _version(data, version=1):
    if type(data.get('schema_version')) is not int or data['schema_version'] != version:
        raise ValueError(f'schema_version must be integer {version}')


def _read(path):
    """Read a YAML mapping, rejecting duplicate keys (an ambiguous experiment)."""
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


def config_path(relative):
    """Locate a file under njord_sim/config, preferring the editable copy.

    Search order: $NJORD_CONFIG_DIR (compose.yaml sets /config, the host's
    njord_sim/config mounted read-only), the source tree, then the installed
    package. Host edits therefore apply to the next run without a rebuild,
    and a team CONFIG_HOST without the file falls back to the image's copy.
    """
    candidates = []
    if os.environ.get('NJORD_CONFIG_DIR'):
        candidates.append(Path(os.environ['NJORD_CONFIG_DIR']) / relative)
    candidates.append(Path(__file__).resolve().parents[1] / 'config' / relative)
    for path in candidates:
        if path.exists():
            return path
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('njord_sim')) / 'config' / relative


def wamv_defaults_file():
    """The WAM-V vessel file that partial sensor overrides are merged onto."""
    return config_path('vessels/wamv.yaml')


# --------------------------------------------------------------------------
# Vessel
# --------------------------------------------------------------------------

def wamv_profile_from_settings(settings):
    """Wrap complete flat WAM-V settings in the versioned vessel structure."""
    _keys(settings, WAMV_SETTING_KEYS, where='WAM-V settings')
    return {'schema_version': 1, 'profile': 'wamv_reference', 'settings': copy.deepcopy(settings)}


def _validate_sensors(settings):
    """Sensor rates/sizes must be positive, noise nonnegative, counts integers."""
    _keys(settings, SENSOR_KEYS, where='sensor settings')
    counts = {'camera_width', 'camera_height', 'lidar_samples', 'lidar_vertical_samples'}
    for key, value in settings.items():
        _number(value, key, 0, positive='noise' not in key)
        if key in counts and type(value) is not int:
            raise ValueError(f'{key} must be an integer')
    if settings['camera_horizontal_fov_rad'] >= math.pi:
        raise ValueError('camera FOV must be below pi')


def _validate_inertia(inertia):
    """Require a physically realizable inertia tensor.

    Positive definiteness of I, and positive semidefiniteness of the second
    moment tensor C = tr(I)/2 * identity - I, together give positive principal
    moments that satisfy every principal triangle inequality.
    """
    _keys(inertia, {'ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'}, where='inertia_kg_m2')
    for value in inertia.values():
        _number(value, 'inertia')
    a, b, c, d, e, f = (inertia[k] for k in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'))

    def definite(x, y, z, xy, xz, yz, strict=False):
        # Sylvester-style check on all principal minors of a symmetric 3x3.
        minors = [x, y, z, x * y - xy * xy, x * z - xz * xz, y * z - yz * yz,
                  x * y * z + 2 * xy * xz * yz - x * yz * yz - y * xz * xz - z * xy * xy]
        return all(m > 0 if strict else m >= -1e-10 for m in minors)

    if (not definite(a, b, c, d, e, f, strict=True)
            or not definite((b + c - a) / 2, (a + c - b) / 2, (a + b - c) / 2, -d, -e, -f)):
        raise ValueError('inertia must be positive definite and satisfy principal moment triangle inequalities')


def _validate_geometry(geometry, resource_base, resource_files):
    """Validate visual/collision/buoyancy shapes; load and embed OBJ meshes."""
    _keys(geometry, {'visual', 'collision', 'buoyancy'}, where='geometry')
    for name, definition in geometry.items():
        # Only buoyancy may be a list of separate volumes.
        shapes = definition if isinstance(definition, list) and name == 'buoyancy' else [definition]
        if not shapes:
            raise ValueError('buoyancy requires at least one volume')
        for shape in shapes:
            if not isinstance(shape, dict):
                raise ValueError(f'{name} geometry must be a mapping')
            if shape.get('type') == 'box':
                _keys(shape, {'type', 'size_m', 'pose'}, where=name)
                _vector(shape['size_m'], 3, name)
                if min(shape['size_m']) <= 0:
                    raise ValueError('box size must be positive')
            elif shape.get('type') == 'mesh':
                _keys(shape, {'type', 'uri', 'pose'}, where=name)
                if not isinstance(shape['uri'], str) or not shape['uri']:
                    raise ValueError('mesh uri must be a local OBJ path')
                path = Path(shape['uri']).expanduser()
                if not path.is_absolute():
                    path = Path(resource_base or '.') / path  # relative to the vessel file
                path = path.resolve()
                if path.suffix.lower() != '.obj' or not path.is_file():
                    raise ValueError(f'missing or unsupported mesh resource: {path}')
                vertices, faces, volume = load_obj(path)
                shape.update(uri=str(path), vertices=vertices, faces=faces, volume_m3=volume)
                if resource_files is not None:
                    resource_files.append(path)  # checksummed in the run manifest
            else:
                raise ValueError('geometry type must be box or convex triangular OBJ mesh')
            _vector(shape['pose'], 6, name)
            if any(shape['pose'][3:]):
                raise ValueError('geometry rotation must be prepared into mesh vertices; pose rotation must be zero')
    buoyancy = geometry['buoyancy']
    validate_disjoint_volumes(buoyancy if isinstance(buoyancy, list) else [buoyancy])


def _validate_wind(wind):
    """Reference areas/length plus an ordered periodic coefficient table."""
    _keys(wind, {'reference_area_m2', 'reference_length_m', 'coefficients'}, where='wind')
    _vector(wind['reference_area_m2'], 2, 'wind.reference_area_m2', 0)
    _number(wind['reference_length_m'], 'wind.reference_length_m', positive=True)
    table = wind['coefficients']
    if not isinstance(table, list) or len(table) < 2:
        raise ValueError('wind coefficients need at least two periodic angle samples')
    angles = []
    for row in table:
        _keys(row, {'angle_deg', 'cx', 'cy', 'cn'}, where='wind coefficient')
        for value in row.values():
            _number(value, 'wind coefficient')
        if not 0 <= row['angle_deg'] < 360:
            raise ValueError('wind angle must be in [0,360)')
        angles.append(row['angle_deg'])
    if angles != sorted(set(angles)):
        raise ValueError('wind angle samples must be unique and ordered')


def thruster_axis(yaw_deg):
    """Body-frame unit thrust direction for a thruster turned ``yaw_deg`` from forward.

    The angle is counter-clockwise about body z (up): 0 pushes forward, 90
    pushes to port (left), 180 pushes aft. Rounded so 90 degrees gives an
    exact zero x component.
    """
    angle = math.radians(yaw_deg)
    return [round(math.cos(angle), 12) + 0.0, round(math.sin(angle), 12) + 0.0, 0.0]


def _validate_thrusters(thrusters, center_of_mass):
    """Two or more fixed thrusters, in command order, able to surge and steer.

    Each thruster is named (the name selects its command topic), placed at
    ``position_m`` in the body frame and pushes along the horizontal
    direction ``yaw_deg``. Tunnel and reverse-mounted thrusters are allowed;
    together the thrusters must be able to set surge force and yaw moment
    independently. Whether they can also set sway (dynamic positioning) is
    reported by ``fully_actuated``.
    """
    if not isinstance(thrusters, list) or len(thrusters) < 2:
        raise ValueError('at least two fixed thrusters required')
    names = []
    for thruster in thrusters:
        _keys(thruster, {'name', 'position_m', 'yaw_deg', 'forward_limit_n', 'reverse_limit_n',
                         'response_time_s'}, where='thruster')
        if not isinstance(thruster['name'], str) or not re.fullmatch(r'[a-z][a-z0-9_]*', thruster['name']):
            raise ValueError('thruster name must match [a-z][a-z0-9_]* (it becomes a ROS topic name)')
        names.append(thruster['name'])
        _vector(thruster['position_m'], 3, 'thruster position')
        _number(thruster['yaw_deg'], 'thruster yaw_deg')
        if not -360 <= thruster['yaw_deg'] <= 360:
            raise ValueError('thruster yaw_deg must be within [-360, 360] degrees')
        for key in ('forward_limit_n', 'reverse_limit_n'):
            _number(thruster[key], key, positive=True)
        _number(thruster['response_time_s'], 'response_time_s', 0)
    if len(set(names)) != len(names):
        raise ValueError('thruster names must be unique')
    positions, axes = _arms(thrusters, center_of_mass)
    surge, _, yaw = allocation_matrix(positions, axes)
    if independent_rows([surge, yaw]) < 2:
        raise ValueError('thruster geometry cannot independently control surge and yaw')


def _arms(thrusters, center_of_mass):
    """Flattened thruster positions relative to the COM and unit axes."""
    positions = [p - c for t in thrusters for p, c in zip(t['position_m'], center_of_mass)]
    axes = [v for t in thrusters for v in thruster_axis(t['yaw_deg'])]
    return positions, axes


def convert_legacy_vessel(data):
    """Convert a schema 1 Njord vessel to schema 2.

    Schema 1 gave each thruster a planar unit ``axis`` vector; schema 2 gives
    the same direction as ``yaw_deg``. Names and order are kept, so the
    command topics follow the old names. Non-planar or non-unit axes are
    rejected instead of guessed.
    """
    result = copy.deepcopy(data)
    if result.get('schema_version') != 1 or result.get('profile') != 'njord':
        raise ValueError('legacy vessel conversion expects a schema 1 Njord vessel')
    for thruster in result.get('thrusters') or []:
        if not isinstance(thruster, dict) or 'yaw_deg' in thruster:
            raise ValueError('schema 1 thrusters use axis, not yaw_deg')
        axis = thruster.pop('axis', None)
        _vector(axis, 3, 'thruster axis')
        if abs(axis[2]) > 1e-12 or not math.isclose(sum(v * v for v in axis), 1., abs_tol=1e-9):
            raise ValueError('thruster axes must be planar unit vectors')
        thruster['yaw_deg'] = math.degrees(math.atan2(axis[1], axis[0]))
    result['schema_version'] = 2
    return result


def thruster_table(vessel):
    """Every thruster of a validated vessel, in command order.

    This is the single derived view of the thrusters: model generation reads
    it directly, and the command guard, guidance and recorder through the
    public parameters built from it by ``autonomy_parameters``, so the vessel file
    stays the only place thrusters are defined. The WAM-V reference uses the
    pinned VRX layout from constants.py with its configured symmetric limit.

    Returns a list of dicts with ``name``, ``topic`` (the ROS command topic),
    ``position_m`` (body frame, m), ``axis`` (body-frame unit vector),
    ``forward_limit_n`` / ``reverse_limit_n`` (N, positive magnitudes) and
    ``response_time_s`` (None for the WAM-V, whose response is VRX's).
    """
    if vessel['profile'] == 'wamv_reference':
        limit = vessel['settings']['max_thrust_n']
        source = [dict(t, forward_limit_n=limit, reverse_limit_n=limit, response_time_s=None)
                  for t in WAMV_THRUSTERS]
    else:
        source = vessel['thrusters']
    return [{'name': t['name'], 'topic': THRUSTER_COMMAND_TOPIC.format(name=t['name']),
             'position_m': list(t['position_m']), 'axis': thruster_axis(t['yaw_deg']),
             'forward_limit_n': t['forward_limit_n'], 'reverse_limit_n': t['reverse_limit_n'],
             'response_time_s': t['response_time_s']} for t in source]


def fully_actuated(vessel):
    """True when the thrusters can set surge, sway and yaw independently."""
    table = thruster_table(vessel)
    center = vessel.get('center_of_mass_m', [0.0, 0.0, 0.0])
    positions = [p - c for t in table for p, c in zip(t['position_m'], center)]
    return independent_rows(allocation_matrix(positions, [v for t in table for v in t['axis']])) == 3


def validate_vessel(vessel, resource_base=None, resource_files=None):
    """Validate a versioned vessel (``wamv_reference`` or ``njord``) in place.

    ``resource_base`` resolves relative mesh paths; loaded mesh paths are
    appended to ``resource_files`` so the run manifest can checksum them.
    WAM-V files are schema 1 and Njord files schema 2; a schema 1 Njord file
    is converted by ``convert_legacy_vessel`` and the converted copy returned.
    """
    if vessel.get('profile') == 'njord' and vessel.get('schema_version') == 1:
        vessel = convert_legacy_vessel(vessel)
    _version(vessel, 2 if vessel.get('profile') == 'njord' else 1)
    if vessel.get('profile') == 'wamv_reference':
        _keys(vessel, {'schema_version', 'profile', 'settings'}, where='WAM-V vessel')
        _keys(vessel['settings'], WAMV_SETTING_KEYS, where='WAM-V settings')
        _validate_sensors({k: vessel['settings'][k] for k in SENSOR_KEYS})
        _number(vessel['settings']['max_thrust_n'], 'max_thrust_n', positive=True)
        return vessel

    _keys(vessel, {'schema_version', 'profile', 'name', 'revision', 'calibration', 'mass_kg',
                   'center_of_mass_m', 'inertia_kg_m2', 'geometry', 'hydrodynamics', 'wind',
                   'thrusters', 'sensors'}, where='vessel')
    if vessel['profile'] != 'njord':
        raise ValueError('unknown vessel profile')
    for key in ('name', 'revision'):
        if not isinstance(vessel[key], str) or not vessel[key]:
            raise ValueError(f'{key} must be nonempty')
    _keys(vessel['calibration'], {'status', 'valid_speed_mps'}, where='calibration')
    if vessel['calibration']['status'] != 'uncalibrated':
        raise ValueError('only uncalibrated infrastructure profiles are supported pending measurement review')
    _vector(vessel['calibration']['valid_speed_mps'], 2, 'valid_speed_mps', 0)
    if vessel['calibration']['valid_speed_mps'][0] >= vessel['calibration']['valid_speed_mps'][1]:
        raise ValueError('invalid calibration speed interval')
    _number(vessel['mass_kg'], 'mass_kg', positive=True)
    _vector(vessel['center_of_mass_m'], 3, 'center_of_mass_m')
    _validate_inertia(vessel['inertia_kg_m2'])
    _validate_geometry(vessel['geometry'], resource_base, resource_files)
    _keys(vessel['hydrodynamics'], {'linear_damping', 'quadratic_damping', 'added_mass'}, where='hydrodynamics')
    for key, value in vessel['hydrodynamics'].items():
        _vector(value, 6, key, 0)
    _validate_wind(vessel['wind'])
    _validate_thrusters(vessel['thrusters'], vessel['center_of_mass_m'])
    _keys(vessel['sensors'], {'settings', 'poses'}, where='sensors')
    _validate_sensors(vessel['sensors']['settings'])
    if vessel['sensors']['settings']['lidar_range'] <= 0.2:
        raise ValueError('Njord lidar_range must exceed the physical minimum range 0.2 m')
    _keys(vessel['sensors']['poses'], {'camera', 'camera_right', 'lidar', 'gps', 'imu'}, where='sensor poses')
    for value in vessel['sensors']['poses'].values():
        _vector(value, 6, 'sensor pose')
    return vessel


# --------------------------------------------------------------------------
# Scenario
# --------------------------------------------------------------------------

def convert_legacy_scenario(data):
    """Convert an unversioned scenario (before schema_version 1).

    Renames ``wind_direction_deg`` and fills the environment keys that old
    files lacked with still water at 1000 kg/m^3, level 0 and a 4 ms step.
    """
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
        for key, value in dict(current_speed_mps=0., current_direction_to_deg_enu=0.,
                               water_density_kg_m3=1000., water_level_m=0., physics_step_s=0.004).items():
            env.setdefault(key, value)
    return result


def resolve_scenario(data, seed=None, environment=None):
    """Validate a scenario mapping and resolve seed, environment and gate jitter.

    Returns a new dictionary with ``environment`` (the selected preset, plus
    derived ENU wind/current velocity vectors) and ``resolved: True``. The
    scoring hull is added later from the vessel.
    """
    data = copy.deepcopy(data)
    if 'schema_version' not in data:
        data = convert_legacy_scenario(data)
    _version(data)
    _keys(data, {'schema_version', 'name', 'start', 'timeout_s', 'gates', 'environments'},
          {'seed', 'hull', 'gate_y_jitter_m', 'obstacles'}, where='scenario')
    _vector(data['start'], 6, 'start')
    _number(data['timeout_s'], 'timeout_s', positive=True)
    _number(data.get('gate_y_jitter_m', 0), 'gate_y_jitter_m', 0)
    if not isinstance(data['environments'], dict) or not data['environments']:
        raise ValueError('environments must be nonempty')
    for env in data['environments'].values():
        _keys(env, ENVIRONMENT_KEYS, where='environment')
        for key, value in env.items():
            _number(value, key)
        for key in ('wind_speed_mps', 'wind_variance_gain', 'wave_gain', 'wave_steepness', 'current_speed_mps'):
            _number(env[key], key, 0)
        for key in ('wave_period_s', 'water_density_kg_m3', 'physics_step_s'):
            _number(env[key], key, positive=True)
        for kind in ('wind', 'current'):
            angle = math.radians(env[f'{kind}_direction_to_deg_enu'])
            speed = env[f'{kind}_speed_mps']
            env[f'{kind}_velocity_enu'] = [speed * math.cos(angle), speed * math.sin(angle), 0.]
        # The VRX wind plugin in the world generator still reads this name.
        env['wind_direction_deg'] = env['wind_direction_to_deg_enu']
    if not isinstance(data['gates'], list):
        raise ValueError('gates must be a list')
    for gate in data['gates']:
        _keys(gate, {'name', 'red', 'green', 'radius_m'}, where='gate')
        for key in ('red', 'green'):
            _vector(gate[key], 2, key)
        _number(gate['radius_m'], 'radius_m', positive=True)
    for obstacle in data.get('obstacles', []):
        _keys(obstacle, {'name', 'position', 'radius_m'}, {'color'}, where='obstacle')
        _vector(obstacle['position'], 2, 'position')
        _number(obstacle['radius_m'], 'radius_m', positive=True)
    if 'hull' in data:
        _keys(data['hull'], {'length_m', 'beam_m'}, where='hull')
        for value in data['hull'].values():
            _number(value, 'hull', positive=True)

    data['seed'] = data.get('seed', 1) if seed is None else seed
    # USVWind interprets seed 0 as nondeterministic, so require a positive seed.
    if type(data['seed']) is not int or data['seed'] <= 0:
        raise ValueError('seed must be a positive integer')
    name = environment or 'calm'
    if name not in data['environments']:
        raise ValueError(f'unknown environment {name}')
    data['environment_name'] = name
    data['environment'] = copy.deepcopy(data['environments'][name])
    # Seeded lateral jitter: the same seed always produces the same course.
    rng = random.Random(data['seed'])
    for gate in data['gates']:
        offset = rng.uniform(-data.get('gate_y_jitter_m', 0), data.get('gate_y_jitter_m', 0))
        gate['red'][1] += offset
        gate['green'][1] += offset
    data['resolved'] = True
    return data


# --------------------------------------------------------------------------
# Algorithms
# --------------------------------------------------------------------------

def _resolve_algorithms(algorithms, profile):
    """Validate algorithms.yaml and select the speed ceiling for ``profile``."""
    _version(algorithms)
    _keys(algorithms, {'schema_version', 'speed_profiles_mps', 'guidance', 'planner', 'mapping'},
          where='algorithms')
    profiles = algorithms['speed_profiles_mps']
    if not isinstance(profiles, dict) or not profiles:
        raise ValueError('speed_profiles_mps must map profile names to speeds')
    for name, speed in profiles.items():
        _number(speed, f'speed_profiles_mps.{name}', positive=True)
    if profile not in profiles:
        raise ValueError(f'unknown PROFILE {profile!r}; algorithms.yaml defines {sorted(profiles)}')
    _keys(algorithms['guidance'], GUIDANCE_KEYS, where='guidance')
    _keys(algorithms['planner'], {'stale_after_s', 'publish_hz'}, where='planner')
    _keys(algorithms['mapping'], {'safety_margin_m'}, where='mapping')
    for group in ('guidance', 'planner', 'mapping'):
        for key, value in algorithms[group].items():
            _number(value, key, 0, positive=key not in NONNEGATIVE_ALGORITHM_KEYS)
    algorithms['profile'] = profile
    algorithms['guidance']['max_speed'] = profiles[profile]
    return algorithms


# --------------------------------------------------------------------------
# Complete run configuration
# --------------------------------------------------------------------------

def resolve_configuration(vessel_file, scenario_file, algorithms_file, seed=None,
                          environment=None, profile=None):
    """Resolve and cross-check the vessel, scenario and algorithm files.

    A vessel file without ``schema_version`` is a partial WAM-V override: its
    keys replace those in vessels/wamv.yaml. ``profile`` selects a speed profile
    from algorithms.yaml (default 'fast'). Returns a JSON-serializable dict with
    ``vessel``, ``scenario``, ``algorithms`` and SHA256 digests of every source
    file under ``resources``.
    """
    vessel_data = _read(vessel_file)
    source_files = [vessel_file, scenario_file, algorithms_file]
    if 'schema_version' not in vessel_data:
        from .vessel import load_config
        defaults = wamv_defaults_file()
        vessel_data = wamv_profile_from_settings(load_config(vessel_file, defaults))
        source_files.append(defaults)
    vessel = validate_vessel(vessel_data, Path(vessel_file).resolve().parent, source_files)

    scenario_data = _read(scenario_file)
    versioned_scenario = 'schema_version' in scenario_data
    scenario = resolve_scenario(scenario_data, seed, environment)
    algorithms = _resolve_algorithms(_read(algorithms_file), profile or DEFAULT_PROFILE)
    env = scenario['environment']

    if vessel['profile'] == 'njord':
        # The Njord force model supports flat water with constant wind only.
        if env['wave_gain'] or env['wave_steepness']:
            raise ValueError('Njord flat-water profile rejects nonzero waves')
        if env['wind_variance_gain']:
            raise ValueError('Njord initial wind adapter supports constant wind only')
        buoyancy = vessel['geometry']['buoyancy']
        volumes = buoyancy if isinstance(buoyancy, list) else [buoyancy]
        if sum(geometry_volume(g) for g in volumes) * env['water_density_kg_m3'] <= vessel['mass_kg']:
            raise ValueError('insufficient maximum displacement volume')
        # Scoring envelope: a rectangle about the body origin enclosing the
        # collision geometry, including any offset.
        vertices = geometry_vertices(vessel['geometry']['collision'])
        vessel['hull'] = {'length_m': 2 * max(abs(v[0]) for v in vertices),
                          'beam_m': 2 * max(abs(v[1]) for v in vertices)}
        if versioned_scenario and 'hull' in scenario and scenario['hull'] != vessel['hull']:
            raise ValueError('versioned scenario hull conflicts with authoritative vessel collision geometry')
        scenario['hull'] = copy.deepcopy(vessel['hull'])
        limits = [t[k] for t in vessel['thrusters'] for k in ('forward_limit_n', 'reverse_limit_n')]
    else:
        # The VRX WAM-V model has fixed water properties and no current input.
        for key, default in {'current_speed_mps': 0., 'water_density_kg_m3': 1000., 'water_level_m': 0.}.items():
            if env[key] != default:
                raise ValueError(f'WAM-V reference does not support overriding {key}')
        if 'hull' in scenario and scenario['hull'] != WAMV_HULL:
            raise ValueError('WAM-V scoring hull must match the pinned reference envelope')
        scenario['hull'] = dict(WAMV_HULL)
        limits = [vessel['settings']['max_thrust_n']]
    if algorithms['guidance']['max_thrust'] > min(limits):
        raise ValueError('algorithm max_thrust exceeds physical actuator capacity')
    validate_scenario(scenario)

    resources = {str(Path(p).resolve()): hashlib.sha256(Path(p).read_bytes()).hexdigest()
                 for p in source_files}
    resolved = {'schema_version': 1, 'vessel': vessel, 'scenario': scenario,
                'algorithms': algorithms, 'resources': resources}
    json.dumps(resolved, allow_nan=False)  # reject NaN/Infinity anywhere
    return resolved


def autonomy_parameters(resolved):
    """ROS parameters for the reference nodes, keyed by executable name.

    This is the public projection handed to autonomy (public_parameters.json):
    tuning and physical actuator data only, never scenario truth.
    """
    vessel = resolved['vessel']
    algorithms = copy.deepcopy(resolved['algorithms'])
    result = {'guidance': algorithms['guidance'], 'planner': algorithms['planner']}
    table = thruster_table(vessel)
    # Positions are handed to allocation relative to the center of mass
    # (flattened xyz); the WAM-V's pinned VRX model is symmetric about x.
    center = vessel.get('center_of_mass_m', [0.0, 0.0, 0.0])
    forward = [t['forward_limit_n'] for t in table]
    reverse = [t['reverse_limit_n'] for t in table]
    topics = [t['topic'] for t in table]
    result['guidance'].update(
        thruster_topics=topics,
        thruster_positions=[p - c for t in table for p, c in zip(t['position_m'], center)],
        thruster_axes=[v for t in table for v in t['axis']],
        thruster_forward_limits=forward, thruster_reverse_limits=reverse)
    hull = vessel['hull'] if vessel['profile'] == 'njord' else WAMV_HULL
    result['command_guard'] = {'thruster_topics': topics,
                               'forward_limits': forward, 'reverse_limits': reverse,
                               'max_thrust': result['guidance']['max_thrust'],
                               'timeout_s': COMMAND_TIMEOUT_S}
    settings = sensor_settings(resolved)
    result['sensor_adapter'] = {'seed': resolved['scenario']['seed'],
                                'orientation_noise_rad': settings['imu_orientation_noise_rad'],
                                'gps_xy_std_m': settings['gps_horizontal_noise_m'],
                                'gps_z_std_m': settings['gps_vertical_noise_m']}
    # Inflate obstacles by the vessel's circumscribed radius plus the margin.
    result['mapper'] = {'inflation_m': math.hypot(hull['length_m'], hull['beam_m']) / 2
                        + algorithms['mapping']['safety_margin_m']}
    return result


def sensor_settings(resolved):
    """Flat sensor and thrust settings of the resolved vessel.

    Written to the run directory as vessel_config.yaml and used by the model
    generators. Includes ``max_thrust_n``, the smallest physical thruster limit.
    """
    vessel = resolved['vessel']
    if vessel['profile'] == 'wamv_reference':
        return copy.deepcopy(vessel['settings'])
    settings = copy.deepcopy(vessel['sensors']['settings'])
    settings['max_thrust_n'] = min(t[k] for t in vessel['thrusters'] for k in ('forward_limit_n', 'reverse_limit_n'))
    return settings
