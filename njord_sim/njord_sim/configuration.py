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

from .constants import COMMAND_TIMEOUT_S, PROCESS_LIVENESS_S, THRUSTER_COMMAND_TOPIC, WAMV_HULL, WAMV_THRUSTERS
from .control_core import allocation_matrix, independent_rows
from .mission_core import SEARCH_BEARINGS_DEG
from .mesh_geometry import geometry_vertices, geometry_volume, load_obj, validate_disjoint_volumes
from .scenario_core import COURSE_KINDS, validate_scenario

# Sensor settings shared by every vessel profile.
SENSOR_KEYS = {
    'camera_width', 'camera_height', 'camera_rate', 'camera_horizontal_fov_rad',
    'camera_noise_stddev', 'lidar_range', 'lidar_samples', 'lidar_vertical_samples',
    'lidar_rate', 'lidar_noise_stddev', 'gps_rate', 'imu_rate', 'gps_horizontal_noise_m',
    'gps_vertical_noise_m', 'imu_orientation_noise_rad',
    'imu_angular_velocity_noise_rad_s', 'imu_linear_acceleration_noise_m_s2',
}
# The WAM-V profile exposes the sensors plus its thrust limit (vessels/wamv.yaml).
WAMV_SETTING_KEYS = SENSOR_KEYS | {'max_thrust_n'}
GUIDANCE_KEYS = {
    'lookahead_m', 'kp_yaw', 'kd_yaw', 'kp_surge', 'max_thrust', 'goal_tolerance_m',
    'control_hz', 'stale_after_s', 'braking_deceleration_mps2', 'reaction_time_s',
    'stopping_margin_m',
}
# Reference mission geometry, freshness, search and retry (algorithms.yaml ``mission``).
MISSION_KEYS = {
    'min_gate_width_m', 'max_gate_width_m', 'approach_m', 'exit_m', 'arrival_tolerance_m',
    'detection_max_age_s', 'crossing_memory_s', 'crossing_entry_m', 'search_after_s',
    'search_radius_m', 'search_goal_s', 'search_timeout_s', 'max_gate_retries', 'retry_clearance_m',
}
# Reference setpoint controller (algorithms.yaml ``setpoint_control``).
SETPOINT_CONTROL_KEYS = {
    'control_hz', 'stale_after_s', 'approach_radius_m', 'kp_surge', 'kp_yaw', 'kd_yaw',
    'kp_position', 'kd_position', 'braking_deceleration_mps2', 'reaction_time_s',
}
# Algorithm values that may legitimately be zero; everything else must be > 0.
NONNEGATIVE_ALGORITHM_KEYS = {
    'kp_yaw', 'kd_yaw', 'kp_surge', 'reaction_time_s', 'stopping_margin_m', 'safety_margin_m',
    'self_filter_margin_m', 'search_after_s', 'max_gate_retries', 'kp_position', 'kd_position',
}
ENVIRONMENT_KEYS = {
    'wind_speed_mps', 'wind_direction_to_deg_enu', 'wind_variance_gain', 'wave_gain',
    'wave_period_s', 'wave_direction_rad', 'wave_steepness', 'current_speed_mps',
    'current_direction_to_deg_enu', 'water_density_kg_m3', 'water_level_m', 'physics_step_s',
}
DEFAULT_PROFILE = 'fast'
# Fixed occupancy grid of the reference mapper, in algorithms.yaml ``mapping``.
MAP_GRID_KEYS = {'grid_resolution_m', 'grid_size_m', 'grid_origin_m'}


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

    When NJORD_CONFIG_DIR is set, require the file there. Managed runs use
    the frozen workspace defaults. Unmanaged host tools can use the source
    tree or installed package when no configuration directory was specified.
    """
    candidates = []
    if os.environ.get('NJORD_CONFIG_DIR'):
        path = Path(os.environ['NJORD_CONFIG_DIR']) / relative
        if not path.is_file():
            raise ValueError(f'required mounted configuration is missing: {path}')
        return path
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
    return {'schema_version': 2, 'profile': 'wamv_reference', 'settings': copy.deepcopy(settings)}


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


def convert_sensor_schema(vessel):
    """Explicit upgrade: WAM-V 1->2, Njord 2->3; new noise is assumed.

    Values reproduce prior nominal Gazebo IMU noise, with bias removed. They
    are engineering defaults, never a claim of measured vessel calibration.
    """
    result = copy.deepcopy(vessel)
    old = 2 if result.get('profile') == 'njord' else 1
    if result.get('schema_version') != old:
        raise ValueError('sensor conversion expects the previous vessel schema')
    settings = result['sensors']['settings'] if old == 2 else result['settings']
    for key, value in {'imu_angular_velocity_noise_rad_s': .009,
                       'imu_linear_acceleration_noise_m_s2': .021}.items():
        if key in settings:
            raise ValueError(f'{key} requires the new vessel schema')
        settings[key] = value
    result['schema_version'] = old + 1
    return result


def validate_vessel(vessel, resource_base=None, resource_files=None):
    """Validate a versioned vessel (``wamv_reference`` or ``njord``) in place.

    ``resource_base`` resolves relative mesh paths; loaded mesh paths are
    appended to ``resource_files`` so the run manifest can checksum them.
    WAM-V files are schema 2 and Njord files schema 3. Previous schemas
    are explicitly converted on copies, preserving the source files.
    """
    if vessel.get('profile') == 'njord' and vessel.get('schema_version') == 1:
        vessel = convert_legacy_vessel(vessel)
    if vessel.get('schema_version') == (2 if vessel.get('profile') == 'njord' else 1):
        vessel = convert_sensor_schema(vessel)
    _version(vessel, 3 if vessel.get('profile') == 'njord' else 2)
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


# Acceptance and timing of one target pose; each may be set per setpoint or
# in ``setpoints.defaults`` (there is no built-in fallback).
SETPOINT_SETTING_KEYS = ('tolerance_m', 'heading_tolerance_deg', 'hold_s', 'timeout_s')


def _resolve_setpoints(setpoints):
    """Validate ``setpoints`` and return the explicit, fully populated sequence.

    Each target gets ``name``, ``position`` [x, y] (m, map ENU),
    ``heading_deg_enu`` (counter-clockwise from east, or None when the heading
    is free), the SETPOINT_SETTING_KEYS from the entry or ``defaults``, and
    ``advance_after_s`` (None, or the time after which the next target is
    issued whether or not this one was reached; such a target is not required
    for completion).
    """
    _keys(setpoints, {'sequence'}, {'defaults'}, where='setpoints')
    defaults = setpoints.get('defaults', {})
    _keys(defaults, (), SETPOINT_SETTING_KEYS, where='setpoints.defaults')
    if not isinstance(setpoints['sequence'], list) or not setpoints['sequence']:
        raise ValueError('setpoints.sequence must be a nonempty list')
    result, names = [], set()
    for entry in setpoints['sequence']:
        _keys(entry, {'name', 'position'}, {'heading_deg_enu', 'advance_after_s', *SETPOINT_SETTING_KEYS},
              where='setpoint')
        if not isinstance(entry['name'], str) or not entry['name'] or entry['name'] in names:
            raise ValueError('setpoint names must be unique nonempty strings')
        names.add(entry['name'])
        _vector(entry['position'], 2, f"{entry['name']}.position")
        item = {'name': entry['name'], 'position': list(entry['position']),
                'heading_deg_enu': entry.get('heading_deg_enu'),
                'advance_after_s': entry.get('advance_after_s')}
        if item['heading_deg_enu'] is not None:
            _number(item['heading_deg_enu'], f"{entry['name']}.heading_deg_enu")
        if item['advance_after_s'] is not None:
            _number(item['advance_after_s'], f"{entry['name']}.advance_after_s", positive=True)
        for key in SETPOINT_SETTING_KEYS:
            if key not in entry and key not in defaults:
                raise ValueError(f"setpoint {entry['name']}: {key} missing (set it or setpoints.defaults)")
            item[key] = _number(entry.get(key, defaults.get(key)), f"{entry['name']}.{key}",
                                0, positive=key != 'hold_s')
        if item['heading_tolerance_deg'] > 180:
            raise ValueError('heading_tolerance_deg must not exceed 180')
        result.append(item)
    if result[-1]['advance_after_s'] is not None:
        raise ValueError('the last setpoint must be reached; it cannot use advance_after_s')
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
    _keys(data, {'schema_version', 'name', 'start', 'timeout_s', 'environments'},
          {'seed', 'hull', 'gate_y_jitter_m', 'obstacles', 'gates', 'setpoints'}, where='scenario')
    if ('gates' in data) == ('setpoints' in data):
        raise ValueError('scenario needs exactly one of gates (a race) or setpoints (target poses)')
    data['kind'] = 'gates' if 'gates' in data else 'setpoints'
    if data['kind'] == 'setpoints':
        if data.get('gate_y_jitter_m', 0):
            raise ValueError('gate_y_jitter_m applies to gates only')
        data['setpoints'] = _resolve_setpoints(data['setpoints'])
        data['gates'] = []
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
    if not isinstance(data['gates'], list) or data['kind'] == 'gates' and not data['gates']:
        raise ValueError('gates must be a nonempty list')
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

def convert_algorithm_schema(algorithms):
    """Upgrade an older algorithms file to schema 5, one version at a time.

    1->2 introduces the assumed timing and filter tolerances. 2->3 makes the
    occupancy grid explicit with the values the mapper used to build in:
    0.5 m cells, a 160 m square, lower-left corner at (-40, -40) m. 3->4 adds
    ``mission`` with the geometry the mission node used to build in and
    search and retry disabled (no search, no retries), which is the previous
    behaviour. 4->5 adds ``setpoint_control`` with the shipped starting gains;
    gate courses do not use it.
    """
    result = copy.deepcopy(algorithms)
    if result.get('schema_version') == 1:
        if 'navigation' in result or 'self_filter_margin_m' in result.get('mapping', {}):
            raise ValueError('new timing/filter settings require algorithms schema 2')
        result['schema_version'] = 2
        result['mapping']['self_filter_margin_m'] = .02
        result['navigation'] = dict(stale_after_s=.5, processing_margin_s=.1,
                                    clock_stall_after_s=.5, sync_slop_s=.12)
    if result.get('schema_version') == 2:
        if MAP_GRID_KEYS & set(result.get('mapping', {})):
            raise ValueError('occupancy grid settings require algorithms schema 3')
        result['schema_version'] = 3
        result['mapping'].update(grid_resolution_m=.5, grid_size_m=160., grid_origin_m=[-40., -40.])
    if result.get('schema_version') == 3:
        if 'mission' in result:
            raise ValueError('mission settings require algorithms schema 4')
        result['schema_version'] = 4
        # search_after_s beyond any race and no retries reproduce schema 3.
        result['mission'] = dict(min_gate_width_m=8., max_gate_width_m=30., approach_m=5., exit_m=6.,
                                 arrival_tolerance_m=2., detection_max_age_s=2., crossing_memory_s=45.,
                                 crossing_entry_m=15., search_after_s=1e9, search_radius_m=8.,
                                 search_goal_s=10., search_timeout_s=60., max_gate_retries=0,
                                 retry_clearance_m=6.)
    if result.get('schema_version') == 4:
        if 'setpoint_control' in result:
            raise ValueError('setpoint controller settings require algorithms schema 5')
        result['schema_version'] = 5
        result['setpoint_control'] = dict(control_hz=20., stale_after_s=.5, approach_radius_m=5.,
                                          kp_surge=200., kp_yaw=400., kd_yaw=300., kp_position=100.,
                                          kd_position=200., braking_deceleration_mps2=.25,
                                          reaction_time_s=1.)
    _version(result, 5)
    return result


# RUN_MODE: 'race' scores a course and only allows thrust while the evaluator
# reports the race active; 'free' runs without an evaluator (./scripts/njord lab).
RUN_MODES = ('race', 'free')


def guard_requirements(components, run_mode):
    """What the command guard (and, in a race, the evaluator) requires before thrust.

    ``components`` maps 'autonomy' and 'controller' to 'reference' or
    'external', and 'course' to a scenario kind (COURSE_KINDS, default
    'gates'). Navigation always runs, so its heartbeat is always required.
    The other heartbeats are required only when the simulator starts the
    reference node that publishes them: planner and mission on a gate course,
    the setpoint controller on a setpoint course. An external stack only has
    to send fresh thruster commands and zero them itself on bad input. The
    evaluator's run-active signal is required only in a race. Returns
    {'required_status': [...], 'require_race_active': bool}.
    """
    if run_mode not in RUN_MODES:
        raise ValueError(f'RUN_MODE must be one of {RUN_MODES}')
    autonomy = components.get('autonomy', 'reference')
    controller = components.get('controller', 'reference')
    course = components.get('course', 'gates')
    if autonomy not in ('reference', 'external') or controller not in ('reference', 'external'):
        raise ValueError('autonomy and controller must be reference or external')
    if course not in COURSE_KINDS:
        raise ValueError(f'course must be one of {COURSE_KINDS}')
    status = ['navigation']
    if autonomy == 'reference' and course == 'gates':
        status += ['planner', 'mission']
    elif autonomy == 'reference' and controller == 'reference':
        status += ['controller']
    return {'required_status': status, 'require_race_active': run_mode == 'race'}


def inflation_m(hull, algorithms):
    """Obstacle inflation (m): the hull's circumscribed radius plus the safety margin."""
    return math.hypot(hull['length_m'], hull['beam_m']) / 2 + algorithms['mapping']['safety_margin_m']


def speed_profile_names(algorithms_file):
    """Sorted ``PROFILE`` names defined by ``speed_profiles_mps`` in an algorithms file."""
    profiles = _read(algorithms_file).get('speed_profiles_mps')
    if not isinstance(profiles, dict) or not profiles:
        raise ValueError('speed_profiles_mps must map profile names to speeds')
    return sorted(profiles)


# Sections of algorithms.yaml a vessel may override (``vessel_overrides``).
VESSEL_OVERRIDE_SECTIONS = ('guidance', 'mapping', 'mission', 'setpoint_control')


def vessel_key(vessel):
    """Name that selects a vessel's ``vessel_overrides`` entry: 'wamv' or the vessel ``name``."""
    return 'wamv' if vessel['profile'] == 'wamv_reference' else vessel['name']


def _apply_vessel_overrides(algorithms, vessel_name):
    """Merge the ``vessel_overrides`` entry for ``vessel_name`` into ``algorithms``.

    Every entry is checked, applied or not: only VESSEL_OVERRIDE_SECTIONS and
    keys that exist in the shared section are allowed, so a misspelled
    setting cannot silently do nothing. The merged values are validated with
    the shared rules afterwards. Returns the applied vessel name, or None.
    """
    overrides = algorithms.pop('vessel_overrides', None) or {}
    if not isinstance(overrides, dict):
        raise ValueError('vessel_overrides must map vessel names to sections')
    for name, sections in overrides.items():
        _keys(sections, (), VESSEL_OVERRIDE_SECTIONS, where=f'vessel_overrides.{name}')
        for section, values in sections.items():
            _keys(values, (), algorithms[section].keys(), where=f'vessel_overrides.{name}.{section}')
    if vessel_name not in overrides:
        return None
    for section, values in overrides[vessel_name].items():
        algorithms[section].update(copy.deepcopy(values))
    return vessel_name


def _resolve_algorithms(algorithms, profile, vessel_name='wamv'):
    """Validate algorithms.yaml, apply the vessel's overrides and select ``profile``'s speed ceiling."""
    algorithms = copy.deepcopy(algorithms)
    if algorithms.get('schema_version') in (1, 2, 3, 4):
        algorithms = convert_algorithm_schema(algorithms)
    _version(algorithms, 5)
    _keys(algorithms, {'schema_version', 'speed_profiles_mps', 'guidance', 'planner', 'mapping', 'navigation',
                       'mission', 'setpoint_control'}, {'vessel_overrides'}, where='algorithms')
    for section in VESSEL_OVERRIDE_SECTIONS:
        if not isinstance(algorithms[section], dict):
            raise ValueError(f'{section} must be a mapping')
    applied = _apply_vessel_overrides(algorithms, vessel_name)
    profiles = algorithms['speed_profiles_mps']
    if not isinstance(profiles, dict) or not profiles:
        raise ValueError('speed_profiles_mps must map profile names to speeds')
    for name, speed in profiles.items():
        _number(speed, f'speed_profiles_mps.{name}', positive=True)
    if profile not in profiles:
        raise ValueError(f'unknown PROFILE {profile!r}; algorithms.yaml defines {sorted(profiles)}')
    _keys(algorithms['guidance'], GUIDANCE_KEYS, where='guidance')
    _keys(algorithms['planner'], {'stale_after_s', 'publish_hz'}, where='planner')
    _keys(algorithms['mapping'], {'safety_margin_m', 'self_filter_margin_m', *MAP_GRID_KEYS}, where='mapping')
    # The grid corner may be negative; every other algorithm value is a scalar >= 0.
    _vector(algorithms['mapping']['grid_origin_m'], 2, 'grid_origin_m')
    _keys(algorithms['navigation'], {'stale_after_s', 'processing_margin_s', 'clock_stall_after_s', 'sync_slop_s'}, where='navigation')
    _keys(algorithms['mission'], MISSION_KEYS, where='mission')
    _keys(algorithms['setpoint_control'], SETPOINT_CONTROL_KEYS, where='setpoint_control')
    retries = algorithms['mission']['max_gate_retries']
    if type(retries) is not int:
        raise ValueError('max_gate_retries must be an integer')
    if algorithms['mission']['min_gate_width_m'] > algorithms['mission']['max_gate_width_m']:
        raise ValueError('min_gate_width_m must not exceed max_gate_width_m')
    if algorithms['mission']['search_goal_s'] * len(SEARCH_BEARINGS_DEG) > algorithms['mission']['search_timeout_s']:
        raise ValueError('search_timeout_s must leave search_goal_s for every search bearing')
    for group in ('guidance', 'planner', 'mapping', 'navigation', 'mission', 'setpoint_control'):
        for key, value in algorithms[group].items():
            if key != 'grid_origin_m':
                _number(value, key, 0, positive=key not in NONNEGATIVE_ALGORITHM_KEYS)
    algorithms['profile'] = profile
    algorithms['vessel_override'] = applied
    algorithms['guidance']['max_speed'] = profiles[profile]
    return algorithms


# --------------------------------------------------------------------------
# Complete run configuration
# --------------------------------------------------------------------------

def resolve_configuration(vessel_file, scenario_file, algorithms_file, seed=None,
                          environment=None, profile=None, real_time_factor=1.0):
    """Resolve and cross-check the vessel, scenario and algorithm files.

    A vessel file without ``schema_version`` is a partial WAM-V override: its
    keys replace those in vessels/wamv.yaml. ``profile`` selects a speed profile
    from algorithms.yaml (default 'fast'). ``real_time_factor`` is Gazebo's
    target of simulated seconds per wall second (a run option,
    REAL_TIME_FACTOR). Returns a JSON-serializable dict with ``vessel``,
    ``scenario``, ``algorithms``, ``run`` (the run options) and SHA256 digests
    of every source file under ``resources``.
    """
    _number(real_time_factor, 'real_time_factor', positive=True)
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
    algorithms = _resolve_algorithms(_read(algorithms_file), profile or DEFAULT_PROFILE, vessel_key(vessel))
    env = scenario['environment']
    settings = vessel['settings'] if vessel['profile'] == 'wamv_reference' else vessel['sensors']['settings']
    sensor_periods = sensor_timing(settings, env['physics_step_s'])

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
    # A retry waypoint inside the inflated buoy would be an invalid planner
    # endpoint; 1 m allows for the buoy's own radius.
    hull = vessel['hull'] if vessel['profile'] == 'njord' else WAMV_HULL
    if algorithms['mission']['retry_clearance_m'] < inflation_m(hull, algorithms) + 1.0:
        raise ValueError('mission.retry_clearance_m must exceed the obstacle inflation by 1 m')
    validate_scenario(scenario)

    resources = {str(Path(p).resolve()): hashlib.sha256(Path(p).read_bytes()).hexdigest()
                 for p in source_files}
    resolved = {'schema_version': 1, 'vessel': vessel, 'scenario': scenario,
                'algorithms': algorithms, 'run': {'real_time_factor': float(real_time_factor)},
                'resources': resources,
                'sensor_max_period_s': sensor_periods}
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
                               'timeout_s': COMMAND_TIMEOUT_S, 'liveness_s': PROCESS_LIVENESS_S}
    settings = sensor_settings(resolved)
    result['sensor_adapter'] = {'seed': resolved['scenario']['seed'],
                                'orientation_noise_rad': settings['imu_orientation_noise_rad'],
                                'angular_velocity_noise_rad_s': settings['imu_angular_velocity_noise_rad_s'],
                                'linear_acceleration_noise_m_s2': settings['imu_linear_acceleration_noise_m_s2'],
                                'stale_after_s': algorithms['navigation']['stale_after_s'],
                                'clock_stall_after_s': algorithms['navigation']['clock_stall_after_s'],
                                'gps_xy_std_m': settings['gps_horizontal_noise_m'],
                                'gps_z_std_m': settings['gps_vertical_noise_m']}
    mapping = algorithms['mapping']
    result['mapper'] = {'inflation_m': inflation_m(hull, algorithms),
                        # float(): a YAML integer would not match the declared ROS double.
                        'resolution': float(mapping['grid_resolution_m']),
                        'size_m': float(mapping['grid_size_m']),
                        'origin_x': float(mapping['grid_origin_m'][0]),
                        'origin_y': float(mapping['grid_origin_m'][1]),
                        'max_range_m': settings['lidar_range'],
                        'input_max_age_s': algorithms['navigation']['stale_after_s'],
                        'self_filter_margin_m': mapping['self_filter_margin_m'],
                        'lidar_noise_stddev_m': settings['lidar_noise_stddev']}
    # float(): a YAML integer would not match the declared ROS double.
    result['mission'] = {**{key: value if key == 'max_gate_retries' else float(value)
                            for key, value in algorithms['mission'].items()},
                         'camera_max_age_s': algorithms['navigation']['stale_after_s'],
                         'odometry_max_age_s': algorithms['navigation']['stale_after_s']}
    result['perception'] = {'sync_tolerance_s': algorithms['navigation']['sync_slop_s'],
                            'input_max_age_s': algorithms['navigation']['stale_after_s']}
    # The setpoint controller shares the speed ceiling, the thrust cap the
    # guard enforces and the thruster layout with guidance.
    result['setpoint_controller'] = {
        **{key: float(value) for key, value in algorithms['setpoint_control'].items()},
        'max_speed': float(result['guidance']['max_speed']),
        'max_thrust': float(result['guidance']['max_thrust']),
        **{key: result['guidance'][key] for key in ('thruster_topics', 'thruster_positions', 'thruster_axes',
                                                    'thruster_forward_limits', 'thruster_reverse_limits')}}
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


def sensor_timing(settings, physics_step):
    """Bound quantized sensor periods; rates above physics rate are invalid.

    Nonintegral periods are allowed; the bound rounds up to the next physics
    tick. Thus 100 Hz at 4 ms has a conservative 12 ms acquisition interval.
    """
    result = {}
    for sensor in ('camera', 'lidar', 'gps', 'imu'):
        rate = settings[sensor + '_rate']
        if rate * physics_step > 1 + 1e-9:
            raise ValueError(f'{sensor}_rate={rate} Hz cannot be represented with physics_step_s={physics_step}')
        result[sensor] = math.ceil(1 / (rate * physics_step) - 1e-9) * physics_step
    return result


def validate_reference_timing(public, periods, navigation, components):
    """Check effective ROS parameters only for reference components being used."""
    margin = navigation['processing_margin_s']
    def budget(node, key, sensor):
        actual = public[node][key]
        required = 2 * periods[sensor] + margin
        if actual + 1e-12 < required:
            raise ValueError(f'{node}.{key}={actual}s incompatible with {sensor} period<={periods[sensor]}s: requires {required}s (two periods + processing_margin_s={margin})')
    for sensor in ('gps', 'imu'):
        budget('sensor_adapter', 'stale_after_s', sensor)
    if components.get('autonomy') != 'reference':
        return
    if components.get('course', 'gates') == 'setpoints':
        # Only the setpoint controller runs; it consumes odometry alone.
        if components.get('controller') == 'reference':
            budget('setpoint_controller', 'stale_after_s', 'imu')
        return
    budget('mission', 'odometry_max_age_s', 'imu')
    if components.get('perception') == 'reference':
        budget('mission', 'camera_max_age_s', 'camera')
        for sensor in ('camera', 'lidar'):
            budget('perception', 'input_max_age_s', sensor)
        tolerance = public['perception']['sync_tolerance_s']
        if tolerance < max(periods['camera'], periods['lidar']):
            raise ValueError(f'perception.sync_tolerance_s={tolerance} cannot cover camera/lidar periods {periods}')
    if components.get('mapping') == 'reference':
        budget('mapper', 'input_max_age_s', 'lidar')
        budget('planner', 'stale_after_s', 'lidar')
        if components.get('controller') == 'reference':
            budget('guidance', 'stale_after_s', 'lidar')
