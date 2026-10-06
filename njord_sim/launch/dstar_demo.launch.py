"""Autonomy service: state estimation, reference autonomy and the command guard.

Always started: sensor_adapter (GPS/IMU noise and navigation health),
navsat_transform and command_guard, plus the navigation state source:

  state_source:=estimate (default) -> the two robot_localization EKFs
  state_source:=truth              -> truth_relay: simulator ground truth on
                                      /njord/odometry and TF (explicit truth mode)

Raw noisy GPS/IMU are published in both modes for a team's own estimator.

The reference chain depends on the course kind (course:=, from run_ready.json):

  course:=gates      mapper, perception, mission, planner and guidance
  course:=setpoints  setpoint_controller only; the evaluator issues the
                     targets on /njord/setpoint (constants.SETPOINT_TOPIC)

Reference nodes that a team can replace with its own ('external'):
  autonomy:=external    -> every reference node of the chain
  controller:=external  -> guidance or setpoint_controller
  perception:=external  -> perception (gate course)
  mapping:=external     -> mapper (gate course)

run_mode:=race (default) lets the guard pass thrust only while the evaluator
reports the race active; run_mode:=free (./scripts/njord lab) drives without
one. What the guard requires comes from configuration.guard_requirements.

Parameter precedence for each reference node, lowest to highest:
  node defaults < ROS_PARAMS_FILE < public_parameters.json < use_sim_time
  (and, for the guard, its requirements from run_mode and autonomy).
public_parameters.json is written by the simulator from the resolved vessel and
algorithms.yaml, so vessel limits and algorithm tuning always come from those
files. scripts/run_autonomy.py passes it automatically.
"""
from pathlib import Path
import json
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from njord_sim.configuration import (COURSE_KINDS, DEFAULT_PROFILE, RUN_MODES, config_path, guard_requirements,
                                     speed_profile_names, validate_reference_timing)
from njord_sim.constants import GPS_TOPIC, IMU_TOPIC, WORLD_ORIGIN_WGS84

COMPONENTS = ('autonomy', 'controller', 'perception', 'mapping')
STATE_SOURCES = ('estimate', 'truth')


def launch(context):
    p = lambda name: LaunchConfiguration(name).perform(context)
    for component in COMPONENTS:
        if p(component) not in ('reference', 'external'):
            raise ValueError(f'{component} must be reference or external')
    if p('state_source') not in STATE_SOURCES:
        raise ValueError('state_source must be estimate or truth')
    if p('course') not in COURSE_KINDS:
        raise ValueError(f'course must be one of {COURSE_KINDS}')
    components = {name: p(name) for name in (*COMPONENTS, 'course')}
    requirements = guard_requirements(components, p('run_mode'))
    public_path = context.launch_configurations.get('public_parameters', '')
    public = json.loads(Path(public_path).read_text()) if public_path else {}
    # The simulator resolved the profile against algorithms.yaml; without its
    # handoff the nodes use the default algorithms.yaml, so check against that.
    profiles = [public['_profile']] if '_profile' in public else speed_profile_names(config_path('algorithms.yaml'))
    if p('profile') not in profiles:
        raise ValueError(f'profile must be one of {profiles}')
    params_file = p('params_file')
    if params_file and not Path(params_file).is_file():
        raise ValueError(f'params_file does not exist: {params_file}')
    if public.get('_timing'):
        validate_reference_timing(public, public['_timing']['periods'],
                                  public['_timing']['navigation'], components)
    common = {'use_sim_time': True}  # always last: nodes must run on /clock
    overrides = [params_file] if params_file else []
    actions = []
    if not public:
        get_logger("dstar_demo").warning(
            'No public_parameters given; reference nodes use their built-in defaults '
            'instead of the resolved vessel and algorithms.yaml.')

    def node(executable, params=None, enforced=None):
        # ``enforced`` values come last with use_sim_time: no file can weaken them.
        return Node(package='njord_sim', executable=executable, output='screen',
                    parameters=[params or {}, *overrides, public.get(executable, {}),
                                {**(enforced or {}), **common}])

    # Navigation state (always runs, also with a fully external autonomy stack).
    actions.append(node('sensor_adapter', {'seed': int(p('seed'))}))
    localization = context.launch_configurations.get('localization_config', '') or str(config_path('localization.yaml'))
    if p('state_source') == 'truth':
        # Explicit truth mode: ground truth replaces the EKF estimate.
        actions.append(node('truth_relay'))
    else:
        for name, output in [('ekf_local', '/njord/local/odometry'), ('ekf_global', '/njord/odometry')]:
            actions.append(Node(package='robot_localization', executable='ekf_node', name=name, output='screen',
                                parameters=[localization, *overrides, common],
                                remappings=[('odometry/filtered', output)]))
    actions.append(Node(package='robot_localization', executable='navsat_transform_node', name='navsat',
                        output='screen',
                        parameters=[localization, *overrides, {'datum': list(WORLD_ORIGIN_WGS84)}, common],
                        remappings=[('imu', IMU_TOPIC),
                                    ('gps/fix', GPS_TOPIC),
                                    ('odometry/filtered', '/njord/odometry'),
                                    ('odometry/gps', '/njord/gps/odometry')]))

    # Reference autonomy chain; each part can be left out for a team's own node.
    if p('autonomy') == 'reference' and p('course') == 'setpoints':
        if p('controller') == 'reference':
            actions.append(node('setpoint_controller'))
    elif p('autonomy') == 'reference':
        if p('mapping') == 'reference':
            actions.append(node('mapper'))
        if p('perception') == 'reference':
            actions.append(node('perception'))
        actions += [node('mission', {'expected_gates': int(p('expected_gates'))}), node('planner')]
        if p('controller') == 'reference':
            actions.append(node('guidance'))

    # The guard is the single actuator authority and always runs.
    actions.append(node('command_guard', enforced=requirements))
    return actions


def generate_launch_description():
    env = os.environ.get
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value=env('PROFILE', DEFAULT_PROFILE),
                              description='Recorded speed profile; the speed itself comes from public_parameters'),
        DeclareLaunchArgument('seed', default_value=env('SEED', '1')),
        DeclareLaunchArgument('expected_gates', default_value='3'),
        DeclareLaunchArgument('course', default_value='gates',
                              description=' or '.join(COURSE_KINDS) + ': selects the reference chain'),
        *[DeclareLaunchArgument(name, default_value=env(name.upper(), 'reference'),
                                description='reference or external') for name in COMPONENTS],
        DeclareLaunchArgument('state_source', default_value=env('STATE_SOURCE', 'estimate'),
                              description='estimate (EKFs) or truth (simulator ground truth on /njord/odometry)'),
        DeclareLaunchArgument('run_mode', default_value=env('RUN_MODE', 'race'),
                              description=' or '.join(RUN_MODES) + ': race needs an active evaluator'),
        DeclareLaunchArgument('params_file', default_value=env('ROS_PARAMS_FILE', '')),
        DeclareLaunchArgument('public_parameters', default_value=''),
        DeclareLaunchArgument('localization_config', default_value=''),
        DeclareLaunchArgument('vessel_config', default_value='',
                              description="The run's resolved vessel_config.yaml (provenance only)"),
        OpaqueFunction(function=launch)])
