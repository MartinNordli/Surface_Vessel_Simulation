"""Simulator service: resolve the run configuration, generate models, start Gazebo.

Steps, all before any autonomy process may start:
1. Resolve vessel + scenario + algorithms YAML into one validated configuration.
2. Freeze checksummed copies of the inputs and generate the vessel model, the
   world SDF and the bridge configuration into the run directory.
3. Write public_parameters.json (autonomy tuning, no course truth) and the run
   manifest, then atomically publish run_ready.json.
4. Launch Gazebo, spawn the vessel, bridge topics and publish the robot TF.

Launch arguments default to the environment variables set by compose.yaml.
"""
from pathlib import Path
import json
import os
import hashlib

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from njord_sim.configuration import autonomy_parameters, config_path, resolve_configuration, sensor_settings
from njord_sim.run_manifest import atomic_text, freeze_default_configuration, freeze_resources, publish_ready, write_manifest
from njord_sim.scenario import world_xml
from njord_sim.scenario_core import scenario_digest
from njord_sim.constants import GZ_MODEL_NAME
from njord_sim.vessel import generate as generate_wamv


def _launch(context):
    p = lambda name: LaunchConfiguration(name).perform(context)
    out = Path(p('output_dir')).resolve()
    run_id = os.environ.get('RUN_ID', '')
    if not run_id:
        raise ValueError('Set a unique RUN_ID or use scripts/njord')
    if (out / 'run_ready.json').exists():
        raise ValueError('Output directory contains a previous run; select a fresh OUTPUT_HOST')

    # Freeze defaults first: even node fallback values belong to this run.
    out.mkdir(parents=True, exist_ok=True)
    freeze_default_configuration(out, config_path('algorithms.yaml').parent)

    # 1. One validated configuration; every later step derives from it.
    sources = {'vessel': p('vessel_config'), 'scenario': p('scenario'), 'algorithms': p('algorithms_config')}
    resolved = resolve_configuration(sources['vessel'], sources['scenario'], sources['algorithms'],
                                     seed=int(p('seed')) if p('seed') else None, environment=p('environment'),
                                     profile=p('profile'), real_time_factor=float(p('real_time_factor')))
    scenario = resolved['scenario']
    sources['localization'] = str(config_path('localization.yaml'))
    if os.environ.get('ROS_PARAMS_FILE'):
        sources['ros_params'] = os.environ['ROS_PARAMS_FILE']
    for path in sources.values():
        resolved['resources'].setdefault(str(Path(path).resolve()), hashlib.sha256(Path(path).read_bytes()).hexdigest())

    # 2. Frozen inputs and generated model/world files.
    out.mkdir(parents=True, exist_ok=True)
    freeze_resources(out, resolved)
    if resolved['vessel']['profile'] == 'njord':
        from njord_sim.njord_model import generate as generate_njord
        urdf, _ = generate_njord(out, resolved)
    else:
        urdf, _ = generate_wamv(out, resolved_config=sensor_settings(resolved))
    (out / 'njord_course.sdf').write_text(world_xml(scenario, resolved['vessel']['profile'], resolved['run']['real_time_factor']) + '\n')
    atomic_text(out / 'resolved_scenario.json', json.dumps(scenario, indent=2, allow_nan=False) + '\n')
    (out / 'scenario.sha256').write_text(scenario_digest(scenario) + '\n')

    # 3. Public handoff to autonomy, sealed by the manifest digest.
    geometry = json.loads((out / 'self_geometry.json').read_text())
    resolved['resources'].update(geometry.get('resources', {}))
    public = autonomy_parameters(resolved)
    public['_profile'] = resolved['algorithms']['profile']
    public['mapper']['self_geometry_path'] = str(out / 'self_geometry.json')
    public['_timing'] = {'periods': resolved['sensor_max_period_s'],
                         'navigation': resolved['algorithms']['navigation']}
    manifest = write_manifest(out, run_id, resolved, sources, public)
    publish_ready(out, run_id, scenario, manifest)

    # 4. Gazebo, spawn, bridges and robot TF.
    args = ['gz', 'sim', '-r', '-v', '3', '--seed', str(scenario['seed'])]
    if p('headless').lower() == 'true':
        args += ['-s', '--headless-rendering']
    args += [str(out / 'njord_course.sdf')]
    sim = ExecuteProcess(cmd=args, output='screen')
    pose = scenario['start']
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-world', 'njord_course', '-name', GZ_MODEL_NAME, '-file', str(out / 'vessel.sdf'),
                            '-x', str(pose[0]), '-y', str(pose[1]), '-z', str(pose[2]),
                            '-R', str(pose[3]), '-P', str(pose[4]), '-Y', str(pose[5])])
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                  parameters=[{'use_sim_time': True, 'config_file': str(out / 'bridges.yaml')}])
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
               parameters=[{'use_sim_time': True, 'robot_description': urdf}])

    def spawned(event, _):
        if event.returncode:
            return [EmitEvent(event=Shutdown(reason='vessel spawn failed'))]
        return []

    # Shut the whole launch down if spawning fails or Gazebo exits.
    return [sim, bridge, rsp, RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=spawned)),
            spawn, RegisterEventHandler(OnProcessExit(target_action=sim,
                                                      on_exit=[EmitEvent(event=Shutdown(reason='Gazebo exited'))]))]


def launch(context):
    """Preserve startup failures without replacing artifacts from another run."""
    try:
        return _launch(context)
    except Exception as error:
        out = Path(LaunchConfiguration('output_dir').perform(context)).resolve()
        if not (out/'run_ready.json').exists():
            out.mkdir(parents=True, exist_ok=True)
            try:
                with (out/'startup_failure.json').open('x') as stream:
                    json.dump({'complete': False, 'error_type': type(error).__name__,
                               'reason': str(error), 'run_id': os.environ.get('RUN_ID', '')}, stream)
            except FileExistsError:
                pass
        raise


def generate_launch_description():
    config = config_path('algorithms.yaml').parent
    env = os.environ.get
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value=env('SCENARIO', '/opt/njord/scenarios/reference.yaml')),
        DeclareLaunchArgument('seed', default_value=env('SEED', '')),
        DeclareLaunchArgument('environment', default_value=env('ENVIRONMENT', 'calm')),
        DeclareLaunchArgument('profile', default_value=env('PROFILE', 'fast')),
        DeclareLaunchArgument('real_time_factor', default_value=env('REAL_TIME_FACTOR', '1.0'),
                              description='Target simulated seconds per wall second'),
        DeclareLaunchArgument('output_dir', default_value=env('OUTPUT_DIR', '/outputs')),
        DeclareLaunchArgument('headless', default_value=env('HEADLESS', 'true')),
        DeclareLaunchArgument('vessel_config', default_value=env('VESSEL_CONFIG', str(config / 'vessels/wamv.yaml'))),
        DeclareLaunchArgument('algorithms_config', default_value=env('ALGORITHMS_CONFIG', str(config / 'algorithms.yaml'))),
        OpaqueFunction(function=launch)])
