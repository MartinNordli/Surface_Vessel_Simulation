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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from njord_sim.configuration import autonomy_parameters, resolve_configuration, sensor_settings
from njord_sim.run_manifest import atomic_text, freeze_resources, publish_ready, write_manifest
from njord_sim.scenario import world_xml
from njord_sim.scenario_core import scenario_digest
from njord_sim.vessel import generate as generate_wamv


def launch(context):
    p = lambda name: LaunchConfiguration(name).perform(context)
    out = Path(p('output_dir')).resolve()
    run_id = os.environ.get('RUN_ID', '')
    if not run_id:
        raise ValueError('Set a unique RUN_ID or use scripts/njord')
    if (out / 'run_ready.json').exists():
        raise ValueError('Output directory contains a previous run; select a fresh OUTPUT_HOST')

    # 1. One validated configuration; every later step derives from it.
    sources = {'vessel': p('vessel_config'), 'scenario': p('scenario'), 'algorithms': p('algorithms_config')}
    resolved = resolve_configuration(sources['vessel'], sources['scenario'], sources['algorithms'],
                                     seed=int(p('seed')), environment=p('environment'),
                                     profile=p('profile'))
    scenario = resolved['scenario']

    # 2. Frozen inputs and generated model/world files.
    out.mkdir(parents=True, exist_ok=True)
    freeze_resources(out, resolved)
    if resolved['vessel']['profile'] == 'njord':
        from njord_sim.njord_model import generate as generate_njord
        urdf, _ = generate_njord(out, resolved)
    else:
        urdf, _ = generate_wamv(out, resolved_config=sensor_settings(resolved))
    (out / 'njord_course.sdf').write_text(world_xml(scenario, resolved['vessel']['profile']) + '\n')
    atomic_text(out / 'resolved_scenario.json', json.dumps(scenario, indent=2, allow_nan=False) + '\n')
    (out / 'scenario.sha256').write_text(scenario_digest(scenario) + '\n')

    # 3. Public handoff to autonomy, sealed by the manifest digest.
    manifest = write_manifest(out, run_id, resolved, sources, autonomy_parameters(resolved))
    publish_ready(out, run_id, scenario, manifest)

    # 4. Gazebo, spawn, bridges and robot TF.
    args = ['gz', 'sim', '-r', '-v', '3', '--seed', str(scenario['seed'])]
    if p('headless').lower() == 'true':
        args += ['-s', '--headless-rendering']
    args += [str(out / 'njord_course.sdf')]
    sim = ExecuteProcess(cmd=args, output='screen')
    pose = scenario['start']
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-world', 'njord_course', '-name', 'wamv', '-file', str(out / 'wamv.sdf'),
                            '-x', str(pose[0]), '-y', str(pose[1]), '-z', str(pose[2]),
                            '-R', str(pose[3]), '-P', str(pose[4]), '-Y', str(pose[5])])
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                  parameters=[{'use_sim_time': True, 'config_file': str(out / 'bridges.yaml')}])
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
               parameters=[{'use_sim_time': True, 'robot_description': urdf}],
               remappings=[('/joint_states', '/wamv/joint_states')])

    def spawned(event, _):
        if event.returncode:
            return [EmitEvent(event=Shutdown(reason='WAM-V spawn failed'))]
        return []

    # Shut the whole launch down if spawning fails or Gazebo exits.
    return [sim, bridge, rsp, RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=spawned)),
            spawn, RegisterEventHandler(OnProcessExit(target_action=sim,
                                                      on_exit=[EmitEvent(event=Shutdown(reason='Gazebo exited'))]))]


def generate_launch_description():
    config = Path(get_package_share_directory('njord_sim')) / 'config'
    env = os.environ.get
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value=env('SCENARIO', '/opt/njord/scenarios/reference.yaml')),
        DeclareLaunchArgument('seed', default_value=env('SEED', '1')),
        DeclareLaunchArgument('environment', default_value=env('ENVIRONMENT', 'calm')),
        DeclareLaunchArgument('profile', default_value=env('PROFILE', 'fast')),
        DeclareLaunchArgument('output_dir', default_value=env('OUTPUT_DIR', '/outputs')),
        DeclareLaunchArgument('headless', default_value=env('HEADLESS', 'true')),
        DeclareLaunchArgument('vessel_config', default_value=env('VESSEL_CONFIG', str(config / 'vessels/wamv.yaml'))),
        DeclareLaunchArgument('algorithms_config', default_value=env('ALGORITHMS_CONFIG', str(config / 'algorithms.yaml'))),
        OpaqueFunction(function=launch)])
