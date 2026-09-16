"""Generate the scenario/model, run Gazebo and bridge physical sensors."""
from pathlib import Path
import os
import json
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from njord_sim.scenario import world_xml
from njord_sim.scenario_core import scenario_digest
from njord_sim.vessel import generate as generate_vessel
from njord_sim.run_manifest import publish_ready, write_manifest, atomic_text, freeze_resources
from njord_sim.configuration import resolve_configuration, autonomy_parameters, legacy_vessel_settings


def launch(context):
    p = lambda name: LaunchConfiguration(name).perform(context)
    out = Path(p('output_dir')).resolve()
    run_id = os.environ.get('RUN_ID', '')
    if not run_id:
        raise ValueError('Set a unique RUN_ID or use scripts/njord')
    if (out/'run_ready.json').exists():
        raise ValueError('Output directory contains a previous run; select a fresh OUTPUT_HOST')
    sources = {'vessel': p('vessel_config'), 'scenario': p('scenario'), 'algorithms': p('algorithms_config')}
    legacy = 'schema_version' not in yaml.safe_load(Path(sources['vessel']).read_text())
    resolved = resolve_configuration(sources['vessel'], sources['scenario'], sources['algorithms'],
                                     seed=int(p('seed')), environment=p('environment'), legacy_vessel=legacy)
    scenario = resolved['scenario']
    out.mkdir(parents=True, exist_ok=True)
    freeze_resources(out, resolved)
    if resolved['vessel']['profile'] == 'njord':
        from njord_sim.njord_model import generate
        urdf, _ = generate(out, resolved)
    else:
        urdf, _ = generate_vessel(out, resolved_config=legacy_vessel_settings(resolved))
    (out/'njord_course.sdf').write_text(world_xml(scenario, resolved['vessel']['profile'])+'\n')
    atomic_text(out/'resolved_scenario.json', json.dumps(scenario, indent=2, allow_nan=False)+'\n')
    (out/'scenario.sha256').write_text(scenario_digest(scenario)+'\n')
    public = autonomy_parameters(resolved)
    if legacy and os.environ.get('PROFILE', 'fast') == 'conservative':
        public['guidance']['max_speed'] = min(1.0, public['guidance']['max_speed'])
    manifest = write_manifest(out, run_id, resolved, sources, public)
    publish_ready(out, run_id, scenario, manifest)
    args = ['gz', 'sim', '-r', '-v', '3', '--seed', str(scenario['seed'])]
    if p('headless').lower() == 'true':
        args += ['-s', '--headless-rendering']
    args += [str(out/'njord_course.sdf')]
    sim = ExecuteProcess(cmd=args, output='screen')
    pose = scenario['start']
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-world', 'njord_course', '-name', 'wamv', '-file', str(out/'wamv.sdf'),
                            '-x', str(pose[0]), '-y', str(pose[1]), '-z', str(pose[2]),
                            '-R', str(pose[3]), '-P', str(pose[4]), '-Y', str(pose[5])])
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                  parameters=[{'use_sim_time': True, 'config_file': str(out/'bridges.yaml')}])
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
               parameters=[{'use_sim_time': True, 'robot_description': urdf}],
               remappings=[('/joint_states', '/wamv/joint_states')])
    def spawned(event, _):
        if event.returncode:
            return [EmitEvent(event=Shutdown(reason='WAM-V spawn failed'))]
        return []
    return [sim, bridge, rsp, RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=spawned)),
            spawn, RegisterEventHandler(OnProcessExit(target_action=sim,
                         on_exit=[EmitEvent(event=Shutdown(reason='Gazebo exited'))]))]


def generate_launch_description():
    share = Path(get_package_share_directory('njord_sim'))
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value=os.environ.get('SCENARIO', '/opt/njord/scenarios/reference.yaml')),
        DeclareLaunchArgument('seed', default_value=os.environ.get('SEED', '1')),
        DeclareLaunchArgument('environment', default_value=os.environ.get('ENVIRONMENT', 'calm')),
        DeclareLaunchArgument('output_dir', default_value=os.environ.get('OUTPUT_DIR', '/outputs')),
        DeclareLaunchArgument('headless', default_value=os.environ.get('HEADLESS', 'true')),
        DeclareLaunchArgument('vessel_config', default_value=os.environ.get('VESSEL_CONFIG', str(share/'config/vessel.yaml'))),
        DeclareLaunchArgument('algorithms_config', default_value=os.environ.get('ALGORITHMS_CONFIG', str(share/'config/algorithms.yaml'))),
        OpaqueFunction(function=launch)])
