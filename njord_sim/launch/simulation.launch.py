"""Generate the scenario/model, run Gazebo and bridge physical sensors."""
from pathlib import Path
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from njord_sim.scenario import generate as generate_scenario
from njord_sim.vessel import generate as generate_vessel


def launch(context):
    p = lambda name: LaunchConfiguration(name).perform(context)
    out = Path(p('output_dir')).resolve()
    scenario = generate_scenario(p('scenario'), out, seed=int(p('seed')), environment=p('environment'))
    urdf, _ = generate_vessel(out, p('vessel_config'))
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
        DeclareLaunchArgument('vessel_config', default_value=str(share/'config/vessel.yaml')),
        OpaqueFunction(function=launch)])
