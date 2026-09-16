"""Sensor-based Njord reference autonomy against the simulator service."""
from pathlib import Path
import os
import json
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from njord_sim.vessel import load_config


def launch(context):
    p = lambda name: LaunchConfiguration(name).perform(context)
    share = Path(get_package_share_directory('njord_sim'))
    if p('profile') not in ('fast', 'conservative'):
        raise ValueError('profile must be fast or conservative')
    for component in ('autonomy', 'controller', 'perception', 'mapping'):
        if p(component) not in ('reference', 'external'):
            raise ValueError(f'{component} must be reference or external')
    config = load_config(p('vessel_config'), share/'config/vessel.yaml')
    public_path = context.launch_configurations.get('public_parameters', '')
    public = json.loads(Path(public_path).read_text()) if public_path else {}
    params_file = p('params_file')
    if params_file and not Path(params_file).is_file():
        raise ValueError(f'params_file does not exist: {params_file}')
    common = {'use_sim_time': True}
    overrides = [params_file] if params_file else []
    nodes = []
    def node(executable, params=None):
        return Node(package='njord_sim', executable=executable, output='screen',
                    parameters=[params or {}, *overrides, public.get(executable, {}), common])
    nodes.append(node('sensor_adapter', {'seed': int(p('seed')),
                 'orientation_noise_rad': config['imu_orientation_noise_rad'],
                 'gps_xy_std_m': config['gps_horizontal_noise_m'], 'gps_z_std_m': config['gps_vertical_noise_m']}))
    localization = str(share/'config/localization.yaml')
    for name, output in [('ekf_local', '/njord/local/odometry'), ('ekf_global', '/njord/odometry')]:
        nodes.append(Node(package='robot_localization', executable='ekf_node', name=name, output='screen',
                          parameters=[localization, *overrides, common], remappings=[('odometry/filtered', output)]))
    nodes.append(Node(package='robot_localization', executable='navsat_transform_node', name='navsat',
                      output='screen', parameters=[localization, *overrides, common], remappings=[
                          ('imu', '/wamv/sensors/imu/imu/data'), ('gps/fix', '/wamv/sensors/gps/gps/fix'),
                          ('odometry/filtered', '/njord/odometry'), ('odometry/gps', '/njord/gps/odometry')]))
    if p('autonomy') == 'reference':
        if p('mapping') == 'reference':
            nodes.append(node('mapper'))
        if p('perception') == 'reference':
            nodes.append(node('perception'))
        nodes += [node('mission', {'expected_gates': int(p('expected_gates'))}), node('planner')]
        if p('controller') == 'reference':
            nodes.append(node('guidance', {
                      'max_speed': 2.0 if p('profile') == 'fast' else 1.0,
                      'thruster_separation_m': config['thruster_separation_m'], 'max_thrust': config['max_thrust_n']}))
    nodes += [node('command_guard', {'max_thrust': config['max_thrust_n']})]
    return nodes


def generate_launch_description():
    share = Path(get_package_share_directory('njord_sim'))
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value=os.environ.get('PROFILE', 'fast')),
        DeclareLaunchArgument('seed', default_value=os.environ.get('SEED', '1')),
        DeclareLaunchArgument('expected_gates', default_value='3'),
        DeclareLaunchArgument('autonomy', default_value=os.environ.get('AUTONOMY', 'reference')),
        *[DeclareLaunchArgument(name, default_value=os.environ.get(name.upper(), 'reference'))
          for name in ('controller', 'perception', 'mapping')],
        DeclareLaunchArgument('params_file', default_value=os.environ.get('ROS_PARAMS_FILE', '')),
        DeclareLaunchArgument('public_parameters', default_value=''),
        DeclareLaunchArgument('vessel_config', default_value=os.environ.get('VESSEL_CONFIG', str(share/'config/vessel.yaml'))),
        OpaqueFunction(function=launch)])
