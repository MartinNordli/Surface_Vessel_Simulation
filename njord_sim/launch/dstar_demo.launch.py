"""Sensor-based Njord reference autonomy against the simulator service."""
from pathlib import Path
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch(context):
    p = lambda name: LaunchConfiguration(name).perform(context)
    share = Path(get_package_share_directory('njord_sim'))
    if p('profile') not in ('fast', 'conservative'):
        raise ValueError('profile must be fast or conservative')
    config = yaml.safe_load(Path(p('vessel_config')).read_text())
    common = {'use_sim_time': True}
    nodes = []
    def node(executable, params=None):
        return Node(package='njord_sim', executable=executable, output='screen',
                    parameters=[common, params or {}])
    nodes.append(node('sensor_adapter', {'seed': int(p('seed')),
                 'orientation_noise_rad': config['imu_orientation_noise_rad'],
                 'gps_xy_std_m': config['gps_horizontal_noise_m'], 'gps_z_std_m': config['gps_vertical_noise_m']}))
    localization = str(share/'config/localization.yaml')
    for name, output in [('ekf_local', '/njord/local/odometry'), ('ekf_global', '/njord/odometry')]:
        nodes.append(Node(package='robot_localization', executable='ekf_node', name=name, output='screen',
                          parameters=[localization, common], remappings=[('odometry/filtered', output)]))
    nodes.append(Node(package='robot_localization', executable='navsat_transform_node', name='navsat',
                      output='screen', parameters=[localization, common], remappings=[
                          ('imu', '/wamv/sensors/imu/imu/data'), ('gps/fix', '/wamv/sensors/gps/gps/fix'),
                          ('odometry/filtered', '/njord/odometry'), ('odometry/gps', '/njord/gps/odometry')]))
    if p('autonomy') == 'reference':
        nodes += [node('mapper'), node('perception'), node('mission', {'expected_gates': int(p('expected_gates'))}),
                  node('planner'), node('guidance', {
                      'max_speed': 2.0 if p('profile') == 'fast' else 1.0,
                      'thruster_separation_m': config['thruster_separation_m'], 'max_thrust': config['max_thrust_n']})]
    elif p('autonomy') != 'external':
        raise ValueError('autonomy must be reference or external')
    nodes += [node('command_guard', {'max_thrust': config['max_thrust_n']})]
    return nodes


def generate_launch_description():
    share = Path(get_package_share_directory('njord_sim'))
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value=os.environ.get('PROFILE', 'fast')),
        DeclareLaunchArgument('seed', default_value=os.environ.get('SEED', '1')),
        DeclareLaunchArgument('expected_gates', default_value='3'),
        DeclareLaunchArgument('autonomy', default_value='reference'),
        DeclareLaunchArgument('vessel_config', default_value=str(share/'config/vessel.yaml')),
        OpaqueFunction(function=launch)])
