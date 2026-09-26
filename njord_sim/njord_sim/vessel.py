"""Generate the WAM-V reference model and its explicit ROS/Gazebo bridges.

The hull, hydrodynamics and thrusters come from the pinned VRX xacro. This
module applies the configured sensor settings, attaches the Njord actuator
watchdog and writes, into the run directory:

- wamv.urdf / wamv.sdf   the robot description and the spawnable Gazebo model
- bridges.yaml           one-way ros_gz_bridge topics (no unguarded thrust input)
- vessel_config.yaml     the flat sensor/thrust settings actually used
"""
from pathlib import Path
import math
import subprocess
import xml.etree.ElementTree as ET

import yaml

from .constants import COMMAND_TIMEOUT_S, WAMV_THRUSTER_SEPARATION_M

# Integer-valued settings; every other setting is stored as float.
COUNT_KEYS = ('camera_width', 'camera_height', 'lidar_samples', 'lidar_vertical_samples')


def _read_settings(path):
    """Read flat WAM-V settings from a flat file or a versioned vessel file."""
    data = yaml.safe_load(Path(path).read_text())
    if isinstance(data, dict) and data.get('profile') == 'wamv_reference':
        return dict(data['settings'])
    return data


def load_config(config_file=None, defaults_file=None):
    """Merge a partial flat override onto the WAM-V defaults and validate it.

    ``defaults_file`` is vessels/wamv.yaml (or any flat settings file); passing
    it explicitly allows validation without a ROS installation. Unknown keys
    fail, so a misspelled sensor setting never silently does nothing.
    """
    if defaults_file is None:
        from .configuration import wamv_defaults_file
        defaults_file = wamv_defaults_file()
    config = _read_settings(defaults_file)
    if config_file is not None:
        override = yaml.safe_load(Path(config_file).read_text())
        if not isinstance(override, dict):
            raise ValueError('vessel configuration must be a YAML mapping')
        unknown = set(override) - set(config)
        if unknown:
            raise ValueError(f'unknown vessel configuration keys: {sorted(unknown, key=str)}')
        config.update(override)
    for key, value in config.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'{key} must be a finite number')
        if key in COUNT_KEYS and not isinstance(value, int):
            raise ValueError(f'{key} must be an integer')
        allow_zero = 'noise' in key
        if value < 0 or (value == 0 and not allow_zero):
            raise ValueError(f'{key} must be {"nonnegative" if allow_zero else "positive"}')
        if key not in COUNT_KEYS:
            config[key] = float(value)
    if not 0 < config['camera_horizontal_fov_rad'] < math.pi:
        raise ValueError('camera_horizontal_fov_rad must be between 0 and pi radians')
    return config


def set_text(parent, path, value):
    """Set the text of ``parent/path``, creating missing elements on the way."""
    for part in path.split('/'):
        child = parent.find(part)
        if child is None:
            child = ET.SubElement(parent, part)
        parent = child
    parent.text = str(value)


def generate(output_dir, config_file=None, resolved_config=None):
    """Write the WAM-V model files into ``output_dir``; return (urdf, settings).

    ``resolved_config`` is the flat output of configuration.sensor_settings.
    Without it, ``config_file`` (a partial override, or None for the defaults)
    is merged onto vessels/wamv.yaml.
    """
    from ament_index_python.packages import get_package_share_directory
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    share = Path(get_package_share_directory('njord_sim'))
    if resolved_config is not None:
        config = resolved_config
    else:
        config = load_config(config_file, share / 'config/vessels/wamv.yaml')
        config['thruster_separation_m'] = WAMV_THRUSTER_SEPARATION_M

    # Expand the upstream VRX WAM-V with our sensor mounting poses, then let
    # Gazebo convert URDF to SDF so we can edit the sensor elements below.
    urdf = subprocess.check_output([
        'xacro', str(Path(get_package_share_directory('wamv_gazebo')) / 'urdf/wamv_gazebo.urdf.xacro'),
        'namespace:=wamv', 'locked:=false', 'thruster_config:=H',
        'yaml_component_generation:=true',
        f'component_xacro_file:={share}/config/vessels/wamv_sensors.xacro'], text=True)
    (out / 'wamv.urdf').write_text(urdf)
    raw_sdf = subprocess.check_output(['gz', 'sdf', '-p', str(out / 'wamv.urdf')], text=True)
    root = ET.fromstring(raw_sdf)
    model = root.find('model')
    model.set('name', 'wamv')
    bridges = []

    def bridge(gz_topic, ros_topic, ros_type, gz_type, direction='GZ_TO_ROS'):
        bridges.append(dict(gz_topic_name=gz_topic, ros_topic_name=ros_topic,
                            ros_type_name=ros_type, gz_type_name=gz_type, direction=direction))

    # Overwrite each sensor's topic, frame and settings from the configuration.
    for sensor in model.findall('.//sensor'):
        name, kind = sensor.get('name'), sensor.get('type')
        if kind == 'camera':
            prefix = f'/wamv/sensors/cameras/{name}'
            frame = f'wamv/{name.removesuffix("_sensor")}_link_optical'
            set_text(sensor, 'topic', prefix + '/image_raw')
            set_text(sensor, 'gz_frame_id', frame)
            set_text(sensor, 'camera/optical_frame_id', frame)
            set_text(sensor, 'camera/camera_info_topic', prefix + '/camera_info')
            for key, field in [('width', 'camera_width'), ('height', 'camera_height')]:
                set_text(sensor, 'camera/image/' + key, config[field])
            set_text(sensor, 'update_rate', config['camera_rate'])
            set_text(sensor, 'camera/horizontal_fov', config['camera_horizontal_fov_rad'])
            set_text(sensor, 'camera/noise/stddev', config['camera_noise_stddev'])
            bridge(prefix + '/image_raw', prefix + '/image_raw', 'sensor_msgs/msg/Image', 'gz.msgs.Image')
            bridge(prefix + '/camera_info', prefix + '/camera_info', 'sensor_msgs/msg/CameraInfo', 'gz.msgs.CameraInfo')
        elif kind in ('gpu_ray', 'gpu_lidar'):
            prefix = f'/wamv/sensors/lidars/{name}'
            set_text(sensor, 'topic', prefix + '/scan')
            set_text(sensor, 'gz_frame_id', 'wamv/lidar_wamv_link')
            ray = 'lidar' if sensor.find('lidar') is not None else 'ray'  # SDF element name varies
            if config['lidar_range'] <= float(sensor.findtext(ray + '/range/min', '0')):
                raise ValueError('lidar_range must exceed the upstream lidar minimum range')
            set_text(sensor, ray + '/range/max', config['lidar_range'])
            set_text(sensor, ray + '/scan/horizontal/samples', config['lidar_samples'])
            set_text(sensor, ray + '/scan/vertical/samples', config['lidar_vertical_samples'])
            set_text(sensor, ray + '/noise/stddev', config['lidar_noise_stddev'])
            set_text(sensor, 'update_rate', config['lidar_rate'])
            bridge(prefix + '/scan', prefix + '/scan', 'sensor_msgs/msg/LaserScan', 'gz.msgs.LaserScan')
            bridge(prefix + '/scan/points', prefix + '/points', 'sensor_msgs/msg/PointCloud2', 'gz.msgs.PointCloudPacked')
        elif kind == 'imu':
            topic = '/wamv/sensors/imu/imu/data_raw'
            set_text(sensor, 'topic', topic)
            set_text(sensor, 'gz_frame_id', 'wamv/imu_wamv_link')
            set_text(sensor, 'update_rate', config['imu_rate'])
            bridge(topic, topic, 'sensor_msgs/msg/Imu', 'gz.msgs.IMU')
        elif kind == 'navsat':
            topic = '/wamv/sensors/gps/gps/fix_raw'
            set_text(sensor, 'topic', topic)
            set_text(sensor, 'gz_frame_id', 'wamv/gps_wamv_link')
            set_text(sensor, 'update_rate', config['gps_rate'])
            # Harmonic applies horizontal NavSat noise in degrees, not metres,
            # so it is disabled here; sensor_adapter_node adds metric noise.
            for axis, std in [('horizontal', 0.0), ('vertical', 0.0)]:
                parent = ET.SubElement(sensor, 'navsat') if sensor.find('navsat') is None else sensor.find('navsat')
                set_text(parent, f'position_sensing/{axis}/noise/stddev', std)
                parent.find(f'position_sensing/{axis}/noise').set('type', 'gaussian')
            bridge(topic, topic, 'sensor_msgs/msg/NavSatFix', 'gz.msgs.NavSat')

    # Ground truth for the evaluator only; no pose broadcaster feeds the
    # navigation TF tree.
    plugin = ET.SubElement(model, 'plugin', filename='gz-sim-odometry-publisher-system',
                           name='gz::sim::systems::OdometryPublisher')
    for key, value in dict(odom_frame='map', robot_base_frame='wamv/base_link', dimensions=3,
                           odom_publish_frequency=30, odom_topic='/wamv/ground_truth/odometry').items():
        set_text(plugin, key, value)
    # The watchdog is the only path from /njord/actuator_forces to the VRX
    # thrusters; it zeroes thrust when commands stop arriving.
    watchdog = ET.SubElement(model, 'plugin', filename='libNjordActuatorWatchdog.so', name='njord::ActuatorWatchdog')
    set_text(watchdog, 'timeout_s', COMMAND_TIMEOUT_S)
    set_text(watchdog, 'max_force_n', config['max_thrust_n'])
    bridge('/wamv/ground_truth/odometry', '/wamv/ground_truth/odometry', 'nav_msgs/msg/Odometry', 'gz.msgs.Odometry')
    bridge('/clock', '/clock', 'rosgraph_msgs/msg/Clock', 'gz.msgs.Clock')
    bridge('/njord/contacts', '/njord/contacts', 'ros_gz_interfaces/msg/Contacts', 'gz.msgs.Contacts')
    bridge('/njord/actuator_forces', '/njord/actuator_forces', 'geometry_msgs/msg/Twist', 'gz.msgs.Twist', 'ROS_TO_GZ')
    # Only physical joint states are published; they contain no world pose.
    bridge('/world/njord_course/model/wamv/joint_state', '/wamv/joint_states', 'sensor_msgs/msg/JointState', 'gz.msgs.Model')
    # Upstream plugins that would leak ground-truth pose or detach parts.
    for plugin in list(model.findall('plugin')):
        if 'PosePublisher' in plugin.get('name', '') or 'DetachableJoint' in plugin.get('name', ''):
            model.remove(plugin)
    ET.indent(root)
    (out / 'wamv.sdf').write_text(ET.tostring(root, encoding='unicode'))
    (out / 'bridges.yaml').write_text(yaml.safe_dump(bridges))
    (out / 'vessel_config.yaml').write_text(yaml.safe_dump(config, sort_keys=True))
    return urdf, config
