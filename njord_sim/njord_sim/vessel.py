"""Generate the WAM-V reference model and its explicit ROS/Gazebo bridges.

The hull, hydrodynamics and thrusters come from the pinned VRX xacro. This
module applies the configured sensor settings, attaches the Njord actuator
watchdog and writes, into the run directory:

- vessel.urdf / vessel.sdf  the robot description and the spawnable Gazebo model
- bridges.yaml           one-way ros_gz_bridge topics (no unguarded thrust input)
- vessel_config.yaml     the flat sensor/thrust settings actually used
"""
from pathlib import Path
import math
import subprocess
import xml.etree.ElementTree as ET

import yaml

from .constants import (BASE_FRAME, COMMAND_TIMEOUT_S, PROCESS_LIVENESS_S, GPS_FRAME, GPS_RAW_TOPIC, GROUND_TRUTH_TOPIC,
                        GZ_MODEL_NAME, IMU_FRAME, IMU_RAW_TOPIC, LIDAR_FRAME, LIDAR_POINTS_TOPIC,
                        LIDAR_SCAN_TOPIC, CAMERAS, camera_frame, camera_topic)

# Namespace of the upstream VRX xacro. It stays on the Gazebo-internal
# thruster topics (/wamv/thrusters/...); link and joint names lose it.
VRX_NAMESPACE = 'wamv'

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


def remove_sensor_noise(sensor):
    """Remove all upstream noise/bias distributions; adapter owns IMU/GPS."""
    for parent in sensor.iter():
        for child in list(parent):
            if child.tag == 'noise':
                parent.remove(child)


def strip_link_prefix(urdf_text, prefix=VRX_NAMESPACE + '/'):
    """Drop ``prefix`` from every URDF link and joint name and each reference to one.

    The VRX xacro names everything ``wamv/<name>``. Renaming before the URDF
    is converted to SDF keeps Gazebo links, plugin references (link_name,
    joint_name, gazebo reference=...) and TF frames identical. Only exact
    link or joint names change; topics such as ``wamv/thrusters/left/pos``
    are left alone.
    """
    root = ET.fromstring(urdf_text)
    names = {element.get('name') for element in root.iter() if element.tag in ('link', 'joint')}
    renamed = {name: name.removeprefix(prefix) for name in names if name and name.startswith(prefix)}
    if len(set(renamed.values()) | (names - set(renamed))) != len(names):
        raise ValueError('removing the VRX prefix would make link or joint names collide')
    for element in root.iter():
        for key, value in element.attrib.items():
            if value in renamed:
                element.set(key, renamed[value])
        if element.text and element.text.strip() in renamed:
            element.text = renamed[element.text.strip()]
    return ET.tostring(root, encoding='unicode')


def configure_thrusters(model, limit):
    """Fail closed on upstream contract drift and set symmetric force limits."""
    plugins = [p for p in model.findall('plugin')
               if p.get('name') == 'gz::sim::systems::Thruster']
    expected = {'thrusters/left/thrust': 'left_engine_propeller_joint',
                'thrusters/right/thrust': 'right_engine_propeller_joint'}
    if len(plugins) != len(expected):
        raise ValueError('WAM-V requires exactly two upstream Thruster plugins')
    joints = {j.get('name'): j for j in model.findall('joint')}
    links = {l.get('name') for l in model.findall('link')}
    seen = set()
    for plugin in plugins:
        topic, joint = plugin.findtext('topic'), plugin.findtext('joint_name')
        if (topic in seen or expected.get(topic) != joint or joint not in joints
                or joints[joint].findtext('child') not in links
                or plugin.get('filename') != 'gz-sim-thruster-system'
                or plugin.findtext('namespace') != VRX_NAMESPACE
                or plugin.findtext('use_angvel_cmd', 'false').lower() not in ('false', '0')
                or plugin.findtext('velocity_control') not in ('true', '1')):
            raise ValueError(f'upstream WAM-V force thruster contract changed: {topic}, {joint}')
        seen.add(topic)
        set_text(plugin, 'use_angvel_cmd', 'false')
        set_text(plugin, 'max_thrust_cmd', limit)
        set_text(plugin, 'min_thrust_cmd', -limit)


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

    # Expand the upstream VRX WAM-V with our sensor mounting poses, drop the
    # VRX link prefix, then let Gazebo convert URDF to SDF so we can edit the
    # sensor elements below.
    urdf = strip_link_prefix(subprocess.check_output([
        'xacro', str(Path(get_package_share_directory('wamv_gazebo')) / 'urdf/wamv_gazebo.urdf.xacro'),
        f'namespace:={VRX_NAMESPACE}', 'locked:=false', 'thruster_config:=H',
        'yaml_component_generation:=true',
        f'component_xacro_file:={share}/config/vessels/wamv_sensors.xacro'], text=True))
    (out / 'vessel.urdf').write_text(urdf)
    raw_sdf = subprocess.check_output(['gz', 'sdf', '-p', str(out / 'vessel.urdf')], text=True)
    root = ET.fromstring(raw_sdf)
    model = root.find('model')
    model.set('name', GZ_MODEL_NAME)
    configure_thrusters(model, config['max_thrust_n'])
    bridges = []

    def bridge(gz_topic, ros_topic, ros_type, gz_type, direction='GZ_TO_ROS'):
        bridges.append(dict(gz_topic_name=gz_topic, ros_topic_name=ros_topic,
                            ros_type_name=ros_type, gz_type_name=gz_type, direction=direction))

    # Overwrite each sensor's topic, frame and settings from the configuration.
    for sensor in model.findall('.//sensor'):
        name, kind = sensor.get('name'), sensor.get('type')
        if kind == 'camera':
            # e.g. front_left_camera_sensor (wamv_sensors.xacro) -> front_left.
            camera = name.removesuffix('_camera_sensor')
            if camera not in CAMERAS:
                raise ValueError(f'unexpected WAM-V camera sensor {name}')
            image, info = camera_topic(camera, 'image_raw'), camera_topic(camera, 'camera_info')
            frame = camera_frame(camera, optical=True)
            set_text(sensor, 'topic', image)
            set_text(sensor, 'gz_frame_id', frame)
            set_text(sensor, 'camera/optical_frame_id', frame)
            set_text(sensor, 'camera/camera_info_topic', info)
            for key, field in [('width', 'camera_width'), ('height', 'camera_height')]:
                set_text(sensor, 'camera/image/' + key, config[field])
            set_text(sensor, 'update_rate', config['camera_rate'])
            set_text(sensor, 'camera/horizontal_fov', config['camera_horizontal_fov_rad'])
            set_text(sensor, 'camera/noise/stddev', config['camera_noise_stddev'])
            bridge(image, image, 'sensor_msgs/msg/Image', 'gz.msgs.Image')
            bridge(info, info, 'sensor_msgs/msg/CameraInfo', 'gz.msgs.CameraInfo')
        elif kind in ('gpu_ray', 'gpu_lidar'):
            set_text(sensor, 'topic', LIDAR_SCAN_TOPIC)
            set_text(sensor, 'gz_frame_id', LIDAR_FRAME)
            ray = 'lidar' if sensor.find('lidar') is not None else 'ray'  # SDF element name varies
            if config['lidar_range'] <= float(sensor.findtext(ray + '/range/min', '0')):
                raise ValueError('lidar_range must exceed the upstream lidar minimum range')
            set_text(sensor, ray + '/range/max', config['lidar_range'])
            set_text(sensor, ray + '/scan/horizontal/samples', config['lidar_samples'])
            set_text(sensor, ray + '/scan/vertical/samples', config['lidar_vertical_samples'])
            set_text(sensor, ray + '/noise/stddev', config['lidar_noise_stddev'])
            set_text(sensor, 'update_rate', config['lidar_rate'])
            bridge(LIDAR_SCAN_TOPIC, LIDAR_SCAN_TOPIC, 'sensor_msgs/msg/LaserScan', 'gz.msgs.LaserScan')
            # Gazebo publishes the point cloud on <scan topic>/points.
            bridge(LIDAR_SCAN_TOPIC + '/points', LIDAR_POINTS_TOPIC, 'sensor_msgs/msg/PointCloud2',
                   'gz.msgs.PointCloudPacked')
        elif kind == 'imu':
            remove_sensor_noise(sensor)
            topic = IMU_RAW_TOPIC
            set_text(sensor, 'topic', topic)
            set_text(sensor, 'gz_frame_id', IMU_FRAME)
            set_text(sensor, 'update_rate', config['imu_rate'])
            bridge(topic, topic, 'sensor_msgs/msg/Imu', 'gz.msgs.IMU')
        elif kind == 'navsat':
            remove_sensor_noise(sensor)
            topic = GPS_RAW_TOPIC
            set_text(sensor, 'topic', topic)
            set_text(sensor, 'gz_frame_id', GPS_FRAME)
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
    for key, value in dict(odom_frame='map', robot_base_frame=BASE_FRAME, dimensions=3,
                           odom_publish_frequency=30, odom_topic=GROUND_TRUTH_TOPIC).items():
        set_text(plugin, key, value)
    # The watchdog is the only path from /njord/actuator_forces to the VRX
    # thrusters; it zeroes thrust when commands stop arriving.
    watchdog = ET.SubElement(model, 'plugin', filename='libNjordActuatorWatchdog.so', name='njord::ActuatorWatchdog')
    set_text(watchdog, 'timeout_s', COMMAND_TIMEOUT_S)  # simulation time
    set_text(watchdog, 'liveness_timeout_s', PROCESS_LIVENESS_S)  # steady time
    set_text(watchdog, 'max_force_n', config['max_thrust_n'])
    bridge(GROUND_TRUTH_TOPIC, GROUND_TRUTH_TOPIC, 'nav_msgs/msg/Odometry', 'gz.msgs.Odometry')
    bridge('/clock', '/clock', 'rosgraph_msgs/msg/Clock', 'gz.msgs.Clock')
    bridge('/njord/contacts', '/njord/contacts', 'ros_gz_interfaces/msg/Contacts', 'gz.msgs.Contacts')
    # Two forces in newtons, port then starboard (constants.WAMV_THRUSTERS).
    bridge('/njord/actuator_forces', '/njord/actuator_forces', 'ros_gz_interfaces/msg/Float32Array',
           'gz.msgs.Float_V', 'ROS_TO_GZ')
    # Only physical joint states are published; they contain no world pose.
    bridge(f'/world/njord_course/model/{GZ_MODEL_NAME}/joint_state', '/joint_states',
           'sensor_msgs/msg/JointState', 'gz.msgs.Model')
    # Upstream plugins that would leak ground-truth pose or detach parts.
    for plugin in list(model.findall('plugin')):
        if 'PosePublisher' in plugin.get('name', '') or 'DetachableJoint' in plugin.get('name', ''):
            model.remove(plugin)
    ET.indent(root)
    (out / 'vessel.sdf').write_text(ET.tostring(root, encoding='unicode'))
    from .visual_geometry import export_visual_geometry
    export_visual_geometry(model, out)
    (out / 'bridges.yaml').write_text(yaml.safe_dump(bridges))
    (out / 'vessel_config.yaml').write_text(yaml.safe_dump(config, sort_keys=True))
    return urdf, config
