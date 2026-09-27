"""Independent, uncalibrated analytic Njord test hull; no upstream VRX physics.

Generates the Njord vessel model from the resolved configuration (see
configuration.resolve_configuration) and writes, into the run directory:

- vessel.sdf          spawnable Gazebo model: hull link, sensors and plugins
- vessel.urdf         robot description for robot_state_publisher (static TF)
- bridges.yaml        one-way ros_gz_bridge topics (thrust enters only via
                      /njord/actuator_forces, guarded by the physics plugin)
- vessel_config.yaml  the flat sensor/thrust settings actually used

Model, link, frame and topic names are the vessel-neutral ones from
constants.py, the same as for the WAM-V, so launch, bridges, autonomy and the
evaluator work unchanged for either vessel profile.

Physics is split between two plugins:

- libNjordPhysics.so (njord_gz_plugins/src/NjordPhysics.cc): mesh buoyancy,
  first-order thrusters driven in newtons, and constant wind loads.
- gz-sim Hydrodynamics: linear and quadratic damping and the ENU current.

Added mass is given to the physics engine through ``fluid_added_mass`` on the
link inertial. The parameter values are placeholders, not measured Njord data.
Conventions: SI units, world ENU, body frame forward-left-up.
"""

from pathlib import Path
import math
import xml.etree.ElementTree as ET
import yaml
from .configuration import sensor_settings, thruster_table
from .constants import (BASE_FRAME, COMMAND_TIMEOUT_S, PROCESS_LIVENESS_S, GPS_FRAME, GPS_RAW_TOPIC, GROUND_TRUTH_TOPIC,
                        GZ_MODEL_NAME, IMU_FRAME, IMU_RAW_TOPIC, LIDAR_FRAME, LIDAR_POINTS_TOPIC,
                        LIDAR_SCAN_TOPIC, camera_frame, camera_topic)
from .vessel import set_text


def generate(output_dir, resolved_configuration):
    """Write the Njord model files into ``output_dir``; return (urdf, settings).

    Args:
        output_dir: run directory; created if missing.
        resolved_configuration: result of configuration.resolve_configuration
            (a dict, or an object with ``to_dict()``) with the ``njord`` profile.

    Returns the URDF text and the flat settings dict from ``sensor_settings``.
    """
    resolved = resolved_configuration
    if hasattr(resolved, "to_dict"):
        resolved = resolved.to_dict()
    vessel = resolved["vessel"]
    env = resolved["scenario"]["environment"]
    config = sensor_settings(resolved)
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    # The SDF and URDF are built in parallel: the SDF is what Gazebo simulates,
    # the URDF only feeds robot_state_publisher with the same fixed frames.
    root = ET.Element("sdf", version="1.11")
    model = ET.SubElement(root, "model", name=GZ_MODEL_NAME)
    link = ET.SubElement(model, "link", name=BASE_FRAME)
    urdf = ET.Element("robot", name=GZ_MODEL_NAME)
    ul = ET.SubElement(urdf, "link", name=BASE_FRAME)

    def txt(parent, path, value):
        # SDF vectors and poses are space-separated text.
        set_text(
            parent,
            path,
            " ".join(map(str, value)) if isinstance(value, (list, tuple)) else value,
        )

    # --- Rigid-body inertia -------------------------------------------------
    # Inertia is about the centre of mass with body-parallel axes, so the
    # inertial pose is a pure translation to the COM (body frame, metres).
    com = vessel["center_of_mass_m"]
    txt(link, "inertial/mass", vessel["mass_kg"])
    txt(link, "inertial/pose", com + [0, 0, 0])
    ui = ET.SubElement(ul, "inertial")
    ET.SubElement(ui, "origin", xyz=" ".join(map(str, com)), rpy="0 0 0")
    ET.SubElement(ui, "mass", value=str(vessel["mass_kg"]))
    ET.SubElement(
        ui, "inertia", **{k: str(v) for k, v in vessel["inertia_kg_m2"].items()}
    )
    for key, value in vessel["inertia_kg_m2"].items():
        txt(link, "inertial/inertia/" + key, value)
    # Diagonal six-DOF added mass (kg for surge/sway/heave, kg m^2 for
    # roll/pitch/yaw) as SDF 1.11 fluid_added_mass entries.
    for key, value in zip(
        ("xx", "yy", "zz", "pp", "qq", "rr"), vessel["hydrodynamics"]["added_mass"]
    ):
        txt(link, "inertial/fluid_added_mass/" + key, value)

    # --- Visual and collision geometry (box or pre-scaled convex OBJ) -------
    for kind in ("visual", "collision"):
        geom = vessel["geometry"][kind]
        g = ET.SubElement(link, kind, name="hull_" + kind)
        txt(g, "pose", geom["pose"])
        txt(
            g,
            "geometry/box/size" if geom["type"] == "box" else "geometry/mesh/uri",
            geom["size_m"] if geom["type"] == "box" else geom["uri"],
        )
        ug = ET.SubElement(ul, kind)
        ET.SubElement(
            ug,
            "origin",
            xyz=" ".join(map(str, geom["pose"][:3])),
            rpy=" ".join(map(str, geom["pose"][3:])),
        )
        if geom["type"] == "box":
            ET.SubElement(
                ET.SubElement(ug, "geometry"),
                "box",
                size=" ".join(map(str, geom["size_m"])),
            )
        else:
            ET.SubElement(ET.SubElement(ug, "geometry"), "mesh", filename=geom["uri"])
    bridges = []

    def bridge(topic, ros_type, gz_type, ros_topic=None, direction="GZ_TO_ROS"):
        # One ros_gz_bridge entry; the ROS topic defaults to the Gazebo topic.
        bridges.append(
            dict(
                gz_topic_name=topic,
                ros_topic_name=ros_topic or topic,
                ros_type_name=ros_type,
                gz_type_name=gz_type,
                direction=direction,
            )
        )

    def frame(name, pose, parent=BASE_FRAME):
        # Fixed URDF joint for a sensor frame; pose is [x y z roll pitch yaw]
        # relative to ``parent`` (metres, radians).
        ET.SubElement(urdf, "link", name=name)
        j = ET.SubElement(urdf, "joint", name=name + "_joint", type="fixed")
        ET.SubElement(j, "parent", link=parent)
        ET.SubElement(j, "child", link=name)
        ET.SubElement(
            j,
            "origin",
            xyz=" ".join(map(str, pose[:3])),
            rpy=" ".join(map(str, pose[3:])),
        )

    # --- Sensors --------------------------------------------------------------
    # (pose key in vessel YAML, Gazebo sensor name, Gazebo sensor type, frame).
    # Names, frames and topics are the shared ones from constants.py.
    poses = vessel["sensors"]["poses"]
    for short, name, kind, sensor_frame in [
        ("camera", "front_left_camera_sensor", "camera", camera_frame("front_left")),
        ("camera_right", "front_right_camera_sensor", "camera", camera_frame("front_right")),
        ("lidar", "lidar_sensor", "gpu_lidar", LIDAR_FRAME),
        ("gps", "gps_sensor", "navsat", GPS_FRAME),
        ("imu", "imu_sensor", "imu", IMU_FRAME),
    ]:
        sensor = ET.SubElement(link, "sensor", name=name, type=kind)
        frame(sensor_frame, poses[short])
        txt(sensor, "pose", poses[short])
        txt(sensor, "always_on", "true")
        txt(sensor, "gz_frame_id", sensor_frame)
        # Both cameras share camera_rate; the others use <short>_rate.
        txt(
            sensor,
            "update_rate",
            config[("camera" if kind == "camera" else short) + "_rate"],
        )
        if kind == "camera":
            # ROS optical frame (z forward, x right, y down) under the
            # body-aligned camera link; images are stamped in this frame.
            camera = name.removesuffix("_camera_sensor")
            optical = camera_frame(camera, optical=True)
            frame(optical, [0, 0, 0, -math.pi / 2, 0, -math.pi / 2], sensor_frame)
            image, info = camera_topic(camera, "image_raw"), camera_topic(camera, "camera_info")
            txt(sensor, "topic", image)
            txt(sensor, "gz_frame_id", optical)
            txt(sensor, "camera/optical_frame_id", optical)
            for key, val in {
                "image/width": config["camera_width"],
                "image/height": config["camera_height"],
                "image/format": "R8G8B8",
                "horizontal_fov": config["camera_horizontal_fov_rad"],
                "clip/near": 0.1,
                "clip/far": 200,
                "camera_info_topic": info,
                "noise/stddev": config["camera_noise_stddev"],
            }.items():
                txt(sensor, "camera/" + key, val)
            txt(sensor, "camera/noise/type", "gaussian")
            bridge(image, "sensor_msgs/msg/Image", "gz.msgs.Image")
            bridge(info, "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo")
        elif short == "lidar":
            # 360 degree horizontal scan, +/-0.26 rad (about +/-15 degrees)
            # vertical fan, 0.2 m minimum range.
            txt(sensor, "topic", LIDAR_SCAN_TOPIC)
            for key, val in {
                "scan/horizontal/samples": config["lidar_samples"],
                "scan/horizontal/min_angle": -math.pi,
                "scan/horizontal/max_angle": math.pi,
                "scan/vertical/samples": config["lidar_vertical_samples"],
                "scan/vertical/min_angle": -0.26,
                "scan/vertical/max_angle": 0.26,
                "range/min": 0.2,
                "range/max": config["lidar_range"],
                "range/resolution": 0.01,
                "noise/stddev": config["lidar_noise_stddev"],
            }.items():
                txt(sensor, "lidar/" + key, val)
            txt(sensor, "lidar/noise/type", "gaussian")
            bridge(LIDAR_SCAN_TOPIC, "sensor_msgs/msg/LaserScan", "gz.msgs.LaserScan")
            # Gazebo publishes the point cloud on <scan topic>/points.
            bridge(
                LIDAR_SCAN_TOPIC + "/points",
                "sensor_msgs/msg/PointCloud2",
                "gz.msgs.PointCloudPacked",
                LIDAR_POINTS_TOPIC,
            )
        else:
            # GPS and IMU raw topics carry no SDF noise here: sensor_adapter_node
            # adds metric GPS noise and IMU orientation noise, because
            # Harmonic's NavSat noise is specified in degrees, not metres.
            topic = GPS_RAW_TOPIC if short == "gps" else IMU_RAW_TOPIC
            txt(sensor, "topic", topic)
            bridge(
                topic,
                "sensor_msgs/msg/" + ("NavSatFix" if short == "gps" else "Imu"),
                "gz.msgs." + ("NavSat" if short == "gps" else "IMU"),
            )

    def plugin(filename, name):
        return ET.SubElement(model, "plugin", filename=filename, name=name)

    # --- NjordPhysics: buoyancy, thrusters and wind ----------------------------
    p = plugin("libNjordPhysics.so", "njord::Physics")
    buoyancy = vessel["geometry"]["buoyancy"]
    from .mesh_geometry import geometry_mesh

    # Each buoyancy volume becomes a closed triangle mesh in body coordinates:
    # <vertex>x y z</vertex> and <triangle>i j k</triangle> (zero-based).
    volumes = buoyancy if isinstance(buoyancy, list) else [buoyancy]
    for geometry in volumes:
        volume = ET.SubElement(p, "buoyancy_volume")
        vertices, faces = geometry_mesh(geometry)
        for vertex in vertices:
            ET.SubElement(volume, "vertex").text = " ".join(map(str, vertex))
        for face in faces:
            ET.SubElement(volume, "triangle").text = " ".join(map(str, face))
    fields = {
        "link_name": BASE_FRAME,
        "center_of_mass": com,                    # body frame, m
        "water_density": env["water_density_kg_m3"],
        "water_level": env["water_level_m"],      # world z of the flat surface, m
        "wind_velocity": env["wind_velocity_enu"],  # constant, world ENU, m/s
        "wind_area_x": vessel["wind"]["reference_area_m2"][0],  # frontal, m^2
        "wind_area_y": vessel["wind"]["reference_area_m2"][1],  # lateral, m^2
        "wind_length": vessel["wind"]["reference_length_m"],    # yaw lever, m
        # Command freshness in simulation time, and process liveness in
        # steady time; a stale or dead command targets zero thrust.
        "timeout_s": COMMAND_TIMEOUT_S,
        "liveness_timeout_s": PROCESS_LIVENESS_S,
    }
    for k, v in fields.items():
        txt(p, k, v)
    # Thrusters in vessel-file order: element i receives data[i] of
    # /njord/actuator_forces. Forces in newtons, position and unit axis
    # (from yaw_deg) in the body frame, response_time is the first-order lag.
    for t in thruster_table(vessel):
        child = ET.SubElement(p, "thruster", name=t["name"])
        for key, value in {
            "position": t["position_m"],
            "axis": t["axis"],
            "forward": t["forward_limit_n"],
            "reverse": t["reverse_limit_n"],
            "response_time": t["response_time_s"],
        }.items():
            txt(child, key, value)
    # Wind coefficient table indexed by the direction of the relative air flow
    # in the body frame (degrees, see physics_core.wind_coefficients).
    for row in vessel["wind"]["coefficients"]:
        child = ET.SubElement(p, "wind_coefficient")
        for key in ("cx", "cy", "cn"):
            txt(child, key, row[key])
        txt(child, "angle", row["angle_deg"])

    # --- gz-sim Hydrodynamics: damping and current -----------------------------
    hydro = plugin("gz-sim-hydrodynamics-system", "gz::sim::systems::Hydrodynamics")
    txt(hydro, "link_name", BASE_FRAME)
    # Added mass is already on the link inertial (fluid_added_mass above);
    # this plugin's added-mass term stays off so it is not applied twice.
    txt(hydro, "disable_added_mass", "true")
    txt(hydro, "disable_coriolis", "false")
    txt(hydro, "default_current", env["current_velocity_enu"])  # world ENU, m/s
    # Parameter names are <force axis><velocity>: xU is the linear surge
    # derivative X_u, xUabsU the quadratic X_|u|u, ..., nR / nRabsR for yaw.
    # Gazebo uses hydrodynamic derivatives, which are negative for damping,
    # so the nonnegative YAML magnitudes are negated here.
    for i, (f, a) in enumerate(zip("xyzkmn", "UVWPQR")):
        txt(hydro, f + a, -vessel["hydrodynamics"]["linear_damping"][i])
        txt(hydro, f + a + "abs" + a, -vessel["hydrodynamics"]["quadratic_damping"][i])

    # --- Ground truth and bridges ------------------------------------------------
    # Ground-truth odometry in the ENU map frame is for the evaluator only;
    # autonomy must localize from sensors.
    odom = plugin(
        "gz-sim-odometry-publisher-system", "gz::sim::systems::OdometryPublisher"
    )
    for k, v in dict(
        odom_frame="map",
        robot_base_frame=BASE_FRAME,
        dimensions=3,
        odom_publish_frequency=30,
        odom_topic=GROUND_TRUTH_TOPIC,
    ).items():
        txt(odom, k, v)
    bridge(GROUND_TRUTH_TOPIC, "nav_msgs/msg/Odometry", "gz.msgs.Odometry")
    bridge("/clock", "rosgraph_msgs/msg/Clock", "gz.msgs.Clock")
    bridge("/njord/actuator_wrench", "geometry_msgs/msg/WrenchStamped", "gz.msgs.Wrench")
    bridge("/njord/contacts", "ros_gz_interfaces/msg/Contacts", "gz.msgs.Contacts")
    # The only ROS -> Gazebo path: one thrust force in newtons per thruster,
    # in vessel-file order, never a velocity command.
    bridge(
        "/njord/actuator_forces",
        "ros_gz_interfaces/msg/Float32Array",
        "gz.msgs.Float_V",
        direction="ROS_TO_GZ",
    )
    for tree in (root, urdf):
        ET.indent(tree)
    urdf_text = ET.tostring(urdf, encoding="unicode")
    (out / "vessel.urdf").write_text(urdf_text)
    (out / "vessel.sdf").write_text(ET.tostring(root, encoding="unicode"))
    from .visual_geometry import export_visual_geometry
    export_visual_geometry(model, out)
    (out / "bridges.yaml").write_text(yaml.safe_dump(bridges))
    (out / "vessel_config.yaml").write_text(yaml.safe_dump(config))
    return urdf_text, config
