"""Independent, uncalibrated analytic Njord test hull; no upstream VRX physics."""

from pathlib import Path
import math
import xml.etree.ElementTree as ET
import yaml
from .vessel import set_text


def generate(output_dir, resolved_configuration):
    resolved = resolved_configuration
    if hasattr(resolved, "to_dict"):
        resolved = resolved.to_dict()
    vessel = resolved["vessel"]
    env = resolved["scenario"]["environment"]
    config = dict(vessel["sensors"]["settings"])
    config["max_thrust_n"] = min(
        min(t["forward_limit_n"], t["reverse_limit_n"]) for t in vessel["thrusters"]
    )
    config["thruster_separation_m"] = abs(
        vessel["thrusters"][0]["position_m"][1]
        - vessel["thrusters"][1]["position_m"][1]
    )
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    root = ET.Element("sdf", version="1.11")
    model = ET.SubElement(root, "model", name="wamv")
    link = ET.SubElement(model, "link", name="wamv/base_link")
    urdf = ET.Element("robot", name="wamv")
    ul = ET.SubElement(urdf, "link", name="wamv/base_link")

    def txt(parent, path, value):
        set_text(
            parent,
            path,
            " ".join(map(str, value)) if isinstance(value, (list, tuple)) else value,
        )

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
    for key, value in zip(
        ("xx", "yy", "zz", "pp", "qq", "rr"), vessel["hydrodynamics"]["added_mass"]
    ):
        txt(link, "inertial/fluid_added_mass/" + key, value)
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
        bridges.append(
            dict(
                gz_topic_name=topic,
                ros_topic_name=ros_topic or topic,
                ros_type_name=ros_type,
                gz_type_name=gz_type,
                direction=direction,
            )
        )

    def frame(name, pose, parent="wamv/base_link"):
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

    poses = vessel["sensors"]["poses"]
    for short, name, kind in [
        ("camera", "front_left_camera_sensor", "camera"),
        ("camera_right", "front_right_camera_sensor", "camera"),
        ("lidar", "lidar_wamv_sensor", "gpu_lidar"),
        ("gps", "gps_wamv_sensor", "navsat"),
        ("imu", "imu_wamv_sensor", "imu"),
    ]:
        sensor = ET.SubElement(link, "sensor", name=name, type=kind)
        sensor_frame = "wamv/" + (
            name.removesuffix("_sensor") + "_link"
            if kind == "camera"
            else short + "_wamv_link"
        )
        frame(sensor_frame, poses[short])
        txt(sensor, "pose", poses[short])
        txt(sensor, "always_on", "true")
        txt(sensor, "gz_frame_id", sensor_frame)
        txt(
            sensor,
            "update_rate",
            config[("camera" if kind == "camera" else short) + "_rate"],
        )
        if kind == "camera":
            optical = sensor_frame + "_optical"
            frame(optical, [0, 0, 0, -math.pi / 2, 0, -math.pi / 2], sensor_frame)
            prefix = "/wamv/sensors/cameras/" + name
            txt(sensor, "topic", prefix + "/image_raw")
            txt(sensor, "gz_frame_id", optical)
            txt(sensor, "camera/optical_frame_id", optical)
            for key, val in {
                "image/width": config["camera_width"],
                "image/height": config["camera_height"],
                "image/format": "R8G8B8",
                "horizontal_fov": config["camera_horizontal_fov_rad"],
                "clip/near": 0.1,
                "clip/far": 200,
                "camera_info_topic": prefix + "/camera_info",
                "noise/stddev": config["camera_noise_stddev"],
            }.items():
                txt(sensor, "camera/" + key, val)
            txt(sensor, "camera/noise/type", "gaussian")
            bridge(prefix + "/image_raw", "sensor_msgs/msg/Image", "gz.msgs.Image")
            bridge(
                prefix + "/camera_info",
                "sensor_msgs/msg/CameraInfo",
                "gz.msgs.CameraInfo",
            )
        elif short == "lidar":
            prefix = "/wamv/sensors/lidars/" + name
            txt(sensor, "topic", prefix + "/scan")
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
            bridge(prefix + "/scan", "sensor_msgs/msg/LaserScan", "gz.msgs.LaserScan")
            bridge(
                prefix + "/scan/points",
                "sensor_msgs/msg/PointCloud2",
                "gz.msgs.PointCloudPacked",
                prefix + "/points",
            )
        else:
            topic = (
                "/wamv/sensors/"
                + short
                + "/"
                + short
                + ("/fix_raw" if short == "gps" else "/data_raw")
            )
            txt(sensor, "topic", topic)
            bridge(
                topic,
                "sensor_msgs/msg/" + ("NavSatFix" if short == "gps" else "Imu"),
                "gz.msgs." + ("NavSat" if short == "gps" else "IMU"),
            )

    def plugin(filename, name):
        return ET.SubElement(model, "plugin", filename=filename, name=name)

    p = plugin("libNjordPhysics.so", "njord::Physics")
    buoyancy = vessel["geometry"]["buoyancy"]
    from .mesh_geometry import geometry_mesh

    volumes = buoyancy if isinstance(buoyancy, list) else [buoyancy]
    for geometry in volumes:
        volume = ET.SubElement(p, "buoyancy_volume")
        vertices, faces = geometry_mesh(geometry)
        for vertex in vertices:
            ET.SubElement(volume, "vertex").text = " ".join(map(str, vertex))
        for face in faces:
            ET.SubElement(volume, "triangle").text = " ".join(map(str, face))
    fields = {
        "link_name": "wamv/base_link",
        "center_of_mass": com,
        "water_density": env["water_density_kg_m3"],
        "water_level": env["water_level_m"],
        "wind_velocity": env["wind_velocity_enu"],
        "wind_area_x": vessel["wind"]["reference_area_m2"][0],
        "wind_area_y": vessel["wind"]["reference_area_m2"][1],
        "wind_length": vessel["wind"]["reference_length_m"],
        "timeout_s": 0.5,
    }
    for k, v in fields.items():
        txt(p, k, v)
    for t in vessel["thrusters"]:
        child = ET.SubElement(p, "thruster", name=t["name"])
        for key, value in {
            "position": t["position_m"],
            "axis": t["axis"],
            "forward": t["forward_limit_n"],
            "reverse": t["reverse_limit_n"],
            "response_time": t["response_time_s"],
        }.items():
            txt(child, key, value)
    for row in vessel["wind"]["coefficients"]:
        child = ET.SubElement(p, "wind_coefficient")
        for key in ("cx", "cy", "cn"):
            txt(child, key, row[key])
        txt(child, "angle", row["angle_deg"])
    hydro = plugin("gz-sim-hydrodynamics-system", "gz::sim::systems::Hydrodynamics")
    txt(hydro, "link_name", "wamv/base_link")
    txt(hydro, "disable_added_mass", "true")
    txt(hydro, "disable_coriolis", "false")
    txt(hydro, "default_current", env["current_velocity_enu"])
    for i, (f, a) in enumerate(zip("xyzkmn", "UVWPQR")):
        txt(hydro, f + a, -vessel["hydrodynamics"]["linear_damping"][i])
        txt(hydro, f + a + "abs" + a, -vessel["hydrodynamics"]["quadratic_damping"][i])
    odom = plugin(
        "gz-sim-odometry-publisher-system", "gz::sim::systems::OdometryPublisher"
    )
    for k, v in dict(
        odom_frame="map",
        robot_base_frame="wamv/base_link",
        dimensions=3,
        odom_publish_frequency=30,
        odom_topic="/wamv/ground_truth/odometry",
    ).items():
        txt(odom, k, v)
    bridge("/wamv/ground_truth/odometry", "nav_msgs/msg/Odometry", "gz.msgs.Odometry")
    bridge("/clock", "rosgraph_msgs/msg/Clock", "gz.msgs.Clock")
    bridge("/njord/contacts", "ros_gz_interfaces/msg/Contacts", "gz.msgs.Contacts")
    bridge(
        "/njord/actuator_forces",
        "geometry_msgs/msg/Twist",
        "gz.msgs.Twist",
        direction="ROS_TO_GZ",
    )
    for tree in (root, urdf):
        ET.indent(tree)
    urdf_text = ET.tostring(urdf, encoding="unicode")
    (out / "wamv.urdf").write_text(urdf_text)
    (out / "wamv.sdf").write_text(ET.tostring(root, encoding="unicode"))
    (out / "bridges.yaml").write_text(yaml.safe_dump(bridges))
    (out / "vessel_config.yaml").write_text(yaml.safe_dump(config))
    return urdf_text, config
