"""Generate an offline VRX course and the exact evaluator configuration.

No vessel is embedded: launch spawns the pinned VRX WAM-V at the resolved
start pose. Markers are fixed vertical cylinders (a moored approximation),
with matching visual/collision/scoring radii. Dynamics remain upstream VRX.
"""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from .scenario_core import load_scenario, obstacles, scenario_digest
from .run_manifest import atomic_text


def element(parent, tag, text=None, **attrs):
    node = ET.SubElement(parent, tag, attrs)
    if text is not None:
        node.text = str(text)
    return node


def world_xml(scenario, vessel_profile='wamv_reference'):
    sdf = ET.Element("sdf", version="1.9")
    world = element(sdf, "world", name="njord_course")
    if vessel_profile == 'njord':
        # Match the hydrostatics adapter's gravity constant. SDF otherwise
        # defaults to 9.8, producing a systematic displacement error.
        element(world, 'gravity', '0 0 -9.81')
    step = scenario['environment'].get('physics_step_s', 0.004)
    physics = element(world, "physics", name=f"{step * 1000:g}ms", type="dart")
    element(physics, "max_step_size", step)
    element(physics, "real_time_factor", 1.0)
    for library, name in [("physics", "Physics"), ("user-commands", "UserCommands"),
                          ("scene-broadcaster", "SceneBroadcaster"), ("sensors", "Sensors"),
                          ("imu", "Imu"), ("navsat", "NavSat"), ("contact", "Contact")]:
        plugin = element(world, "plugin", filename=f"gz-sim-{library}-system",
                         name=f"gz::sim::systems::{name}")
        if name == "Sensors":
            element(plugin, "render_engine", "ogre2")
    scene = element(world, "scene")
    element(scene, "ambient", "0.6 0.6 0.6 1")
    element(scene, "background", "0.7 0.8 0.9 1")
    element(scene, "sky")
    light = element(world, "light", type="directional", name="sun")
    element(light, "pose", "0 0 10 0 0 0")
    element(light, "diffuse", "0.8 0.8 0.8 1")
    element(light, "specular", "0.2 0.2 0.2 1")
    element(light, "direction", "-0.5 0.1 -0.9")
    element(light, "cast_shadows", "true")
    spherical = element(world, "spherical_coordinates")
    for key, value in {"surface_model": "EARTH_WGS84", "world_frame_orientation": "ENU",
                       "latitude_deg": 63.4305, "longitude_deg": 10.3951,
                       "elevation": 0, "heading_deg": 0}.items():
        element(spherical, key, value)
    ocean = element(world, "include")
    element(ocean, "uri", "coast_waves")
    element(ocean, "name", "coast_waves")
    water_level = scenario['environment'].get('water_level_m', 0.0)
    element(ocean, "pose", f"0 0 {water_level} 0 0 0")
    colors = {"red": "1 0.02 0.02 1", "green": "0.02 1 0.02 1", "black": "0.05 0.05 0.05 1"}
    for obstacle in obstacles(scenario):
        model = element(world, "model", name=obstacle["name"])
        element(model, "static", "true")
        x, y = obstacle["position"]
        element(model, "pose", f"{x:.9f} {y:.9f} {water_level + 0.5} 0 0 0")
        link = element(model, "link", name="link")
        for tag in ("collision", "visual"):
            shape = element(link, tag, name=tag)
            cylinder = element(element(shape, "geometry"), "cylinder")
            element(cylinder, "radius", obstacle["radius_m"])
            element(cylinder, "length", 2.0)
            if tag == "visual":
                material = element(shape, "material")
                color = colors[obstacle.get("color", "black")]
                element(material, "ambient", color)
                element(material, "diffuse", color)
        sensor = element(link, "sensor", name="contact", type="contact")
        element(sensor, "always_on", "true")
        element(sensor, "update_rate", 50)
        contact = element(sensor, "contact")
        element(contact, "collision", "collision")
        element(contact, "topic", "/njord/raw_contacts/" + obstacle["name"])
    monitor = element(world, "plugin", filename="libNjordContactMonitor.so", name="njord::ContactMonitor")
    for obstacle in obstacles(scenario):
        element(monitor, "marker", obstacle["name"])
    env = scenario["environment"]
    if vessel_profile == 'njord':
        # Flat-water Njord forces live exclusively on its model plugin. The
        # upstream wind / wave plugins must not apply a second vessel wrench.
        ET.indent(sdf)
        return ET.tostring(sdf, encoding="unicode", xml_declaration=True)
    wind = element(world, "plugin", filename="libUSVWind.so", name="vrx::USVWind")
    obj = element(wind, "wind_obj")
    element(obj, "name", "wamv")
    element(obj, "link_name", "wamv/base_link")
    element(obj, "coeff_vector", "0.5 0.5 0.33")
    for key, value in {"wind_direction": env["wind_direction_deg"],
                       "wind_mean_velocity": env["wind_speed_mps"],
                       "var_wind_gain_constants": env["wind_variance_gain"],
                       "var_wind_time_constants": 2, "random_seed": scenario["seed"],
                       "update_rate": 10,
                       "topic_wind_speed": "/vrx/debug/wind/speed",
                       "topic_wind_direction": "/vrx/debug/wind/direction"}.items():
        element(wind, key, value)
    publisher = element(world, "plugin", filename="libPublisherPlugin.so", name="vrx::PublisherPlugin")
    message = element(publisher, "message", type="gz.msgs.Param",
                      topic="/vrx/wavefield/parameters", every="0.1")
    message.text = "\n" + "\n".join(
        f'params {{ key: "{key}" value {{ type: DOUBLE double_value: {value} }} }}'
        for key, value in {"direction": env["wave_direction_rad"], "gain": env["wave_gain"],
                           "period": env["wave_period_s"], "steepness": env["wave_steepness"]}.items()) + "\n"
    ET.indent(sdf)
    return ET.tostring(sdf, encoding="unicode", xml_declaration=True)


def generate(scenario_file, output_dir, seed=None, environment=None):
    scenario = load_scenario(scenario_file, seed, environment)
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    (output / "njord_course.sdf").write_text(world_xml(scenario) + "\n")
    atomic_text(output / "resolved_scenario.json", json.dumps(scenario, indent=2, allow_nan=False) + "\n")
    (output / "scenario.sha256").write_text(scenario_digest(scenario) + "\n")
    return scenario


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--seed", type=int)
    parser.add_argument("--environment", choices=["calm", "moderate"])
    args = parser.parse_args()
    generate(args.scenario, args.output_dir, args.seed, args.environment)


if __name__ == "__main__":
    main()
