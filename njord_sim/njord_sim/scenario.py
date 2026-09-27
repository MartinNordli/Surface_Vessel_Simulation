"""Generate an offline VRX course and the exact evaluator configuration.

``world_xml`` builds the Gazebo world (njord_course.sdf) from a resolved
scenario; simulation.launch.py calls it with the selected vessel profile.
The command-line ``generate`` writes the world, resolved_scenario.json and its
digest for the WAM-V reference profile only.

No vessel is embedded: launch spawns the generated vessel model at the
resolved start pose. Markers are fixed vertical cylinders (a moored
approximation), with matching visual/collision/scoring radii. With the WAM-V
reference profile, wind and waves come from the upstream VRX plugins; with the
Njord profile the world is flat water and the environment forces come only
from the plugins on the generated Njord model (see njord_model.py).

World frame: ENU, origin at constants.WORLD_ORIGIN_WGS84. Marker positions
are scenario (evaluation) data and are never published to autonomy.
"""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from .constants import BASE_FRAME, GZ_MODEL_NAME, WORLD_ORIGIN_WGS84
from .scenario_core import load_scenario, obstacles, scenario_digest
from .run_manifest import atomic_text


def element(parent, tag, text=None, **attrs):
    """Append ``<tag attrs>text</tag>`` to ``parent`` and return it."""
    node = ET.SubElement(parent, tag, attrs)
    if text is not None:
        node.text = str(text)
    return node


def world_xml(scenario, vessel_profile='wamv_reference', real_time_factor=1.0):
    """Return the SDF world text for a resolved ``scenario``.

    ``vessel_profile`` is ``'wamv_reference'`` (VRX wind and wave plugins) or
    ``'njord'`` (flat water, no upstream environment forces).
    ``real_time_factor`` is the target of simulated seconds per wall second.
    """
    sdf = ET.Element("sdf", version="1.9")
    world = element(sdf, "world", name="njord_course")
    if vessel_profile == 'njord':
        # Match the hydrostatics adapter's gravity constant. SDF otherwise
        # defaults to 9.8, producing a systematic displacement error.
        element(world, 'gravity', '0 0 -9.81')
    # Fixed physics step in seconds; real_time_factor is a target, not a
    # guarantee (slow hosts run slower, simulation time stays exact).
    step = scenario['environment'].get('physics_step_s', 0.004)
    physics = element(world, "physics", name=f"{step * 1000:g}ms", type="dart")
    element(physics, "max_step_size", step)
    element(physics, "real_time_factor", real_time_factor)
    # World systems: physics, entity spawning, GUI/scene state, rendered and
    # non-rendered sensors, and contact detection for the markers.
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
    # Geodetic datum for the NavSat sensor; must match navsat_transform.
    spherical = element(world, "spherical_coordinates")
    for key, value in {"surface_model": "EARTH_WGS84", "world_frame_orientation": "ENU",
                       "latitude_deg": WORLD_ORIGIN_WGS84[0], "longitude_deg": WORLD_ORIGIN_WGS84[1],
                       "elevation": WORLD_ORIGIN_WGS84[2], "heading_deg": 0}.items():
        element(spherical, key, value)
    # VRX ocean surface model, placed at the configured water level.
    ocean = element(world, "include")
    element(ocean, "uri", "coast_waves")
    element(ocean, "name", "coast_waves")
    water_level = scenario['environment'].get('water_level_m', 0.0)
    element(ocean, "pose", f"0 0 {water_level} 0 0 0")
    colors = {"red": "1 0.02 0.02 1", "green": "0.02 1 0.02 1", "black": "0.05 0.05 0.05 1"}
    # Every gate marker and extra obstacle is a static 2 m tall cylinder
    # centred 0.5 m above the water level, with a contact sensor whose
    # collisions are reported through the ContactMonitor below.
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
    # Publishes /njord/contacts as a heartbeat (also when empty) once every
    # listed marker has a live contact sensor; see ContactMonitor.cc.
    monitor = element(world, "plugin", filename="libNjordContactMonitor.so", name="njord::ContactMonitor")
    for obstacle in obstacles(scenario):
        element(monitor, "marker", obstacle["name"])
    env = scenario["environment"]
    if vessel_profile == 'njord':
        # Flat-water Njord forces live exclusively on its model plugin. The
        # upstream wind / wave plugins must not apply a second vessel wrench.
        ET.indent(sdf)
        return ET.tostring(sdf, encoding="unicode", xml_declaration=True)
    # VRX wind on the WAM-V: mean speed and direction plus seeded gusts
    # (wind_variance_gain); wind_direction_deg is the ENU "blowing toward"
    # direction set by configuration.resolve_scenario.
    wind = element(world, "plugin", filename="libUSVWind.so", name="vrx::USVWind")
    obj = element(wind, "wind_obj")
    element(obj, "name", GZ_MODEL_NAME)
    element(obj, "link_name", BASE_FRAME)
    element(obj, "coeff_vector", "0.5 0.5 0.33")
    for key, value in {"wind_direction": env["wind_direction_deg"],
                       "wind_mean_velocity": env["wind_speed_mps"],
                       "var_wind_gain_constants": env["wind_variance_gain"],
                       "var_wind_time_constants": 2, "random_seed": scenario["seed"],
                       "update_rate": 10,
                       "topic_wind_speed": "/vrx/debug/wind/speed",
                       "topic_wind_direction": "/vrx/debug/wind/direction"}.items():
        element(wind, key, value)
    # VRX wave field parameters, republished every 0.1 s on the topic the
    # upstream VRX wave consumers read, so they use the configured sea state.
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
    """Resolve ``scenario_file`` and write the WAM-V world and scenario files.

    Writes njord_course.sdf, resolved_scenario.json (atomically) and
    scenario.sha256 into ``output_dir``; returns the resolved scenario.
    """
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
