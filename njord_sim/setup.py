from setuptools import setup

package_name = "njord_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/dstar_demo.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Njord simulation",
    maintainer_email="you@example.com",
    description="D* Lite closed-loop proof of concept for VRX / Gazebo",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mapper = njord_sim.mapper_node:main",
            "planner = njord_sim.planner_node:main",
            "guidance = njord_sim.guidance_node:main",
            "evaluator = njord_sim.evaluator_node:main",
        ],
    },
)
