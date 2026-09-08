"""Launch the D* Lite stack against an already running VRX simulation.

    ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
    ros2 launch njord_sim dstar_demo.launch.py

Every topic name is an argument because they are the single thing most likely
to differ between VRX releases and thruster configurations. Check them against
`ros2 topic list` before blaming the code.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Ground truth positions of the obstacles you spawned, as [x, y, radius, ...].
# Keep in sync with the world file, the evaluator has no other way to know.
OBSTACLES = [
    -480.0, 200.0, 3.0,
    -450.0, 230.0, 3.0,
    -420.0, 190.0, 3.0,
]
GOAL = [-380.0, 250.0]


def generate_launch_description():
    args = [
        DeclareLaunchArgument("odom_topic", default_value="/wamv/ground_truth/odometry"),
        DeclareLaunchArgument("points_topic", default_value="/wamv/sensors/lidars/lidar_wamv_sensor/points"),
        DeclareLaunchArgument("left_topic", default_value="/wamv/thrusters/left/thrust"),
        DeclareLaunchArgument("right_topic", default_value="/wamv/thrusters/right/thrust"),
        DeclareLaunchArgument("output", default_value="run_metrics.json"),
        DeclareLaunchArgument("run_label", default_value="run"),
    ]
    odom = LaunchConfiguration("odom_topic")

    common = {"use_sim_time": True}

    nodes = [
        Node(
            package="njord_sim",
            executable="mapper",
            name="mapper",
            output="screen",
            parameters=[
                common,
                {
                    "odom_topic": odom,
                    "points_topic": LaunchConfiguration("points_topic"),
                    "resolution": 2.0,
                    "size_m": 600.0,
                    "origin_x": -600.0,
                    "origin_y": 0.0,
                    "inflation_m": 8.0,
                    "footprint_m": 6.0,
                },
            ],
        ),
        Node(
            package="njord_sim",
            executable="planner",
            name="planner",
            output="screen",
            parameters=[common, {"odom_topic": odom}],
        ),
        Node(
            package="njord_sim",
            executable="guidance",
            name="guidance",
            output="screen",
            parameters=[
                common,
                {
                    "odom_topic": odom,
                    "left_topic": LaunchConfiguration("left_topic"),
                    "right_topic": LaunchConfiguration("right_topic"),
                },
            ],
        ),
        Node(
            package="njord_sim",
            executable="evaluator",
            name="evaluator",
            output="screen",
            parameters=[
                common,
                {
                    "odom_topic": odom,
                    "goal": GOAL,
                    "obstacles": OBSTACLES,
                    "output": LaunchConfiguration("output"),
                    "run_label": LaunchConfiguration("run_label"),
                },
            ],
        ),
    ]
    return LaunchDescription(args + nodes)
