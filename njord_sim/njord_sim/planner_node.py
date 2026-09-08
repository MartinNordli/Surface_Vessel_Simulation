"""D* Lite planner node.

Holds one persistent D* Lite search and repairs it as the occupancy grid
changes, which is the whole point of using D* Lite rather than re-running A*.
Publishes the plan latency on /njord/plan_ms so that the incremental repair can
be compared against a full replan with real numbers rather than assertion.
"""

import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Float64

from njord_sim.dstar_lite import INF, DStarLite, Grid

OCCUPIED = 50  # OccupancyGrid values at or above this count as blocked


class Planner(Node):
    def __init__(self):
        super().__init__("planner")
        self.declare_parameters(
            "",
            [
                ("grid_topic", "/njord/occupancy"),
                ("odom_topic", "/wamv/ground_truth/odometry"),
                ("goal_topic", "/njord/goal"),
                ("path_topic", "/njord/path"),
                ("map_frame", "map"),
            ],
        )
        p = self.get_parameter
        self.map_frame = p("map_frame").value

        self.grid = None
        self.planner = None
        self.previous = None
        self.info = None
        self.goal = None
        self.position = None

        self.create_subscription(OccupancyGrid, p("grid_topic").value, self.on_grid, 1)
        self.create_subscription(Odometry, p("odom_topic").value, self.on_odom, 10)
        self.create_subscription(PoseStamped, p("goal_topic").value, self.on_goal, 1)
        self.path_pub = self.create_publisher(Path, p("path_topic").value, 1)
        self.latency_pub = self.create_publisher(Float64, "/njord/plan_ms", 1)

    # ------------------------------------------------------------------ #

    def to_cell(self, x, y):
        return (
            int((y - self.info.origin.position.y) // self.info.resolution),
            int((x - self.info.origin.position.x) // self.info.resolution),
        )

    def to_world(self, cell):
        return (
            self.info.origin.position.x + (cell[1] + 0.5) * self.info.resolution,
            self.info.origin.position.y + (cell[0] + 0.5) * self.info.resolution,
        )

    def on_odom(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def on_goal(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.planner = None  # force a fresh search
        self.get_logger().info(f"new goal: {self.goal[0]:.1f}, {self.goal[1]:.1f}")

    def on_grid(self, msg):
        if self.position is None or self.goal is None:
            return

        occupancy = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width) >= OCCUPIED
        geometry_changed = self.info is None or (
            msg.info.resolution != self.info.resolution
            or msg.info.width != self.info.width
            or msg.info.origin.position.x != self.info.origin.position.x
        )
        self.info = msg.info

        if self.planner is None or geometry_changed:
            self.grid = Grid(msg.info.height, msg.info.width)
            self.grid.blocked = {tuple(c) for c in np.argwhere(occupancy)}
            self.previous = occupancy.copy()
            self.planner = DStarLite(self.grid, self.to_cell(*self.position), self.to_cell(*self.goal))
            self.replan(fresh=True)
            return

        changed = [tuple(c) for c in np.argwhere(occupancy != self.previous)]
        self.previous = occupancy.copy()
        for cell in changed:
            self.grid.set_blocked(cell, bool(occupancy[cell]))

        start = self.to_cell(*self.position)
        if not changed and start == self.planner.start:
            return

        self.planner.update_start(start)
        if changed:
            self.planner.apply_changes(changed)
        self.replan()

    def replan(self, fresh=False):
        t0 = time.perf_counter()
        self.planner.compute_shortest_path()
        cells = self.planner.path()
        elapsed = (time.perf_counter() - t0) * 1e3
        self.latency_pub.publish(Float64(data=elapsed))

        if not cells:
            self.get_logger().warn("no feasible path to goal")
            return

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.map_frame
        for cell in cells:
            x, y = self.to_world(cell)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)

        if fresh:
            cost = self.planner.path_cost()
            self.get_logger().info(
                f"initial plan: {len(cells)} cells, cost {cost:.1f}, {elapsed:.1f} ms"
                if cost != INF
                else "initial plan: unreachable"
            )


def main():
    rclpy.init()
    node = Planner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
