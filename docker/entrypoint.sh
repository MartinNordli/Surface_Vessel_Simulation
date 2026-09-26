#!/usr/bin/env bash
# Container entrypoint of the simulator image (ENTRYPOINT in the Dockerfile).
# Every Compose service runs through it. It prepares the environment and then
# execs the service command ("$@", e.g. ros2 launch ...), so that command
# becomes the main process and receives stop signals directly.
set -e
# Containers run as the calling host user (see compose.yaml), which has no
# home directory in the image; ROS and Gazebo need a writable HOME.
mkdir -p "${HOME:-/tmp/njord-home}"
# Overlay the workspaces in dependency order: ROS Jazzy, the source-built
# Gazebo vendor packages, VRX, then the Njord packages (last one wins).
source /opt/ros/jazzy/setup.bash
source /opt/ros_gz_ws/install/setup.bash
source /opt/vrx_ws/install/setup.bash
source /opt/njord/install/setup.bash
# Let Gazebo find its installed config, the Njord and VRX system plugins,
# and the VRX models and worlds.
export GZ_CONFIG_PATH="${GZ_CONFIG_PATH:+$GZ_CONFIG_PATH:}/usr/share/gz"
export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/njord/install/lib:/opt/vrx_ws/install/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="/opt/vrx_ws/install/share:/opt/vrx_ws/install/share/vrx_gz/models:/opt/vrx_ws/install/share/vrx_gazebo/models:${GZ_SIM_RESOURCE_PATH:-}"
exec "$@"
