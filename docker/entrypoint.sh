#!/usr/bin/env bash
set -e
mkdir -p "${HOME:-/tmp/njord-home}"
source /opt/ros/jazzy/setup.bash
source /opt/ros_gz_ws/install/setup.bash
source /opt/vrx_ws/install/setup.bash
source /opt/njord/install/setup.bash
export GZ_CONFIG_PATH="${GZ_CONFIG_PATH:+$GZ_CONFIG_PATH:}/usr/share/gz"
export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/njord/install/lib:/opt/vrx_ws/install/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="/opt/vrx_ws/install/share:/opt/vrx_ws/install/share/vrx_gz/models:/opt/vrx_ws/install/share/vrx_gazebo/models:${GZ_SIM_RESOURCE_PATH:-}"
exec "$@"
