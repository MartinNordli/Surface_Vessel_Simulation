#!/usr/bin/env bash
# Builds and starts Control & Autonomy's DP chain (navigation, dp_node,
# thrust_allocator from the Njord2627 repository) against the simulator.
# Step-by-step guide: docs/control-autonomy.md.
#
#   scripts/ca-nodes.sh setup   # once per machine: Python packages the image lacks
#   scripts/ca-nodes.sh build   # after every change to the C&A code
#   scripts/ca-nodes.sh run     # start the nodes; Ctrl-C stops them
#   scripts/ca-nodes.sh shell   # a shell with the workspace sourced
#
# CA_REPO is the Njord2627 checkout (default: next to this repository).
# CA_WS holds build/, install/, log/ and pydeps/ (default: ca_sim_ws next to
# CA_REPO), so neither repository gets build files. The C&A sources are
# mounted read-only at /team_ws/src.
set -euo pipefail
cd "$(dirname "$0")/.."

CA_REPO="$(realpath -m "${CA_REPO:-../Njord2627}")"
CA_WS="$(realpath -m "${CA_WS:-$CA_REPO/../ca_sim_ws}")"
if [ ! -d "$CA_REPO/munin_ws/src" ]; then
  echo "No C&A sources at $CA_REPO/munin_ws/src; clone Njord2627 there or set CA_REPO" >&2
  exit 2
fi
# The simulator's ROS 2 Jazzy image runs the nodes; Docker would otherwise
# create the mount points as root.
mkdir -p "$CA_WS/src"
export TEAM_WS_HOST="$CA_WS" NJORD_UID="$(id -u)" NJORD_GID="$(id -g)"
team=(docker compose -p njord-ca -f compose.yaml -f compose.team.yaml run --rm
      -v "$CA_REPO/munin_ws/src:/team_ws/src:ro")

case "${1:-}" in
  setup)
    # skadipy and its dependencies are not Debian packages and the image has
    # no pip; pip in a matching Python 3.12 image fills pydeps/. numpy and
    # setuptools come from the ROS image.
    docker run --rm -u "$NJORD_UID:$NJORD_GID" -e HOME=/tmp -v "$CA_WS:/ws" python:3.12-slim \
      pip install --quiet --disable-pip-version-check --no-deps --no-cache-dir --target /ws/pydeps \
      skadipy==0.0.3 scipy==1.17.1 shapely==2.2.0 qpsolvers==4.13.0 osqp==1.1.3 \
      pyquaternion==0.9.9 jinja2==3.1.6 markupsafe==3.0.4 joblib==1.6.0 cloudpickle==3.1.2
    echo "Installed Python packages in $CA_WS/pydeps"
    ;;
  build)
    "${team[@]}" team bash -c \
      'colcon build --base-paths src --packages-up-to navigation dynamic_pos thrust_allocation'
    ;;
  run)
    if [ ! -f "$CA_WS/install/setup.bash" ]; then
      echo "Not built yet: run scripts/ca-nodes.sh build first" >&2
      exit 2
    fi
    # Same nodes and parameter files as dpSimulation.launch.py, plus
    # use_sim_time so the nodes run on the simulator's /clock.
    "${team[@]}" --name ca-nodes team bash -c '
      source install/setup.bash
      export PYTHONPATH=/team_ws/pydeps:$PYTHONPATH
      # Ctrl-C or docker stop: stop the three nodes once and exit cleanly.
      trap "trap - INT TERM; kill \$(jobs -p) 2>/dev/null; wait; exit 0" INT TERM
      # navigation_node logs every odometry message; warn keeps the terminal readable.
      ros2 run navigation navigation_node --ros-args --log-level warn \
        --params-file src/navigation/config/NavigationParams.yaml -p use_sim_time:=true &
      ros2 run dynamic_pos dp_node --ros-args \
        --params-file src/dynamic_pos/config/DPParams.yaml -p use_sim_time:=true &
      ros2 run thrust_allocation thrust_allocator --ros-args -p use_sim_time:=true &
      wait'
    ;;
  shell)
    "${team[@]}" team bash -c 'source install/setup.bash 2>/dev/null
      export PYTHONPATH=/team_ws/pydeps:$PYTHONPATH; exec bash'
    ;;
  *)
    sed -n '2,15p' "$0" | sed 's/^# \{0,1\}//'
    exit 2
    ;;
esac
