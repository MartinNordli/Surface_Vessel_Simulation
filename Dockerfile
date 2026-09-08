# Based on osrf/vrx v3.1.0 docker/Dockerfile.base (Apache-2.0).
FROM ros:jazzy-ros-base@sha256:2589a8fba5257307857890173c069852c2abf913a0be7970f172478baecb09e4 AS vrx-base
ENV DEBIAN_FRONTEND=noninteractive

# Setup timezone
ENV TZ=Etc/UTC
RUN echo $TZ > /etc/timezone && \
    ln -fs /usr/share/zoneinfo/$TZ /etc/localtime && \
    apt update && \
    apt install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Set up repo to install Gazebo
RUN curl -s https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install gz-harmonic
RUN apt update \
    && apt install -y --no-install-recommends \
       gz-harmonic \
       equivs \
    && rm -rf /var/lib/apt/lists/* \
    && apt clean -qq

# Solution for problem https://github.com/osrf/vrx/issues/839
# Create fake providers for the ROS Gazebo vendor packages of the Gz libraries
# that support Python bindings and use a colcon workspace to inject
# the real ROS packages.

# Create the dummy package control file for ROS Jazzy Gazebo vendor packages
COPY docker/fake-vendor.control /tmp/fake-ros-jazzy-gz-vendor


# Build and install the dummy package
RUN cd /tmp \
    && cat fake-ros-jazzy-gz-vendor \
    && equivs-build fake-ros-jazzy-gz-vendor \
    && dpkg -i fake-ros-jazzy-gz-vendor_*_all.deb \
    && rm -f /tmp/fake-ros-jazzy-gz-vendor /tmp/fake-ros-jazzy-gz-vendor_*_all.deb

COPY docker/gz.repos /tmp/gz.repos

RUN mkdir -p /opt/ros_gz_ws/src \
    && cd /opt/ros_gz_ws \
    && vcs import src < /tmp/gz.repos \
    && apt update \
    && rosdep install -r --from-paths . --ignore-src --rosdistro jazzy -y --skip-keys="gz_math_vendor gz_msgs_vendor gz_sim_vendor gz_transport_vendor sdformat_vendor" \
    && . /opt/ros/jazzy/setup.sh \
    && colcon build --parallel-workers 2 --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source for interactive shells (both login and non-login)
RUN echo "source /opt/ros_gz_ws/install/setup.bash" > /etc/profile.d/ros_gz_pythons.sh \
    && echo "export GZ_CONFIG_PATH=\$GZ_CONFIG_PATH:/usr/share/gz/" > /etc/profile.d/ros_gz_pythons.sh \
    && chmod +x /etc/profile.d/ros_gz_pythons.sh \
    && echo "source /etc/profile.d/ros_gz_pythons.sh" >> /etc/bash.bashrc
# Source for non-interactive shells
ENV BASH_ENV=/etc/profile.d/ros_gz_pythons.sh

FROM vrx-base AS vrx-runtime
WORKDIR /opt/vrx_ws
RUN apt-get update && apt-get install -y --no-install-recommends git python3-colcon-common-extensions python3-vcstool ros-jazzy-robot-localization ros-jazzy-cv-bridge ros-jazzy-vision-msgs ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs ros-jazzy-rviz2 python3-opencv python3-numpy python3-yaml python3-pytest mesa-utils libegl1 libgl1-mesa-dri && rm -rf /var/lib/apt/lists/*
RUN git clone https://github.com/osrf/vrx.git src/vrx && cd src/vrx && git checkout 03eae362bb544f630595acd53b931e4a7060dffd
RUN . /opt/ros_gz_ws/install/setup.sh && apt-get update && rosdep install --from-paths src --ignore-src --rosdistro jazzy -y && colcon build --merge-install --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF && rm -rf /var/lib/apt/lists/*
RUN . /opt/ros_gz_ws/install/setup.sh && python3 -c 'import sdformat14'
WORKDIR /opt/njord
COPY njord_sim njord_sim
COPY njord_gz_plugins njord_gz_plugins
RUN . /opt/vrx_ws/install/setup.sh && colcon build --merge-install --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release
COPY scenarios /opt/njord/scenarios
COPY tests /opt/njord/tests
COPY scripts /opt/njord/scripts
COPY validation /opt/njord/validation
COPY docker/entrypoint.sh /entrypoint.sh
COPY docker/dependencies.lock.json /opt/njord/dependencies.lock.json
ENV PYTHONUNBUFFERED=1 NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute,display ROS_DOMAIN_ID=42
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "njord_sim", "simulation.launch.py"]
