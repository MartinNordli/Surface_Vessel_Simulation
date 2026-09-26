#!/usr/bin/env bash
# Ubuntu 24.04 host prerequisites. Run with sudo; never installs a GPU driver.
#
#   sudo bash scripts/setup-host.sh
#
# Run once per machine, on the host (not in a container). Installs Docker,
# the Docker Compose v2 plugin and the NVIDIA Container Toolkit, and registers
# the NVIDIA runtime with Docker so containers can use the GPU. The NVIDIA
# driver itself must already be installed (check with nvidia-smi); hosts that
# use NJORD_CPU=1 software rendering do not need a GPU. Afterwards
# ./scripts/njord doctor checks the result.
# Exit codes: 1 if not run as root; otherwise the first failing command's.
set -euo pipefail
if [ "$(id -u)" -ne 0 ]; then
  echo 'Run: sudo bash scripts/setup-host.sh' >&2
  exit 1
fi
apt-get update
apt-get install -y docker.io docker-compose-v2 curl ca-certificates gnupg
# Add NVIDIA's signed apt repository for the container toolkit.
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | gpg --dearmor --yes -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' > /etc/apt/sources.list.d/nvidia-container-toolkit.list
apt-get update
apt-get install -y nvidia-container-toolkit
# Writes the NVIDIA runtime into /etc/docker/daemon.json; restart to load it.
nvidia-ctk runtime configure --runtime=docker
systemctl enable --now docker
systemctl restart docker
# No user is added to the docker group: membership is root-equivalent
# access, so that choice is left to the machine's owner.
echo 'Docker is configured. Run docker through sudo or configure your own Docker access.'
