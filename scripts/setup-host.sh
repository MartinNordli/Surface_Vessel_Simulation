#!/usr/bin/env bash
# Ubuntu 24.04 host prerequisites. Run with sudo; never installs a GPU driver.
set -euo pipefail
if [ "$(id -u)" -ne 0 ]; then
  echo 'Run: sudo bash scripts/setup-host.sh' >&2
  exit 1
fi
apt-get update
apt-get install -y docker.io docker-compose-v2 curl ca-certificates gnupg
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | gpg --dearmor --yes -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' > /etc/apt/sources.list.d/nvidia-container-toolkit.list
apt-get update
apt-get install -y nvidia-container-toolkit
nvidia-ctk runtime configure --runtime=docker
systemctl enable --now docker
systemctl restart docker
echo 'Docker is configured. Run docker through sudo or configure your own Docker access.'
