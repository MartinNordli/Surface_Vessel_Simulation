#!/usr/bin/env python3
"""Explicit maintenance command: resolve remote refs, never run during builds."""
import json
from pathlib import Path
import re
import subprocess
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
VENDORS = ['gz_math_vendor', 'gz_msgs_vendor', 'gz_sim_vendor',
           'gz_transport_vendor', 'sdformat_vendor']

def get(url, headers=None):
    with urllib.request.urlopen(urllib.request.Request(url, headers=headers or {}), timeout=60) as response:
        return response.read(), response.headers


def update_base_image(dockerfile, image):
    """Replace exactly the pinned external base; preserve subsequent stages."""
    if not re.fullmatch(r'ros:jazzy-ros-base@sha256:[0-9a-f]{64}', image):
        raise ValueError('ROS base must contain a complete SHA256 digest')
    pattern = r'^(FROM[ \t]+)ros:jazzy-ros-base@sha256:[0-9a-f]{64}([ \t]+AS[ \t]+vrx-base[ \t]*)$'
    result, count = re.subn(pattern, lambda match: match[1] + image + match[2],
                            dockerfile, flags=re.MULTILINE)
    if count != 1:
        raise ValueError('Dockerfile must contain exactly one pinned vrx-base FROM declaration')
    return result


def write_lock(root, lock):
    # Validate the Dockerfile before changing any lock artifacts.
    dockerfile = root / 'Dockerfile'
    updated = update_base_image(dockerfile.read_text(), lock['ros_image'])
    repos = {'repositories': {f'gz_libs/{name}': {'type': 'git', 'url': info['url'], 'version': info['commit']}
                             for name, info in lock['vendors'].items()}}
    target = root / 'docker'
    target.mkdir(exist_ok=True)
    artifacts = {dockerfile: updated,
                 target / 'dependencies.lock.json': json.dumps(lock, indent=2) + '\n',
                 target / 'gz.repos': json.dumps(repos, indent=2) + '\n'}
    for path, content in artifacts.items():
        temporary = path.with_name(path.name + '.tmp')
        temporary.write_text(content)
        temporary.replace(path)

def main():
    token = json.loads(get('https://auth.docker.io/token?service=registry.docker.io&scope=repository:library/ros:pull')[0])['token']
    _, headers = get('https://registry-1.docker.io/v2/library/ros/manifests/jazzy-ros-base', {
        'Authorization': 'Bearer ' + token,
        'Accept': 'application/vnd.oci.image.index.v1+json, application/vnd.docker.distribution.manifest.list.v2+json'})
    lock = {'ros_image': 'ros:jazzy-ros-base@' + headers['Docker-Content-Digest'],
            'vrx_commit': '03eae362bb544f630595acd53b931e4a7060dffd', 'vendors': {}}
    previous = json.loads((ROOT/'docker/dependencies.lock.json').read_text())
    if 'physics_runtime' in previous:
        # Physical-library updates require measured regressions, not ref refresh.
        lock['physics_runtime'] = previous['physics_runtime']
    for name in VENDORS:
        url = f'https://github.com/gazebo-release/{name}.git'
        sha = subprocess.check_output(['git', 'ls-remote', url, 'refs/heads/jazzy'], text=True).split()[0]
        lock['vendors'][name] = {'url': url, 'commit': sha}
    write_lock(ROOT, lock)
    print(json.dumps(lock, indent=2))

if __name__ == '__main__':
    main()
