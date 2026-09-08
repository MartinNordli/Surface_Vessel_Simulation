#!/usr/bin/env python3
"""Explicit maintenance command: resolve remote refs, never run during builds."""
import json
from pathlib import Path
import subprocess
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
VENDORS = ['gz_math_vendor', 'gz_msgs_vendor', 'gz_sim_vendor',
           'gz_transport_vendor', 'sdformat_vendor']

def get(url, headers=None):
    with urllib.request.urlopen(urllib.request.Request(url, headers=headers or {}), timeout=60) as response:
        return response.read(), response.headers

def main():
    token = json.loads(get('https://auth.docker.io/token?service=registry.docker.io&scope=repository:library/ros:pull')[0])['token']
    _, headers = get('https://registry-1.docker.io/v2/library/ros/manifests/jazzy-ros-base', {
        'Authorization': 'Bearer ' + token,
        'Accept': 'application/vnd.oci.image.index.v1+json, application/vnd.docker.distribution.manifest.list.v2+json'})
    lock = {'ros_image': 'ros:jazzy-ros-base@' + headers['Docker-Content-Digest'],
            'vrx_commit': '03eae362bb544f630595acd53b931e4a7060dffd', 'vendors': {}}
    for name in VENDORS:
        url = f'https://github.com/gazebo-release/{name}.git'
        sha = subprocess.check_output(['git', 'ls-remote', url, 'refs/heads/jazzy'], text=True).split()[0]
        lock['vendors'][name] = {'url': url, 'commit': sha}
    target = ROOT / 'docker'
    target.mkdir(exist_ok=True)
    (target / 'dependencies.lock.json').write_text(json.dumps(lock, indent=2) + '\n')
    repos = {'repositories': {f'gz_libs/{name}': {'type': 'git', 'url': info['url'], 'version': info['commit']}
                             for name, info in lock['vendors'].items()}}
    (target / 'gz.repos').write_text(json.dumps(repos, indent=2) + '\n')
    print(json.dumps(lock, indent=2))

if __name__ == '__main__':
    main()
