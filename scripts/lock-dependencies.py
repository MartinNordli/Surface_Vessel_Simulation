#!/usr/bin/env python3
"""Explicit maintenance command: resolve remote refs, never run during builds.

Refreshes the pinned third-party inputs of the Docker image:

- the ``ros:jazzy-ros-base`` base image digest (Docker Hub) in the Dockerfile;
- the HEAD commit of the ``jazzy`` branch of each Gazebo vendor package
  (gazebo-release/<name>) written to docker/gz.repos, which the Dockerfile
  builds from source;
- docker/dependencies.lock.json, which records all of the above plus the VRX
  commit and the carried-over ``physics_runtime`` pins.

Where to run: on the host, from a checkout, with network access and git:
``python3 scripts/lock-dependencies.py``. Nothing else needs to be running.
It is never called by the build, so a build always uses the committed pins.
Prints the new lock as JSON. Afterwards rebuild the image, run the tests and
review the diff before committing; any change here changes the image.

The VRX commit is hard-coded below and must match the ``git checkout`` in the
Dockerfile. The physics_runtime pins (libgz-sim8 / libgz-physics7 versions,
also asserted in the Dockerfile) are copied unchanged: updating them needs
measured dynamics regressions, not a ref refresh.
"""
import json
from pathlib import Path
import re
import subprocess
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
VENDORS = ['gz_math_vendor', 'gz_msgs_vendor', 'gz_sim_vendor',
           'gz_transport_vendor', 'sdformat_vendor']

def get(url, headers=None):
    """HTTP GET ``url`` (60 s timeout); return (body bytes, response headers)."""
    with urllib.request.urlopen(urllib.request.Request(url, headers=headers or {}), timeout=60) as response:
        return response.read(), response.headers


def update_base_image(dockerfile, image):
    """Replace exactly the pinned external base; preserve subsequent stages.

    ``image`` must be ``ros:jazzy-ros-base@sha256:<64 hex>``. Only the one
    ``FROM ros:jazzy-ros-base@sha256:... AS vrx-base`` line is rewritten; any
    other text (comments included) is kept byte for byte. Returns the new
    Dockerfile text; raises ValueError unless exactly one such line exists.
    """
    if not re.fullmatch(r'ros:jazzy-ros-base@sha256:[0-9a-f]{64}', image):
        raise ValueError('ROS base must contain a complete SHA256 digest')
    pattern = r'^(FROM[ \t]+)ros:jazzy-ros-base@sha256:[0-9a-f]{64}([ \t]+AS[ \t]+vrx-base[ \t]*)$'
    result, count = re.subn(pattern, lambda match: match[1] + image + match[2],
                            dockerfile, flags=re.MULTILINE)
    if count != 1:
        raise ValueError('Dockerfile must contain exactly one pinned vrx-base FROM declaration')
    return result


def write_lock(root, lock):
    """Write the Dockerfile, dependencies.lock.json and gz.repos for ``lock``.

    Each file is written to a ``.tmp`` sibling and renamed into place, so a
    reader never sees a half-written file.
    """
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
    """Resolve the current remote refs and rewrite the lock artifacts."""
    # Anonymous Docker Hub pull token, then read the manifest-list digest of
    # the multi-architecture tag (the digest in the Docker-Content-Digest header).
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
