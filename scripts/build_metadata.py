#!/usr/bin/env python3
"""Fingerprint local Docker COPY inputs, including uncommitted source changes.

Includes Dockerfile and .dockerignore. Ignores timestamps, Git internals and
files outside COPY inputs; applies the repository's Docker ignore patterns.
The digest identifies source inputs, while the immutable image ID identifies
built binaries (including package versions resolved during the build).
"""
import argparse
import fnmatch
import hashlib
import json
import os
from pathlib import Path
import shlex
import stat
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def copy_sources(dockerfile):
    result = set()
    text = dockerfile.replace('\\\n', ' ')
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped.upper().startswith('COPY '):
            continue
        expression = stripped[5:].strip()
        if expression.startswith('--from'):
            continue  # Stage-to-stage copies contain no additional local input.
        if expression.startswith('['):
            tokens = json.loads(expression)
        else:
            tokens = shlex.split(expression)
            if any(token.startswith('--from=') for token in tokens):
                continue
            tokens = [token for token in tokens if not token.startswith('--')]
        if len(tokens) < 2:
            raise ValueError('Cannot determine COPY sources: ' + line)
        for token in tokens[:-1]:
            if '$' in token or Path(token).is_absolute() or '..' in Path(token).parts:
                raise ValueError('COPY source must resolve within the build context: ' + token)
            result.add(token)
    return sorted(result)


def ignored(relative, patterns):
    parts = relative.parts
    prefixes = [Path(*parts[:i]).as_posix() for i in range(1, len(parts) + 1)]
    excluded = False
    for raw in patterns:
        if not raw or raw.startswith('#'):
            continue
        negate = raw.startswith('!')
        pattern = raw[1:] if negate else raw
        pattern = pattern.strip('/')
        if any(fnmatch.fnmatchcase(prefix, pattern) for prefix in prefixes):
            excluded = not negate
    return excluded


def source_digest(root=ROOT):
    root = Path(root)
    dockerfile = (root / 'Dockerfile').read_text()
    patterns = (root / '.dockerignore').read_text().splitlines() if (root / '.dockerignore').exists() else []
    selected = {Path('Dockerfile')}
    if (root / '.dockerignore').exists():
        selected.add(Path('.dockerignore'))
    for source in copy_sources(dockerfile):
        candidates = list(root.glob(source))
        if not candidates:
            raise ValueError('Docker COPY source missing: ' + source)
        for candidate in candidates:
            selected.add(candidate.relative_to(root))
            if candidate.is_dir() and not candidate.is_symlink():
                for directory, dirs, files in os.walk(candidate, followlinks=False):
                    selected.update((Path(directory) / name).relative_to(root) for name in dirs + files)
    digest = hashlib.sha256(b'njord-docker-source-v1\0')
    for relative in sorted(selected, key=lambda value: value.as_posix()):
        if relative.as_posix() not in ('Dockerfile', '.dockerignore') and ignored(relative, patterns):
            continue
        path = root / relative
        info = path.lstat()
        if stat.S_ISLNK(info.st_mode):
            kind, content = 'link', os.readlink(path).encode()
        elif stat.S_ISREG(info.st_mode):
            kind, content = 'file', path.read_bytes()
        elif stat.S_ISDIR(info.st_mode):
            kind, content = 'directory', b''
        else:
            raise ValueError('Unsupported build input type: ' + str(relative))
        header = json.dumps([relative.as_posix(), kind, stat.S_IMODE(info.st_mode), len(content)],
                            ensure_ascii=True, separators=(',', ':')).encode()
        digest.update(header + b'\0' + content + b'\0')
    return digest.hexdigest()


def build_metadata(root=ROOT):
    try:
        commit = subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True,
                                         stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.CalledProcessError):
        commit = 'unknown'
    return {'source_commit': commit, 'source_digest': source_digest(root)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--field', choices=['source_commit', 'source_digest'])
    args = parser.parse_args()
    metadata = build_metadata()
    print(metadata[args.field] if args.field else json.dumps(metadata, indent=2))


if __name__ == '__main__':
    main()
