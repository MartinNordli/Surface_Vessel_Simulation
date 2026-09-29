#!/usr/bin/env python3
"""Fingerprint local Docker COPY inputs, including uncommitted source changes.

Includes Dockerfile and .dockerignore. Ignores timestamps, Git internals and
files outside COPY inputs; applies the repository's Docker ignore patterns.
Permissions are reduced to the executable bit Git tracks, so a clone made under
any umask yields the digest that CI stamped into the published image.
The digest identifies source inputs, while the immutable image ID identifies
built binaries (including package versions resolved during the build).

Where it runs: on the host (or in CI), from the repository root. It needs only
Python 3 and, for the commit, git. ``scripts/njord build`` and CI pass the
digest to ``docker build`` as NJORD_IMAGE_SOURCE_DIGEST, which the Dockerfile
stores in the image label ``io.njord.source.digest``;
``scripts/njord check-image`` recomputes it here and compares the two to tell
whether the local image was built from this checkout.

Output (stdout): JSON {"source_commit": ..., "source_digest": ...}, or a single
value with ``--field``. Exits nonzero (traceback) if a COPY source is missing
or not a plain file, directory or symlink inside the build context.

Digest rules, in short: any byte change to the Dockerfile (comments included),
.dockerignore or a copied file changes the digest, as does adding, removing or
renaming a file or toggling its executable bit. Files matched by .dockerignore,
timestamps, owners and non-executable permission bits do not.
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
    """Return the sorted local source paths named by the Dockerfile's COPY lines.

    Handles line continuations, flags such as --chown, and the JSON array
    form. ``COPY --from=<stage>`` is skipped because it copies from another
    build stage, not from the build context. Raises ValueError for a COPY
    source that could escape the context or depends on a build variable,
    since its content could then not be fingerprinted from the checkout.
    """
    result = set()
    text = dockerfile.replace('\\\n', ' ')  # Join backslash-continued lines.
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
        for token in tokens[:-1]:  # The last token is the destination.
            if '$' in token or Path(token).is_absolute() or '..' in Path(token).parts:
                raise ValueError('COPY source must resolve within the build context: ' + token)
            result.add(token)
    return sorted(result)


def ignored(relative, patterns):
    """Return True if ``relative`` is excluded by the .dockerignore ``patterns``.

    A simplified .dockerignore matcher: a pattern excludes a path when it
    matches the path or any parent directory of it, later patterns override
    earlier ones, and a leading ``!`` re-includes. Comments and blank lines
    are skipped.
    """
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


def source_digest(root=ROOT, *, executable=False):
    """Return the SHA-256 hex digest of every Docker build input under ``root``.

    Inputs are the Dockerfile, .dockerignore and everything (recursively) that
    a COPY line names, minus .dockerignore matches. Each entry is hashed in
    sorted path order as a JSON header [path, kind, mode, length] followed by
    its content (file bytes, or the target of a symlink), so the result does
    not depend on filesystem order, timestamps or ownership.
    """
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
    # The version prefix changes the digest if this hashing scheme ever changes.
    digest = hashlib.sha256(b'njord-executable-v1\0' if executable else b'njord-docker-source-v2\0')
    for relative in sorted(selected, key=lambda value: value.as_posix()):
        if relative.as_posix() not in ('Dockerfile', '.dockerignore') and ignored(relative, patterns):
            continue
        path = root / relative
        info = path.lstat()
        if (executable and relative.parts[:2] == ('njord_sim', 'config')
                and relative.suffix.lower() in ('.yaml', '.yml')
                and stat.S_ISREG(info.st_mode)):
            continue
        if stat.S_ISLNK(info.st_mode):
            kind, content = 'link', os.readlink(path).encode()
        elif stat.S_ISREG(info.st_mode):
            kind, content = 'file', path.read_bytes()
        elif stat.S_ISDIR(info.st_mode):
            kind, content = 'directory', b''
        else:
            raise ValueError('Unsupported build input type: ' + str(relative))
        # Only the executable bit Git tracks matters: 0o755 or 0o644, so the
        # local umask cannot change the digest.
        mode = 0o755 if kind != 'file' or info.st_mode & 0o111 else 0o644
        header = json.dumps([relative.as_posix(), kind, mode, len(content)],
                            ensure_ascii=True, separators=(',', ':')).encode()
        digest.update(header + b'\0' + content + b'\0')
    return digest.hexdigest()


def executable_digest(root=ROOT):
    """Fingerprint executable/model inputs; omit only editable regular config YAML."""
    return source_digest(root, executable=True)


def verify_executable(labels, root=ROOT):
    """Reject unlabeled or stale executable/model resources, without a bypass."""
    actual = labels.get('io.njord.executable.digest')
    source = labels.get('io.njord.source.digest', '')
    if not isinstance(source, str) or len(source) != 64 or any(c not in '0123456789abcdef' for c in source):
        raise ValueError('Image full source digest is missing or malformed. Run ./scripts/njord build.')
    expected = executable_digest(root)
    if actual != expected:
        raise ValueError(f'Image executable digest mismatch: image={actual or "missing"}, '
                         f'checkout={expected}. Run ./scripts/njord build.')
    return expected


def build_metadata(root=ROOT):
    """Return {"source_commit": HEAD or "unknown", "source_digest": ...}.

    The commit alone does not describe uncommitted edits; the digest does.
    """
    try:
        commit = subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True,
                                         stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.CalledProcessError):
        commit = 'unknown'
    return {'source_commit': commit, 'source_digest': source_digest(root),
            'executable_digest': executable_digest(root)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--field', choices=['source_commit', 'source_digest', 'executable_digest'])
    args = parser.parse_args()
    metadata = build_metadata()
    print(metadata[args.field] if args.field else json.dumps(metadata, indent=2))


if __name__ == '__main__':
    main()
