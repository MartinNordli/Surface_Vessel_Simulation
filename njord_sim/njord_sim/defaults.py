"""Fallback ROS parameters for reference nodes started on their own.

A simulator run passes every tuning and physical value to the reference nodes
through public_parameters.json. A node started directly (``ros2 run njord_sim
guidance``) has no such file, so it falls back to the values a default run
would use: the WAM-V vessel, algorithms.yaml and the 'fast' speed profile.

Reading them from the packaged YAML means node code never carries a second
copy of a tuning value that could drift from the configuration files.
"""
from functools import lru_cache

from .configuration import (DEFAULT_PROFILE, _read, _resolve_algorithms, autonomy_parameters,
                            config_path, validate_vessel)


@lru_cache(maxsize=1)
def reference_parameters():
    """Parameters per executable for a default WAM-V run (without the seed)."""
    resolved = {
        'vessel': validate_vessel(_read(config_path('vessels/wamv.yaml'))),
        'algorithms': _resolve_algorithms(_read(config_path('algorithms.yaml')), DEFAULT_PROFILE),
        'scenario': {'seed': 1},
    }
    parameters = autonomy_parameters(resolved)
    parameters['sensor_adapter'].pop('seed')  # always set explicitly by the launch
    return parameters


def node_defaults(executable):
    """Name/value pairs for ``Node.declare_parameters`` of one reference node."""
    return list(reference_parameters()[executable].items())
