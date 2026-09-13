# Copyright 2026 Phuthiphong Wongchantib
#
# Licensed under the Apache License, Version 2.0.
#
# Original author / contributor:
# Phuthiphong Wongchantib
"""YAML configuration store shared by every node.

The customer edits settings from the web UI, so the files that are actually
used must live in a normal writable directory - never inside the colcon
install space. Layout:

    <package>/config/*.yaml      defaults shipped with the code (read only)
    ~/.gps_localize/*.yaml       live configuration, written by the web UI

On first start each default file is copied to the runtime directory. On every
load the defaults are merged underneath the runtime file, so a software update
that adds a new key does not break an existing installation.
"""

import os
import tempfile

import yaml

DEFAULT_RUNTIME_DIR = os.path.expanduser('~/.gps_localize')

# Every config group the UI knows about. One file per topic, so a customer
# only ever opens the one thing they want to change:
#   robot      wheels, track width, top speed
#   nav        waypoint behaviour, GPS quality gate
#   pid        heading and speed controller gains
#   motor      acceleration ramp, electrical motor model
#   estimator  which sensors provide heading and speed
#   safety     watchdog timeouts
#   web        the UI itself
#   waypoints  the route
GROUPS = ('robot', 'nav', 'pid', 'motor', 'estimator', 'safety', 'web', 'waypoints')


def deep_merge(base, override):
    """Return a copy of *base* with *override* merged on top (dicts recurse)."""
    if not isinstance(base, dict) or not isinstance(override, dict):
        return override
    out = dict(base)
    for key, value in override.items():
        out[key] = deep_merge(base.get(key), value) if key in base else value
    return out


class ConfigStore:

    def __init__(self, defaults_dir, runtime_dir=DEFAULT_RUNTIME_DIR, logger=None):
        self.defaults_dir = defaults_dir
        self.runtime_dir = os.path.expanduser(runtime_dir)
        self.logger = logger
        os.makedirs(self.runtime_dir, exist_ok=True)

    # ------------------------------------------------------------------ paths
    def default_path(self, name):
        return os.path.join(self.defaults_dir, name + '.yaml')

    def path(self, name):
        return os.path.join(self.runtime_dir, name + '.yaml')

    # ------------------------------------------------------------------- load
    def load(self, name):
        """Load one group. Missing/broken runtime files fall back to defaults."""
        defaults = self._read(self.default_path(name)) or {}
        runtime_file = self.path(name)
        if not os.path.exists(runtime_file):
            self.save(name, defaults)
            return defaults
        runtime = self._read(runtime_file)
        if runtime is None:                      # unreadable or invalid YAML
            self._log('warn', 'config %s unreadable, using defaults' % name)
            return defaults
        return deep_merge(defaults, runtime)

    def load_all(self, names=GROUPS):
        return {name: self.load(name) for name in names}

    # ------------------------------------------------------------------- save
    def save(self, name, data):
        """Atomically replace one group. Returns True on success."""
        target = self.path(name)
        try:
            fd, tmp = tempfile.mkstemp(dir=self.runtime_dir, prefix='.' + name, suffix='.tmp')
            with os.fdopen(fd, 'w', encoding='utf-8') as handle:
                yaml.safe_dump(data, handle, default_flow_style=False, sort_keys=False,
                               allow_unicode=True)
            os.replace(tmp, target)
            return True
        except OSError as exc:
            self._log('error', 'cannot write %s: %s' % (target, exc))
            return False

    def reset(self, name):
        """Throw the customer's edits away and go back to the shipped defaults."""
        defaults = self._read(self.default_path(name)) or {}
        self.save(name, defaults)
        return defaults

    # ---------------------------------------------------------------- helpers
    def _read(self, path):
        try:
            with open(path, 'r', encoding='utf-8') as handle:
                data = yaml.safe_load(handle)
            return data if isinstance(data, dict) else {}
        except FileNotFoundError:
            return None
        except (OSError, yaml.YAMLError) as exc:
            self._log('warn', 'cannot read %s: %s' % (path, exc))
            return None

    def _log(self, level, message):
        if self.logger is None:
            return
        getattr(self.logger, level, self.logger.info)(message)
