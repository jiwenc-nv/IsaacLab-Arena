# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Hot-reload explicit-actuator PD gains from a YAML file during teleop.

Watches a YAML file on disk and, on every poll, applies any changed
``stiffness`` / ``damping`` / ``effort_limit`` values to the live
``IdealPDActuator`` (or any other explicit actuator) instances of an
``Articulation``. New values take effect on the next ``env.step()``
because explicit actuators read their gain tensors at every call to
``compute()``.

YAML schema
-----------

Top-level keys are actuator-group names (matching the keys of
``ArticulationCfg.actuators``). Each value is a dict whose keys are
parameter names and whose values are either a float (broadcast to all
joints in the group) or a ``{joint_name_regex: float}`` dict that uses
the same regex resolution as the original ``ActuatorBaseCfg``.

Example for H2::

    arms:
      stiffness:
        ".*_shoulder_pitch_joint": 150.0
        ".*_elbow_joint": 60.0
      damping: 6.0
      effort_limit: 80.0
    hands:
      stiffness: 6.0

Only the keys present in the YAML are touched; unmentioned groups and
unmentioned params keep their current live values.
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any

import torch
import yaml

# Params handled purely in Python (explicit actuator law). No PhysX write.
_EXPLICIT_PARAMS: tuple[str, ...] = ("stiffness", "damping", "effort_limit")


class LiveGainTuner:
    """Polls a YAML file and applies actuator gain edits to a live articulation.

    Args:
        robot: a live ``Articulation`` (e.g. ``env.scene["robot"]``).
        yaml_path: path to a YAML file with the schema described in the
            module docstring. Missing file is treated as "no edits yet"
            and is not an error.
        poll_interval: minimum seconds between mtime checks. The check
            itself is cheap (a single ``stat()``); the YAML parse only
            runs when the file's mtime has changed.
        verbose: print a one-line summary on each successful apply.
    """

    def __init__(
        self,
        robot,
        yaml_path: str | Path,
        poll_interval: float = 0.5,
        verbose: bool = True,
    ):
        self._robot = robot
        self._path = Path(yaml_path)
        self._poll_interval = poll_interval
        self._verbose = verbose
        self._mtime: float | None = None
        self._last_check_monotonic = 0.0

    def maybe_reload(self) -> bool:
        """Reload + apply if the YAML has changed since the last call.

        Returns:
            True if an apply happened, False otherwise (file unchanged,
            file missing, parse error, or poll interval not elapsed).
        """
        now = time.monotonic()
        if now - self._last_check_monotonic < self._poll_interval:
            return False
        self._last_check_monotonic = now

        if not self._path.exists():
            return False

        try:
            mtime = self._path.stat().st_mtime
        except OSError:
            return False

        if self._mtime is not None and mtime == self._mtime:
            return False
        self._mtime = mtime

        try:
            raw = self._path.read_text()
            spec = yaml.safe_load(raw) or {}
        except (OSError, yaml.YAMLError) as e:
            # Mid-save / partial write / bad YAML: keep live values.
            print(f"[LiveGainTuner] Skipping reload, YAML error: {e}")
            return False

        try:
            n_changed = self._apply(spec)
        except Exception as e:
            print(f"[LiveGainTuner] Apply failed: {e}")
            return False

        if self._verbose:
            print(
                f"[LiveGainTuner] Applied {n_changed} change(s) from "
                f"{self._path.name}"
            )
        return True

    def _apply(self, spec: dict[str, dict[str, Any]]) -> int:
        if not isinstance(spec, dict):
            raise TypeError(f"Top-level YAML must be a dict, got {type(spec).__name__}")

        n_changed = 0
        for group_name, params in spec.items():
            actuator = self._robot.actuators.get(group_name)
            if actuator is None:
                print(f"[LiveGainTuner] Unknown actuator group: {group_name!r}")
                continue
            if params is None:
                continue
            if not isinstance(params, dict):
                raise TypeError(
                    f"Group {group_name!r} value must be a dict, got "
                    f"{type(params).__name__}"
                )

            for param, value in params.items():
                if param not in _EXPLICIT_PARAMS:
                    print(
                        f"[LiveGainTuner] Unsupported param {param!r} on "
                        f"group {group_name!r}. Skipping."
                    )
                    continue
                if self._sync_param(actuator, param, value):
                    n_changed += 1

        return n_changed

    def _sync_param(self, actuator, param: str, value: Any) -> bool:
        """Resolve ``value`` to a tensor and copy it into ``actuator.<param>``.

        Returns True iff the live tensor actually changed.
        """
        live: torch.Tensor = getattr(actuator, param)

        if isinstance(value, dict):
            from isaaclab.utils import string as string_utils

            # Live-tuning dicts are intentionally partial: unmatched joints
            # keep their current values instead of resetting to zero.
            new_tensor = live.clone()
            indices, _, values = string_utils.resolve_matching_names_values(value, actuator.joint_names)
            new_tensor[:, indices] = torch.tensor(values, dtype=torch.float, device=live.device)
        else:
            # Re-use the actuator's own resolver for float / None semantics.
            new_tensor = actuator._parse_joint_parameter(value, default_value=live)

        if torch.equal(live, new_tensor):
            return False

        # In-place copy so any external references to the tensor remain valid.
        live.copy_(new_tensor)
        return True
