# Copyright (c) 2025-2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Helper to opt every IsaacTeleop session into pipelined retargeting by default.

IsaacTeleop 1.2.x introduced a `retargeting_execution` field on
``TeleopSessionConfig`` that selects between the legacy synchronous retargeter
and a new pipelined latest-result retargeter (TeleopCore commit 260f7660 /
PR #467). The new mode runs retargeting on a background thread and feeds the
sim loop the most recent finished result, paced by a deadline scheduler --
which avoids stalling the sim step on slow IK / dexpilot solves and matters
in particular for high-DOF embodiments such as H2 + Sharpa Wave hands.

To enable this without requiring every ``env_cfg.isaac_teleop`` to opt in (and
to keep arena scripts that construct sessions indirectly via
``create_isaac_teleop_device`` covered), :func:`enable_pipelined_retargeting_default`
monkey-patches ``isaacteleop.teleop_session_manager.TeleopSessionConfig`` so
its constructor defaults ``retargeting_execution`` to a pipelined config when
the caller did not specify one. Sessions that pass an explicit
``retargeting_execution`` are left unchanged.

The patch is idempotent: subsequent calls are no-ops.
"""

from __future__ import annotations

from typing import Any


def enable_pipelined_retargeting_default(safety_margin_s: float = 0.025) -> None:
    """Monkey-patch ``TeleopSessionConfig`` to default to pipelined retargeting.

    Wraps :class:`isaacteleop.teleop_session_manager.TeleopSessionConfig` so that
    constructions which omit ``retargeting_execution`` receive a pipelined
    config with deadline pacing. Explicit values from callers are preserved.

    Idempotent -- safe to call from multiple entry points.

    Args:
        safety_margin_s: Pacing safety margin (seconds) handed to the deadline
            scheduler. Larger values give the retargeter more slack but
            increase teleop-to-sim latency.

    Raises:
        ImportError: If ``isaacteleop`` is not installed (e.g. an environment
            built without the IsaacTeleop extras).
    """
    from isaacteleop import teleop_session_manager as tsm

    if getattr(tsm.TeleopSessionConfig, "_arena_pipelined_default_applied", False):
        return

    base_teleop_session_config = tsm.TeleopSessionConfig

    def teleop_session_config_with_pipelined_default(*args: Any, **kwargs: Any) -> Any:
        kwargs.setdefault(
            "retargeting_execution",
            tsm.RetargetingExecutionConfig(
                mode="pipelined",
                pacing=tsm.DeadlinePacingConfig(safety_margin_s=safety_margin_s),
            ),
        )
        return base_teleop_session_config(*args, **kwargs)

    teleop_session_config_with_pipelined_default._arena_pipelined_default_applied = True
    tsm.TeleopSessionConfig = teleop_session_config_with_pipelined_default
