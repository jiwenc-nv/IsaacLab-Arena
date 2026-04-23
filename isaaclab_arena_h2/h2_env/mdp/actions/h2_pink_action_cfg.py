# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

from dataclasses import MISSING

from isaaclab.managers.action_manager import ActionTerm, ActionTermCfg
from isaaclab.utils import configclass

from isaaclab_arena_h2.h2_env.mdp.actions.h2_pink_action import H2PinkAction


@configclass
class H2PinkActionCfg(ActionTermCfg):
    """Configuration for the H2 PinkIK upper-body action term."""

    class_type: type[ActionTerm] = H2PinkAction

    preserve_order: bool = False
    joint_names: list[str] = MISSING
