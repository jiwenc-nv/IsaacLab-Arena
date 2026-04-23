# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""H2 teleop environment for upper-body reaching with PinkIK and fixed lower body.

Uses the galileo locomanip background scene. No grasping -- the H2 has no
dexterous hands, so this environment focuses on arm reaching via teleop.
"""

from __future__ import annotations

import argparse
from typing import TYPE_CHECKING

from isaaclab_arena.assets.register import register_environment
from isaaclab_arena_environments.example_environment_base import ExampleEnvironmentBase

if TYPE_CHECKING:
    from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment


@register_environment
class H2TeleopEnvironment(ExampleEnvironmentBase):
    # TODO: Add a pick-and-place task variant once a gripper end-effector is
    # integrated. Currently uses NoTask (reaching only).

    name: str = "h2_teleop"

    def get_env(self, args_cli: argparse.Namespace) -> IsaacLabArenaEnvironment:
        from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
        from isaaclab_arena.scene.scene import Scene
        from isaaclab_arena.tasks.no_task import NoTask
        from isaaclab_arena.utils.pose import Pose

        background = self.asset_registry.get_asset_by_name("galileo_locomanip")()
        embodiment = self.asset_registry.get_asset_by_name(args_cli.embodiment)(enable_cameras=args_cli.enable_cameras)

        if args_cli.teleop_device is not None:
            teleop_device = self.device_registry.get_device_by_name(args_cli.teleop_device)()
        else:
            teleop_device = None

        # TODO: Initial pose is copied from G1 locomanip and places the robot
        # relative to the galileo_locomanip shelf. Adjust for H2's different height
        # and a scene that better suits fixed-base manipulation.
        embodiment.set_initial_pose(Pose(position_xyz=(0.0, 0.18, 0.0), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))

        scene = Scene(assets=[background])
        isaaclab_arena_environment = IsaacLabArenaEnvironment(
            name=self.name,
            embodiment=embodiment,
            scene=scene,
            task=NoTask(),
            teleop_device=teleop_device,
        )
        return isaaclab_arena_environment

    @staticmethod
    def add_cli_args(parser: argparse.ArgumentParser) -> None:
        parser.add_argument("--embodiment", type=str, default="h2_pink")
        parser.add_argument("--teleop_device", type=str, default=None)
