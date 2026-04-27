# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""H2 teleop environment with PinkIK and fixed lower body.

Uses the galileo locomanip background scene. Optionally spawns interactable
objects via ``--object`` / ``--destination`` for teleop grasping practice.
"""

from __future__ import annotations

import argparse
import math
from typing import TYPE_CHECKING

from isaaclab_arena.assets.register import register_environment
from isaaclab_arena_environments.example_environment_base import ExampleEnvironmentBase

if TYPE_CHECKING:
    from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment


@register_environment
class H2TeleopEnvironment(ExampleEnvironmentBase):

    name: str = "h2_teleop"

    def get_env(self, args_cli: argparse.Namespace) -> IsaacLabArenaEnvironment:
        from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
        from isaaclab_arena.scene.scene import Scene
        from isaaclab_arena.tasks.no_task import NoTask
        from isaaclab_arena.utils.pose import Pose, PoseRange

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

        scene_assets = [background]

        if args_cli.object is not None:
            pick_up_object = self.asset_registry.get_asset_by_name(args_cli.object)()
            XY_RANGE_M = 0.025
            pick_up_object.set_initial_pose(
                PoseRange(
                    position_xyz_min=(0.5785 - XY_RANGE_M, 0.18 - XY_RANGE_M, 0.0707),
                    position_xyz_max=(0.5785 + XY_RANGE_M, 0.18 + XY_RANGE_M, 0.0707),
                    rpy_min=(math.pi, 0.0, math.pi),
                    rpy_max=(math.pi, 0.0, math.pi),
                )
            )
            scene_assets.append(pick_up_object)

        if args_cli.destination is not None:
            destination = self.asset_registry.get_asset_by_name(args_cli.destination)()
            destination.set_initial_pose(
                Pose(
                    position_xyz=(-0.2450, -1.6272, -0.2641),
                    rotation_xyzw=(0.0, 0.0, 1.0, 0.0),
                )
            )
            scene_assets.append(destination)

        scene = Scene(assets=scene_assets)
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
        parser.add_argument("--object", type=str, default=None, help="Interactable object to spawn (e.g. brown_box)")
        parser.add_argument("--destination", type=str, default=None, help="Destination asset to spawn (e.g. blue_sorting_bin)")
