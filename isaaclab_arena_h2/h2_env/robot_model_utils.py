# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

import os

# TODO: Import from a shared package once RobotModel is refactored out of isaaclab_arena_g1.
from isaaclab_arena_g1.g1_env.robot_model import RobotModel
from isaaclab_arena_h2.h2_env.h2_supplemental_info import H2SupplementalInfo

H2_JOINTS_ORDER_PATH = os.path.join(os.path.dirname(__file__), "config/h2_joints_order_45dof.yaml")


_ROBOT_MENAGERIE_H2_URDF = "robot_menagerie/unitree/h2/urdf/H2_with_hands.urdf"


def _resolve_h2_urdf_path() -> str:
    """Resolve the H2 URDF path following the same convention as G1.

    The canonical source is ``~/repo/robot_menagerie/unitree/h2/urdf/``.
    Override with the ``H2_URDF_PATH`` environment variable if needed.
    """
    env_path = os.environ.get("H2_URDF_PATH")
    if env_path and os.path.isfile(env_path):
        return env_path

    # In-package URDF shipped alongside the assets (works inside Docker where
    # robot_menagerie is not mounted).
    _pkg_urdf = os.path.join(os.path.dirname(os.path.dirname(__file__)), "assets", "urdf", "H2_with_hands.urdf")

    candidates = [
        # Primary: robot_menagerie on the host (~/repo/robot_menagerie/...)
        os.path.expanduser(f"~/repo/{_ROBOT_MENAGERIE_H2_URDF}"),
        # In-package copy (always available if the repo is mounted)
        _pkg_urdf,
        # Fallback without hands
        os.path.expanduser("~/repo/robot_menagerie/unitree/h2/urdf/H2.urdf"),
    ]
    for p in candidates:
        if os.path.isfile(p):
            return p

    raise FileNotFoundError(
        "H2 URDF not found. Ensure ~/repo/robot_menagerie/unitree/h2/ exists "
        "(clone the robot_menagerie repo), or set H2_URDF_PATH."
    )


def instantiate_h2_robot_model(
    urdf_path: str | None = None,
    asset_path: str | None = None,
) -> RobotModel:
    """Instantiate an H2 robot model for PinkIK upper-body control.

    Args:
        urdf_path: Path to the H2 URDF. Auto-resolved from H2_URDF_PATH env var or
                   common locations if not provided.
        asset_path: Package directory for mesh resolution. Defaults to the URDF's parent dir.

    Returns:
        A RobotModel configured for the H2.
    """
    if urdf_path is None:
        urdf_path = _resolve_h2_urdf_path()
    if asset_path is None:
        asset_path = os.path.dirname(urdf_path)

    assert os.path.isfile(urdf_path), f"H2 URDF not found at {urdf_path}"

    supplemental_info = H2SupplementalInfo()

    robot_model = RobotModel(
        urdf_path=urdf_path,
        asset_path=asset_path,
        supplemental_info=supplemental_info,
        joints_order_path=H2_JOINTS_ORDER_PATH,
    )
    return robot_model
