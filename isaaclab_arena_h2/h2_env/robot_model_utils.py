# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

import os

# TODO: Import from a shared package once RobotModel is refactored out of isaaclab_arena_g1.
from isaaclab_arena_g1.g1_env.robot_model import RobotModel
from isaaclab_arena_h2.h2_env.h2_supplemental_info import H2SupplementalInfo

H2_JOINTS_ORDER_PATH = os.path.join(os.path.dirname(__file__), "config/h2_joints_order_45dof.yaml")


def _resolve_h2_urdf_path() -> str:
    """Resolve the H2 URDF path from H2_URDF_PATH env var or common locations."""
    env_path = os.environ.get("H2_URDF_PATH")
    if env_path and os.path.isfile(env_path):
        return env_path

    # Locate the isaaclab_arena_h2 package so paths work regardless of cwd
    try:
        import isaaclab_arena_h2

        pkg_root = os.path.dirname(os.path.abspath(isaaclab_arena_h2.__file__))
    except ImportError:
        pkg_root = None

    candidates = []
    if pkg_root:
        candidates.append(os.path.join(pkg_root, "assets", "urdf", "H2_with_hands.urdf"))
        candidates.append(os.path.join(pkg_root, "assets", "urdf", "H2.urdf"))

    candidates += [
        "/robot_menagerie/unitree/h2/urdf/H2.urdf",
        os.path.expanduser("~/repo/robot_menagerie/unitree/h2/urdf/H2.urdf"),
    ]
    for p in candidates:
        if os.path.isfile(p):
            return p

    raise FileNotFoundError(
        "H2 URDF not found. Set the H2_URDF_PATH environment variable to the path of H2.urdf, "
        "or copy the URDF into isaaclab_arena_h2/assets/urdf/."
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
