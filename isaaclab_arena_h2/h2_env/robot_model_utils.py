# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

import os


_ROBOT_MENAGERIE_H2_URDF = "robot_menagerie/unitree/h2/urdf/H2_with_sharpa_hands.urdf"


def _resolve_h2_urdf_path() -> str:
    """Resolve the H2 URDF path following the same convention as G1.

    Prefers the Sharpa Wave hand URDF (H2_with_sharpa_hands.urdf) with the
    original Dex3-1 URDF (H2_with_hands.urdf) as a fallback.
    Override with the ``H2_URDF_PATH`` environment variable if needed.
    """
    env_path = os.environ.get("H2_URDF_PATH")
    if env_path and os.path.isfile(env_path):
        return env_path

    _assets_urdf_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "assets", "urdf")

    candidates = [
        # Primary: in-package Sharpa Wave hands URDF
        os.path.join(_assets_urdf_dir, "H2_with_sharpa_hands.urdf"),
        # robot_menagerie on the host (~/repo/robot_menagerie/...)
        os.path.expanduser(f"~/repo/{_ROBOT_MENAGERIE_H2_URDF}"),
        # Fallback: in-package original Dex3-1 hands URDF
        os.path.join(_assets_urdf_dir, "H2_with_hands.urdf"),
        # Fallback: bare H2 without hands
        os.path.expanduser("~/repo/robot_menagerie/unitree/h2/urdf/H2.urdf"),
    ]
    for p in candidates:
        if os.path.isfile(p):
            return p

    raise FileNotFoundError(
        "H2 URDF not found. Ensure ~/repo/robot_menagerie/unitree/h2/ exists "
        "(clone the robot_menagerie repo), or set H2_URDF_PATH."
    )
