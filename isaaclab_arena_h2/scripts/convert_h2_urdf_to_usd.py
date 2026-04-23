# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""One-time URDF-to-USD conversion for the Unitree H2 robot with Sharpa Wave hands.

Prerequisites:
    The H2_with_sharpa_hands.urdf (H2 body + Sharpa Wave hands) must be present in
    the in-package assets directory (isaaclab_arena_h2/assets/urdf/).

    Alternatively, clone the robot_menagerie repo at ~/repo/robot_menagerie so that
    the H2 URDF and meshes are available at ~/repo/robot_menagerie/unitree/h2/urdf/.

Usage (inside the Isaac Sim Docker container):
    /isaac-sim/python.sh isaaclab_arena_h2/scripts/convert_h2_urdf_to_usd.py

    # Or with an explicit path:
    /isaac-sim/python.sh isaaclab_arena_h2/scripts/convert_h2_urdf_to_usd.py \\
        --urdf_path ~/repo/robot_menagerie/unitree/h2/urdf/H2_with_sharpa_hands.urdf

The output USD is written to isaaclab_arena_h2/assets/ and will be auto-discovered
by the H2 embodiment at runtime.
"""

import argparse
import os

_PKG_DIR = os.path.dirname(os.path.dirname(__file__))
_DEFAULT_URDF_PATH = os.path.join(_PKG_DIR, "assets", "urdf", "H2_with_sharpa_hands.urdf")
_DEFAULT_OUTPUT_DIR = os.path.join(_PKG_DIR, "assets")

parser = argparse.ArgumentParser(description="Convert H2 URDF to USD for Isaac Sim")
parser.add_argument(
    "--urdf_path",
    type=str,
    default=_DEFAULT_URDF_PATH,
    help=f"Path to the H2 URDF file (default: {_DEFAULT_URDF_PATH})",
)
parser.add_argument(
    "--output_dir",
    type=str,
    default=_DEFAULT_OUTPUT_DIR,
    help="Directory to write the output USD file (default: isaaclab_arena_h2/assets/)",
)

from isaaclab.app import AppLauncher

AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

args_cli.headless = True
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg

assert os.path.isfile(args_cli.urdf_path), (
    f"URDF not found: {args_cli.urdf_path}\n"
    "Ensure H2_with_sharpa_hands.urdf exists in isaaclab_arena_h2/assets/urdf/ "
    "or provide an explicit --urdf_path."
)
os.makedirs(args_cli.output_dir, exist_ok=True)

# TODO: Once the H2 USD is hosted on Nucleus, this script becomes unnecessary.
cfg = UrdfConverterCfg(
    asset_path=args_cli.urdf_path,
    usd_dir=args_cli.output_dir,
    usd_file_name="H2_with_sharpa_hands.usda",
    force_usd_conversion=True,
    make_instanceable=False,
    fix_base=True,
    merge_fixed_joints=False,
    self_collision=False,
)
converter = UrdfConverter(cfg)
print(f"\nUSD written to: {converter.usd_path}")

simulation_app.close()
