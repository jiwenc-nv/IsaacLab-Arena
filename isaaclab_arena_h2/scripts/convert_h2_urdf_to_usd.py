# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""One-time URDF-to-USD conversion for the Unitree H2 robot.

Usage (inside the Isaac Sim Docker container):
    /isaac-sim/python.sh isaaclab_arena_h2/scripts/convert_h2_urdf_to_usd.py \
        --urdf_path /workspaces/isaaclab_arena/isaaclab_arena_h2/assets/urdf/H2.urdf

The output USD can then be referenced by H2SceneCfg via the H2_USD_PATH env var.
"""

import argparse
import os
import sys

# Where the converted USD lives inside the workspace by default
_DEFAULT_OUTPUT_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "assets")

# Parse arguments BEFORE launching the sim app (AppLauncher consumes its own flags)
parser = argparse.ArgumentParser(description="Convert H2 URDF to USD for Isaac Sim")
parser.add_argument(
    "--urdf_path",
    type=str,
    required=True,
    help="Path to the H2 URDF file",
)
parser.add_argument(
    "--output_dir",
    type=str,
    default=_DEFAULT_OUTPUT_DIR,
    help="Directory to write the output USD file (default: isaaclab_arena_h2/assets/)",
)

# AppLauncher needs to parse its own args too (--headless, etc.)
from isaaclab.app import AppLauncher

AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# Launch the simulator (headless by default for a conversion utility)
args_cli.headless = True
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# --- everything below runs with the Omniverse Kit runtime active ---

from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg

assert os.path.isfile(args_cli.urdf_path), f"URDF not found: {args_cli.urdf_path}"
os.makedirs(args_cli.output_dir, exist_ok=True)

# TODO: Once the H2 USD is hosted on Nucleus or checked into a proper asset
# repo, this script becomes unnecessary. For now it must be run manually after
# cloning.
cfg = UrdfConverterCfg(
    asset_path=args_cli.urdf_path,
    usd_dir=args_cli.output_dir,
    usd_file_name="H2.usd",
    force_usd_conversion=True,
    make_instanceable=False,
    fix_base=True,
    merge_fixed_joints=False,
    self_collision=False,
)
converter = UrdfConverter(cfg)
print(f"\nUSD written to: {converter.usd_path}")

simulation_app.close()
