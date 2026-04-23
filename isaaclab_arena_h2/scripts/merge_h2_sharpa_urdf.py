# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Merge the H2 body URDF with Sharpa Wave hand URDFs.

Reads the H2 body-only URDF and left/right Sharpa Wave hand URDFs,
grafts the hand link/joint trees onto the wrist_yaw_link of each arm,
rewrites mesh paths, copies mesh files, and writes the merged URDF.

Usage (from repo root):
    python isaaclab_arena_h2/scripts/merge_h2_sharpa_urdf.py
"""

from __future__ import annotations

import os
import re
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
ASSETS_DIR = REPO_ROOT / "isaaclab_arena_h2" / "assets"
H2_BODY_URDF = ASSETS_DIR / "urdf" / "H2.urdf"
OUTPUT_URDF = ASSETS_DIR / "urdf" / "H2_with_sharpa_hands.urdf"

SHARPA_BASE = Path.home() / "repo" / "sharpa-urdf-usd-xml" / "wave_01"
RIGHT_HAND_URDF = SHARPA_BASE / "right_sharpa_wave" / "right_sharpa_wave.urdf"
LEFT_HAND_URDF = SHARPA_BASE / "left_sharpa_wave" / "left_sharpa_wave.urdf"
RIGHT_HAND_MESHES = SHARPA_BASE / "right_sharpa_wave" / "meshes"
LEFT_HAND_MESHES = SHARPA_BASE / "left_sharpa_wave" / "meshes"

ATTACHMENT_JOINTS = {
    "right": {
        "joint_name": "right_sharpa_palm_joint",
        "parent_link": "right_wrist_yaw_link",
        "child_link": "right_hand_C_MC",
        "xyz": "0.0415 -0.003 0",
        "rpy": "3.14159 1.5708 0",
    },
    "left": {
        "joint_name": "left_sharpa_palm_joint",
        "parent_link": "left_wrist_yaw_link",
        "child_link": "left_hand_C_MC",
        "xyz": "0.0415 0.003 0",
        "rpy": "0 1.5708 0",
    },
}

STRIP_WRIST_YAW_VISUALS = True


def _extract_hand_elements(urdf_path: Path) -> list[ET.Element]:
    """Parse a hand URDF and return all <link> and <joint> elements (skip <mujoco>)."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    elements = []
    for child in root:
        if child.tag in ("link", "joint"):
            elements.append(child)
    return elements


def _rewrite_mesh_paths(elements: list[ET.Element], side: str) -> None:
    """Replace package:// mesh paths with local relative paths in-place."""
    pkg_prefix = f"package://{side}_sharpa_wave/meshes/"
    local_prefix = f"meshes/sharpa_{side}/"
    for elem in elements:
        for mesh in elem.iter("mesh"):
            fn = mesh.get("filename", "")
            if fn.startswith(pkg_prefix):
                mesh.set("filename", fn.replace(pkg_prefix, local_prefix))


def _make_attachment_joint_xml(side: str) -> str:
    """Build the fixed joint XML string that attaches a hand to the wrist."""
    cfg = ATTACHMENT_JOINTS[side]
    return (
        f'  <joint name="{cfg["joint_name"]}" type="fixed">\n'
        f'    <origin xyz="{cfg["xyz"]}" rpy="{cfg["rpy"]}"/>\n'
        f'    <parent link="{cfg["parent_link"]}"/>\n'
        f'    <child link="{cfg["child_link"]}"/>\n'
        f"  </joint>\n"
    )


def _elements_to_xml_string(elements: list[ET.Element]) -> str:
    """Serialize a list of ET elements to an indented XML string fragment."""
    lines: list[str] = []
    for elem in elements:
        ET.indent(elem, space="    ", level=0)
        raw = ET.tostring(elem, encoding="unicode")
        # Add two-space root indent to match H2 body style
        for line in raw.splitlines():
            lines.append("  " + line)
        lines.append("")
    return "\n".join(lines) + "\n"


def _find_insertion_index(body_lines: list[str], joint_name: str) -> int:
    """Find the line index *after* the closing </joint> of the named joint.

    Searches for the <joint name="..."> opening tag, then finds its matching
    </joint> and returns the index of the line immediately after.
    """
    open_pattern = re.compile(rf'<joint\s+name="{re.escape(joint_name)}"')
    for i, line in enumerate(body_lines):
        if open_pattern.search(line):
            for j in range(i, len(body_lines)):
                if "</joint>" in body_lines[j]:
                    return j + 1
    raise ValueError(f"Could not locate joint '{joint_name}' in H2 body URDF")


def _copy_meshes(src_dir: Path, dst_dir: Path) -> int:
    """Copy all mesh files from src to dst, returning the count."""
    dst_dir.mkdir(parents=True, exist_ok=True)
    count = 0
    for f in sorted(src_dir.iterdir()):
        if f.is_file():
            shutil.copy2(f, dst_dir / f.name)
            count += 1
    return count


def main() -> None:
    assert H2_BODY_URDF.exists(), f"H2 body URDF not found: {H2_BODY_URDF}"
    assert RIGHT_HAND_URDF.exists(), f"Right hand URDF not found: {RIGHT_HAND_URDF}"
    assert LEFT_HAND_URDF.exists(), f"Left hand URDF not found: {LEFT_HAND_URDF}"

    body_text = H2_BODY_URDF.read_text()
    body_lines = body_text.splitlines(keepends=True)

    # --- Extract and rewrite hand elements ---
    right_elements = _extract_hand_elements(RIGHT_HAND_URDF)
    _rewrite_mesh_paths(right_elements, "right")
    right_xml = _make_attachment_joint_xml("right") + _elements_to_xml_string(right_elements)

    left_elements = _extract_hand_elements(LEFT_HAND_URDF)
    _rewrite_mesh_paths(left_elements, "left")
    left_xml = _make_attachment_joint_xml("left") + _elements_to_xml_string(left_elements)

    # --- Insert left hand first (higher line number) so right insertion index stays valid ---
    left_idx = _find_insertion_index(body_lines, "left_wrist_yaw_joint")
    body_lines.insert(left_idx, left_xml)

    right_idx = _find_insertion_index(body_lines, "right_wrist_yaw_joint")
    body_lines.insert(right_idx, right_xml)

    merged = "".join(body_lines)
    merged = merged.replace('<robot name="H2">', '<robot name="H2_with_sharpa_hands">', 1)

    OUTPUT_URDF.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_URDF.write_text(merged)
    print(f"Merged URDF written to: {OUTPUT_URDF}")

    # --- Copy mesh files ---
    dst_right = ASSETS_DIR / "urdf" / "meshes" / "sharpa_right"
    dst_left = ASSETS_DIR / "urdf" / "meshes" / "sharpa_left"
    n_right = _copy_meshes(RIGHT_HAND_MESHES, dst_right)
    n_left = _copy_meshes(LEFT_HAND_MESHES, dst_left)
    print(f"Copied {n_right} right-hand meshes to: {dst_right}")
    print(f"Copied {n_left} left-hand meshes to: {dst_left}")


if __name__ == "__main__":
    main()
