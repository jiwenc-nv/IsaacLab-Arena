# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Custom IsaacTeleop pipeline for H2 PinkIK with Sharpa Wave hands: 58D output.

Output layout (58D):
    [left_wrist_pos(3), left_wrist_quat(4),
     right_wrist_pos(3), right_wrist_quat(4),
     hand_joints(44) in PhysX BFS articulation order]

Wrist SE3 from Se3AbsRetargeter connected to HandsSource (hand-tracking wrist poses).
Finger joints from DexHandRetargeter using DexPilot retargeting from raw hand tracking.
No locomotion (lower body is fixed).
"""

import os


def _build_h2_pink_pipeline():
    """Build an IsaacTeleop retargeting pipeline for H2 PinkIK with Sharpa Wave hands.

    Creates two Se3AbsRetargeters for left and right wrist pose tracking (from
    hand-tracking wrist data) and two DexHandRetargeters for left and right
    Sharpa Wave dexterous hand finger control. All outputs are flattened into
    a single 58D action tensor via TensorReorderer.

    Returns:
        OutputCombiner with a single "action" output (58D flattened tensor).
    """
    from isaacteleop.retargeters import (
        DexHandRetargeter,
        DexHandRetargeterConfig,
        Se3AbsRetargeter,
        Se3RetargeterConfig,
        TensorReorderer,
    )
    from isaacteleop.retargeting_engine.deviceio_source_nodes import HandsSource
    from isaacteleop.retargeting_engine.interface import OutputCombiner, ValueInput
    from isaacteleop.retargeting_engine.tensor_types import TransformMatrix

    hands = HandsSource(name="hands")

    transform_input = ValueInput("world_T_anchor", TransformMatrix())
    transformed_hands = hands.transformed(transform_input.output(ValueInput.VALUE))

    # -------------------------------------------------------------------------
    # SE3 Absolute Pose Retargeters (left and right wrists)
    # Offsets calibrated for the H2 wrist frame conventions (from h2-bringup branch).
    # -------------------------------------------------------------------------
    left_se3_cfg = Se3RetargeterConfig(
        input_device=HandsSource.LEFT,
        zero_out_xy_rotation=False,
        use_wrist_rotation=False,
        use_wrist_position=False,
        target_offset_roll=45.0,
        target_offset_pitch=180.0,
        target_offset_yaw=-90.0,
    )
    left_se3 = Se3AbsRetargeter(left_se3_cfg, name="left_ee_pose")
    connected_left_se3 = left_se3.connect(
        {HandsSource.LEFT: transformed_hands.output(HandsSource.LEFT)}
    )

    right_se3_cfg = Se3RetargeterConfig(
        input_device=HandsSource.RIGHT,
        zero_out_xy_rotation=False,
        use_wrist_rotation=False,
        use_wrist_position=False,
        target_offset_roll=-135.0,
        target_offset_pitch=0.0,
        target_offset_yaw=90.0,
    )
    right_se3 = Se3AbsRetargeter(right_se3_cfg, name="right_ee_pose")
    connected_right_se3 = right_se3.connect(
        {HandsSource.RIGHT: transformed_hands.output(HandsSource.RIGHT)}
    )

    # -------------------------------------------------------------------------
    # DexHand Retargeters (left and right Sharpa Wave hands)
    # -------------------------------------------------------------------------
    _this_dir = os.path.dirname(__file__)
    left_yaml_path = os.path.join(_this_dir, "data", "configs", "sharpa_wave_left_dexpilot.yml")
    right_yaml_path = os.path.join(_this_dir, "data", "configs", "sharpa_wave_right_dexpilot.yml")

    _assets_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "assets", "urdf", "sharpa_standalone")
    left_hand_urdf = os.path.join(_assets_dir, "left_sharpa_wave.urdf")
    right_hand_urdf = os.path.join(_assets_dir, "right_sharpa_wave.urdf")

    operator2mano = (0, -1, 0, -1, 0, 0, 0, 0, -1)

    left_hand_joint_names = [
        "left_thumb_CMC_FE",
        "left_thumb_CMC_AA",
        "left_thumb_MCP_FE",
        "left_thumb_MCP_AA",
        "left_thumb_IP",
        "left_index_MCP_FE",
        "left_index_MCP_AA",
        "left_index_PIP",
        "left_index_DIP",
        "left_middle_MCP_FE",
        "left_middle_MCP_AA",
        "left_middle_PIP",
        "left_middle_DIP",
        "left_ring_MCP_FE",
        "left_ring_MCP_AA",
        "left_ring_PIP",
        "left_ring_DIP",
        "left_pinky_CMC",
        "left_pinky_MCP_FE",
        "left_pinky_MCP_AA",
        "left_pinky_PIP",
        "left_pinky_DIP",
    ]

    right_hand_joint_names = [
        "right_thumb_CMC_FE",
        "right_thumb_CMC_AA",
        "right_thumb_MCP_FE",
        "right_thumb_MCP_AA",
        "right_thumb_IP",
        "right_index_MCP_FE",
        "right_index_MCP_AA",
        "right_index_PIP",
        "right_index_DIP",
        "right_middle_MCP_FE",
        "right_middle_MCP_AA",
        "right_middle_PIP",
        "right_middle_DIP",
        "right_ring_MCP_FE",
        "right_ring_MCP_AA",
        "right_ring_PIP",
        "right_ring_DIP",
        "right_pinky_CMC",
        "right_pinky_MCP_FE",
        "right_pinky_MCP_AA",
        "right_pinky_PIP",
        "right_pinky_DIP",
    ]

    left_dex_cfg = DexHandRetargeterConfig(
        hand_retargeting_config=left_yaml_path,
        hand_urdf=left_hand_urdf,
        hand_joint_names=left_hand_joint_names,
        hand_side="left",
        handtracking_to_baselink_frame_transform=operator2mano,
    )
    left_dex = DexHandRetargeter(left_dex_cfg, name="left_hand")
    connected_left_dex = left_dex.connect(
        {HandsSource.LEFT: hands.output(HandsSource.LEFT)}
    )

    right_dex_cfg = DexHandRetargeterConfig(
        hand_retargeting_config=right_yaml_path,
        hand_urdf=right_hand_urdf,
        hand_joint_names=right_hand_joint_names,
        hand_side="right",
        handtracking_to_baselink_frame_transform=operator2mano,
    )
    right_dex = DexHandRetargeter(right_dex_cfg, name="right_hand")
    connected_right_dex = right_dex.connect(
        {HandsSource.RIGHT: hands.output(HandsSource.RIGHT)}
    )

    # -------------------------------------------------------------------------
    # TensorReorderer: flatten into a 58D action tensor
    # [left_wrist(7), right_wrist(7), hand_joints(44)]
    # Hand joints are ordered to match the PhysX BFS articulation traversal
    # so that find_joints(preserve_order=False) returns matching indices.
    # -------------------------------------------------------------------------
    left_ee_elements = ["l_pos_x", "l_pos_y", "l_pos_z", "l_quat_x", "l_quat_y", "l_quat_z", "l_quat_w"]
    right_ee_elements = ["r_pos_x", "r_pos_y", "r_pos_z", "r_quat_x", "r_quat_y", "r_quat_z", "r_quat_w"]

    from isaaclab_arena.embodiments.h2.h2 import H2_SHARPA_HAND_JOINT_NAMES_ARTICULATION_ORDER

    output_order = left_ee_elements + right_ee_elements + H2_SHARPA_HAND_JOINT_NAMES_ARTICULATION_ORDER

    reorderer = TensorReorderer(
        input_config={
            "left_ee_pose": left_ee_elements,
            "right_ee_pose": right_ee_elements,
            "left_hand_joints": left_hand_joint_names,
            "right_hand_joints": right_hand_joint_names,
        },
        output_order=output_order,
        name="action_reorderer",
        input_types={
            "left_ee_pose": "array",
            "right_ee_pose": "array",
            "left_hand_joints": "scalar",
            "right_hand_joints": "scalar",
        },
    )
    connected_reorderer = reorderer.connect(
        {
            "left_ee_pose": connected_left_se3.output("ee_pose"),
            "right_ee_pose": connected_right_se3.output("ee_pose"),
            "left_hand_joints": connected_left_dex.output("hand_joints"),
            "right_hand_joints": connected_right_dex.output("hand_joints"),
        }
    )

    return OutputCombiner({"action": connected_reorderer.output("output")})
