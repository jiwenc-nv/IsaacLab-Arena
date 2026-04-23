# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Custom IsaacTeleop pipeline for H2 PinkIK with Dex3-1 hands: 16D output.

Output layout (16D):
    [left_gripper(1), right_gripper(1),
     left_wrist_pos(3), left_wrist_quat(4),
     right_wrist_pos(3), right_wrist_quat(4)]

The gripper scalars come from TriHandMotionControllerRetargeter (thumb_rotation channel),
mapped from the Quest controller trigger/squeeze. Wrist SE3 from Se3AbsRetargeter.
No locomotion (lower body is fixed).
"""


def _build_h2_pink_pipeline():
    """Build an IsaacTeleop retargeting pipeline for H2 PinkIK with Dex3-1 hands.

    Returns:
        OutputCombiner with a single "action" output (16D flattened tensor).
    """
    from isaacteleop.retargeters import (
        Se3AbsRetargeter,
        Se3RetargeterConfig,
        TensorReorderer,
        TriHandMotionControllerConfig,
        TriHandMotionControllerRetargeter,
    )
    from isaacteleop.retargeting_engine.deviceio_source_nodes import ControllersSource
    from isaacteleop.retargeting_engine.interface import OutputCombiner, ValueInput
    from isaacteleop.retargeting_engine.tensor_types import TransformMatrix

    controllers = ControllersSource(name="controllers")
    transform_input = ValueInput("world_T_anchor", TransformMatrix())
    transformed_controllers = controllers.transformed(transform_input.output(ValueInput.VALUE))

    # SE3 Absolute Pose Retargeters (left and right wrists)
    # TODO: Rotation offsets are copied from G1 and need calibration for the H2's
    # wrist frame conventions. Incorrect offsets cause mirrored/rotated EE targets.
    left_se3_cfg = Se3RetargeterConfig(
        input_device=ControllersSource.LEFT,
        zero_out_xy_rotation=False,
        use_wrist_rotation=False,
        use_wrist_position=False,
        target_offset_roll=45.0,
        target_offset_pitch=180.0,
        target_offset_yaw=-90.0,
    )
    left_se3 = Se3AbsRetargeter(left_se3_cfg, name="left_ee_pose")
    connected_left_se3 = left_se3.connect(
        {ControllersSource.LEFT: transformed_controllers.output(ControllersSource.LEFT)}
    )

    right_se3_cfg = Se3RetargeterConfig(
        input_device=ControllersSource.RIGHT,
        zero_out_xy_rotation=False,
        use_wrist_rotation=False,
        use_wrist_position=False,
        target_offset_roll=-135.0,
        target_offset_pitch=0.0,
        target_offset_yaw=90.0,
    )
    right_se3 = Se3AbsRetargeter(right_se3_cfg, name="right_ee_pose")
    connected_right_se3 = right_se3.connect(
        {ControllersSource.RIGHT: transformed_controllers.output(ControllersSource.RIGHT)}
    )

    # TriHand retargeters for binary gripper from trigger/squeeze
    hand_joint_names = [
        "thumb_rotation",
        "thumb_proximal",
        "thumb_distal",
        "index_proximal",
        "index_distal",
        "middle_proximal",
        "middle_distal",
    ]
    left_trihand_cfg = TriHandMotionControllerConfig(
        hand_joint_names=hand_joint_names,
        controller_side="left",
    )
    left_trihand = TriHandMotionControllerRetargeter(left_trihand_cfg, name="trihand_left")
    connected_left_trihand = left_trihand.connect(
        {ControllersSource.LEFT: transformed_controllers.output(ControllersSource.LEFT)}
    )

    right_trihand_cfg = TriHandMotionControllerConfig(
        hand_joint_names=hand_joint_names,
        controller_side="right",
    )
    right_trihand = TriHandMotionControllerRetargeter(right_trihand_cfg, name="trihand_right")
    connected_right_trihand = right_trihand.connect(
        {ControllersSource.RIGHT: transformed_controllers.output(ControllersSource.RIGHT)}
    )

    # TensorReorderer: 16D = gripper(1+1) + left_wrist(7) + right_wrist(7)
    left_ee_elements = ["l_pos_x", "l_pos_y", "l_pos_z", "l_quat_x", "l_quat_y", "l_quat_z", "l_quat_w"]
    right_ee_elements = ["r_pos_x", "r_pos_y", "r_pos_z", "r_quat_x", "r_quat_y", "r_quat_z", "r_quat_w"]
    left_hand_elements = [
        "l_thumb_rotation",
        "l_thumb_proximal",
        "l_thumb_distal",
        "l_index_proximal",
        "l_index_distal",
        "l_middle_proximal",
        "l_middle_distal",
    ]
    right_hand_elements = [
        "r_thumb_rotation",
        "r_thumb_proximal",
        "r_thumb_distal",
        "r_index_proximal",
        "r_index_distal",
        "r_middle_proximal",
        "r_middle_distal",
    ]

    output_order = (
        ["l_thumb_rotation", "r_thumb_rotation"]
        + left_ee_elements
        + right_ee_elements
    )

    reorderer = TensorReorderer(
        input_config={
            "left_ee_pose": left_ee_elements,
            "right_ee_pose": right_ee_elements,
            "left_hand_joints": left_hand_elements,
            "right_hand_joints": right_hand_elements,
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
    connected_reorderer = reorderer.connect({
        "left_ee_pose": connected_left_se3.output("ee_pose"),
        "right_ee_pose": connected_right_se3.output("ee_pose"),
        "left_hand_joints": connected_left_trihand.output("hand_joints"),
        "right_hand_joints": connected_right_trihand.output("hand_joints"),
    })

    return OutputCombiner({"action": connected_reorderer.output("output")})
