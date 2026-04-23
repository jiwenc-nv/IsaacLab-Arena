# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

from dataclasses import dataclass, field

import isaaclab_arena_h2.h2_env.h2_constants as h2_constants


# TODO: H2SupplementalInfo duplicates the G1SupplementalInfo structure. Both
# should inherit from a shared RobotSupplementalInfo base class/protocol once
# RobotModel is refactored into a shared package.
@dataclass
class H2SupplementalInfo:
    """Supplemental information for the Unitree H2 robot with Dex3-1 hands.

    The H2 body has 31 actuated DOFs: 3 waist, 2 head, 14 arm (7 per side), 12 leg (6 per side).
    With Dex3-1 hands grafted from the G1: 7 hand DOFs per side (thumb x3, index x2, middle x2).
    Total: 31 body + 14 hand = 45 actuated DOFs.
    """

    body_actuated_joints: list[str] = field(
        default_factory=lambda: [
            # Left leg
            "left_hip_pitch_joint",
            "left_hip_roll_joint",
            "left_hip_yaw_joint",
            "left_knee_joint",
            "left_ankle_roll_joint",
            "left_ankle_pitch_joint",
            # Right leg
            "right_hip_pitch_joint",
            "right_hip_roll_joint",
            "right_hip_yaw_joint",
            "right_knee_joint",
            "right_ankle_roll_joint",
            "right_ankle_pitch_joint",
            # Waist
            "waist_yaw_joint",
            "waist_roll_joint",
            "waist_pitch_joint",
            # Head
            "head_pitch_joint",
            "head_yaw_joint",
            # Left arm
            "left_shoulder_pitch_joint",
            "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint",
            "left_elbow_joint",
            "left_wrist_roll_joint",
            "left_wrist_pitch_joint",
            "left_wrist_yaw_joint",
            # Right arm
            "right_shoulder_pitch_joint",
            "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint",
            "right_elbow_joint",
            "right_wrist_roll_joint",
            "right_wrist_pitch_joint",
            "right_wrist_yaw_joint",
        ]
    )

    left_hand_actuated_joints: list[str] = field(
        default_factory=lambda: [
            "left_hand_thumb_0_joint",
            "left_hand_thumb_1_joint",
            "left_hand_thumb_2_joint",
            "left_hand_index_0_joint",
            "left_hand_index_1_joint",
            "left_hand_middle_0_joint",
            "left_hand_middle_1_joint",
        ]
    )

    right_hand_actuated_joints: list[str] = field(
        default_factory=lambda: [
            "right_hand_thumb_0_joint",
            "right_hand_thumb_1_joint",
            "right_hand_thumb_2_joint",
            "right_hand_index_0_joint",
            "right_hand_index_1_joint",
            "right_hand_middle_0_joint",
            "right_hand_middle_1_joint",
        ]
    )

    joint_limits: dict[str, list[float]] = field(
        default_factory=lambda: {
            # Waist
            "waist_yaw_joint": h2_constants.H2_WAIST_YAW_LIMITS,
            "waist_roll_joint": h2_constants.H2_WAIST_ROLL_LIMITS,
            "waist_pitch_joint": h2_constants.H2_WAIST_PITCH_LIMITS,
            # Head
            "head_pitch_joint": h2_constants.H2_HEAD_PITCH_LIMITS,
            "head_yaw_joint": h2_constants.H2_HEAD_YAW_LIMITS,
            # Left arm
            "left_shoulder_pitch_joint": h2_constants.H2_LEFT_SHOULDER_PITCH_LIMITS,
            "left_shoulder_roll_joint": h2_constants.H2_LEFT_SHOULDER_ROLL_LIMITS,
            "left_shoulder_yaw_joint": h2_constants.H2_LEFT_SHOULDER_YAW_LIMITS,
            "left_elbow_joint": h2_constants.H2_LEFT_ELBOW_LIMITS,
            "left_wrist_roll_joint": h2_constants.H2_LEFT_WRIST_ROLL_LIMITS,
            "left_wrist_pitch_joint": h2_constants.H2_LEFT_WRIST_PITCH_LIMITS,
            "left_wrist_yaw_joint": h2_constants.H2_LEFT_WRIST_YAW_LIMITS,
            # Right arm
            "right_shoulder_pitch_joint": h2_constants.H2_RIGHT_SHOULDER_PITCH_LIMITS,
            "right_shoulder_roll_joint": h2_constants.H2_RIGHT_SHOULDER_ROLL_LIMITS,
            "right_shoulder_yaw_joint": h2_constants.H2_RIGHT_SHOULDER_YAW_LIMITS,
            "right_elbow_joint": h2_constants.H2_RIGHT_ELBOW_LIMITS,
            "right_wrist_roll_joint": h2_constants.H2_RIGHT_WRIST_ROLL_LIMITS,
            "right_wrist_pitch_joint": h2_constants.H2_RIGHT_WRIST_PITCH_LIMITS,
            "right_wrist_yaw_joint": h2_constants.H2_RIGHT_WRIST_YAW_LIMITS,
            # Left leg
            "left_hip_pitch_joint": h2_constants.H2_LEFT_HIP_PITCH_LIMITS,
            "left_hip_roll_joint": h2_constants.H2_LEFT_HIP_ROLL_LIMITS,
            "left_hip_yaw_joint": h2_constants.H2_LEFT_HIP_YAW_LIMITS,
            "left_knee_joint": h2_constants.H2_LEFT_KNEE_LIMITS,
            "left_ankle_roll_joint": h2_constants.H2_LEFT_ANKLE_ROLL_LIMITS,
            "left_ankle_pitch_joint": h2_constants.H2_LEFT_ANKLE_PITCH_LIMITS,
            # Right leg
            "right_hip_pitch_joint": h2_constants.H2_RIGHT_HIP_PITCH_LIMITS,
            "right_hip_roll_joint": h2_constants.H2_RIGHT_HIP_ROLL_LIMITS,
            "right_hip_yaw_joint": h2_constants.H2_RIGHT_HIP_YAW_LIMITS,
            "right_knee_joint": h2_constants.H2_RIGHT_KNEE_LIMITS,
            "right_ankle_roll_joint": h2_constants.H2_RIGHT_ANKLE_ROLL_LIMITS,
            "right_ankle_pitch_joint": h2_constants.H2_RIGHT_ANKLE_PITCH_LIMITS,
            # Left hand (Dex3-1)
            "left_hand_thumb_0_joint": h2_constants.H2_LEFT_HAND_THUMB_0_LIMITS,
            "left_hand_thumb_1_joint": h2_constants.H2_LEFT_HAND_THUMB_1_LIMITS,
            "left_hand_thumb_2_joint": h2_constants.H2_LEFT_HAND_THUMB_2_LIMITS,
            "left_hand_index_0_joint": h2_constants.H2_LEFT_HAND_INDEX_0_LIMITS,
            "left_hand_index_1_joint": h2_constants.H2_LEFT_HAND_INDEX_1_LIMITS,
            "left_hand_middle_0_joint": h2_constants.H2_LEFT_HAND_MIDDLE_0_LIMITS,
            "left_hand_middle_1_joint": h2_constants.H2_LEFT_HAND_MIDDLE_1_LIMITS,
            # Right hand (Dex3-1)
            "right_hand_thumb_0_joint": h2_constants.H2_RIGHT_HAND_THUMB_0_LIMITS,
            "right_hand_thumb_1_joint": h2_constants.H2_RIGHT_HAND_THUMB_1_LIMITS,
            "right_hand_thumb_2_joint": h2_constants.H2_RIGHT_HAND_THUMB_2_LIMITS,
            "right_hand_index_0_joint": h2_constants.H2_RIGHT_HAND_INDEX_0_LIMITS,
            "right_hand_index_1_joint": h2_constants.H2_RIGHT_HAND_INDEX_1_LIMITS,
            "right_hand_middle_0_joint": h2_constants.H2_RIGHT_HAND_MIDDLE_0_LIMITS,
            "right_hand_middle_1_joint": h2_constants.H2_RIGHT_HAND_MIDDLE_1_LIMITS,
        }
    )

    joint_groups: dict[str, dict[str, list[str]]] = field(
        default_factory=lambda: {
            "waist": {
                "joints": ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"],
                "groups": [],
            },
            "head": {
                "joints": ["head_pitch_joint", "head_yaw_joint"],
                "groups": [],
            },
            # Leg groups
            "left_leg": {
                "joints": [
                    "left_hip_pitch_joint",
                    "left_hip_roll_joint",
                    "left_hip_yaw_joint",
                    "left_knee_joint",
                    "left_ankle_roll_joint",
                    "left_ankle_pitch_joint",
                ],
                "groups": [],
            },
            "right_leg": {
                "joints": [
                    "right_hip_pitch_joint",
                    "right_hip_roll_joint",
                    "right_hip_yaw_joint",
                    "right_knee_joint",
                    "right_ankle_roll_joint",
                    "right_ankle_pitch_joint",
                ],
                "groups": [],
            },
            "legs": {"joints": [], "groups": ["left_leg", "right_leg"]},
            # Arm groups
            "left_arm": {
                "joints": [
                    "left_shoulder_pitch_joint",
                    "left_shoulder_roll_joint",
                    "left_shoulder_yaw_joint",
                    "left_elbow_joint",
                    "left_wrist_roll_joint",
                    "left_wrist_pitch_joint",
                    "left_wrist_yaw_joint",
                ],
                "groups": [],
            },
            "right_arm": {
                "joints": [
                    "right_shoulder_pitch_joint",
                    "right_shoulder_roll_joint",
                    "right_shoulder_yaw_joint",
                    "right_elbow_joint",
                    "right_wrist_roll_joint",
                    "right_wrist_pitch_joint",
                    "right_wrist_yaw_joint",
                ],
                "groups": [],
            },
            "arms": {"joints": [], "groups": ["left_arm", "right_arm"]},
            # Hand groups (Dex3-1)
            "left_hand": {
                "joints": [
                    "left_hand_thumb_0_joint",
                    "left_hand_thumb_1_joint",
                    "left_hand_thumb_2_joint",
                    "left_hand_index_0_joint",
                    "left_hand_index_1_joint",
                    "left_hand_middle_0_joint",
                    "left_hand_middle_1_joint",
                ],
                "groups": [],
            },
            "right_hand": {
                "joints": [
                    "right_hand_thumb_0_joint",
                    "right_hand_thumb_1_joint",
                    "right_hand_thumb_2_joint",
                    "right_hand_index_0_joint",
                    "right_hand_index_1_joint",
                    "right_hand_middle_0_joint",
                    "right_hand_middle_1_joint",
                ],
                "groups": [],
            },
            "hands": {"joints": [], "groups": ["left_hand", "right_hand"]},
            # Composite groups
            "lower_body": {"joints": [], "groups": ["waist", "legs"]},
            "upper_body_no_hands": {"joints": [], "groups": ["arms"]},
            "upper_body": {"joints": [], "groups": ["upper_body_no_hands", "hands"]},
            "body": {"joints": [], "groups": ["lower_body", "upper_body_no_hands"]},
        }
    )

    joint_name_mapping: dict[str, dict[str, str]] = field(
        default_factory=lambda: {
            "waist_pitch": "waist_pitch_joint",
            "waist_roll": "waist_roll_joint",
            "waist_yaw": "waist_yaw_joint",
            "shoulder_pitch": {
                "left": "left_shoulder_pitch_joint",
                "right": "right_shoulder_pitch_joint",
            },
            "shoulder_roll": {
                "left": "left_shoulder_roll_joint",
                "right": "right_shoulder_roll_joint",
            },
            "shoulder_yaw": {
                "left": "left_shoulder_yaw_joint",
                "right": "right_shoulder_yaw_joint",
            },
            "elbow_pitch": {"left": "left_elbow_joint", "right": "right_elbow_joint"},
            "wrist_pitch": {"left": "left_wrist_pitch_joint", "right": "right_wrist_pitch_joint"},
            "wrist_roll": {"left": "left_wrist_roll_joint", "right": "right_wrist_roll_joint"},
            "wrist_yaw": {"left": "left_wrist_yaw_joint", "right": "right_wrist_yaw_joint"},
        }
    )

    root_frame_name: str = "pelvis"
    hand_frame_names: dict[str, str] = field(
        default_factory=lambda: {"left": "left_wrist_yaw_link", "right": "right_wrist_yaw_link"}
    )
    default_joint_q: dict[str, float] = field(default_factory=lambda: {})
    elbow_calibration_joint_angles = {"left": 0.0, "right": 0.0}
