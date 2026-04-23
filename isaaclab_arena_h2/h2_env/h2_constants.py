# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Constants for H2 robot joint limits (from URDF)."""

# H2 Joint Limit Values (from URDF)
# Format: [min_limit, max_limit]

# Waist limits
H2_WAIST_YAW_LIMITS: list[float] = [-1.7453, 1.7453]
H2_WAIST_ROLL_LIMITS: list[float] = [-0.5236, 0.5236]
H2_WAIST_PITCH_LIMITS: list[float] = [-0.43633, 0.5236]

# Head limits
H2_HEAD_PITCH_LIMITS: list[float] = [-0.5236, 0.83775]
H2_HEAD_YAW_LIMITS: list[float] = [-1.7453, 1.7453]

# Right arm limits
H2_RIGHT_SHOULDER_PITCH_LIMITS: list[float] = [-2.6179939, 1.8325957]
H2_RIGHT_SHOULDER_ROLL_LIMITS: list[float] = [-2.6354472, 0.5166175]
H2_RIGHT_SHOULDER_YAW_LIMITS: list[float] = [-2.6179939, 2.6179939]
H2_RIGHT_ELBOW_LIMITS: list[float] = [-0.986111, 3.0717795]
H2_RIGHT_WRIST_ROLL_LIMITS: list[float] = [-2.6179939, 2.6179939]
H2_RIGHT_WRIST_PITCH_LIMITS: list[float] = [-0.4363323, 0.4363323]
H2_RIGHT_WRIST_YAW_LIMITS: list[float] = [-1.2217305, 1.2217305]

# Left arm limits
H2_LEFT_SHOULDER_PITCH_LIMITS: list[float] = [-2.6179939, 1.8325957]
H2_LEFT_SHOULDER_ROLL_LIMITS: list[float] = [-0.5166175, 2.6354472]
H2_LEFT_SHOULDER_YAW_LIMITS: list[float] = [-2.6179939, 2.6179939]
H2_LEFT_ELBOW_LIMITS: list[float] = [-0.986111, 3.0717795]
H2_LEFT_WRIST_ROLL_LIMITS: list[float] = [-2.6179939, 2.6179939]
H2_LEFT_WRIST_PITCH_LIMITS: list[float] = [-0.4363323, 0.4363323]
H2_LEFT_WRIST_YAW_LIMITS: list[float] = [-1.2217305, 1.2217305]

# Left leg limits
H2_LEFT_HIP_PITCH_LIMITS: list[float] = [-2.4525975, 2.7754225]
H2_LEFT_HIP_ROLL_LIMITS: list[float] = [-0.467441, 2.1688605]
H2_LEFT_HIP_YAW_LIMITS: list[float] = [-2.827, 2.827]
H2_LEFT_KNEE_LIMITS: list[float] = [-0.08725, 2.53025]
H2_LEFT_ANKLE_ROLL_LIMITS: list[float] = [-0.3490658503, 0.296705972783]
H2_LEFT_ANKLE_PITCH_LIMITS: list[float] = [-1.13446401358, 0.61086523808]

# Left hand limits (Dex3-1, same as G1)
H2_LEFT_HAND_THUMB_0_LIMITS: list[float] = [-1.04719755, 1.04719755]
H2_LEFT_HAND_THUMB_1_LIMITS: list[float] = [-0.72431163, 1.04719755]
H2_LEFT_HAND_THUMB_2_LIMITS: list[float] = [0, 1.74532925]
H2_LEFT_HAND_INDEX_0_LIMITS: list[float] = [-1.57079632, 0]
H2_LEFT_HAND_INDEX_1_LIMITS: list[float] = [-1.74532925, 0]
H2_LEFT_HAND_MIDDLE_0_LIMITS: list[float] = [-1.57079632, 0]
H2_LEFT_HAND_MIDDLE_1_LIMITS: list[float] = [-1.74532925, 0]

# Right hand limits (Dex3-1, same as G1)
H2_RIGHT_HAND_THUMB_0_LIMITS: list[float] = [-1.04719755, 1.04719755]
H2_RIGHT_HAND_THUMB_1_LIMITS: list[float] = [-1.04719755, 0.72431163]
H2_RIGHT_HAND_THUMB_2_LIMITS: list[float] = [-1.74532925, 0]
H2_RIGHT_HAND_INDEX_0_LIMITS: list[float] = [0, 1.57079632]
H2_RIGHT_HAND_INDEX_1_LIMITS: list[float] = [0, 1.74532925]
H2_RIGHT_HAND_MIDDLE_0_LIMITS: list[float] = [0, 1.57079632]
H2_RIGHT_HAND_MIDDLE_1_LIMITS: list[float] = [0, 1.74532925]

# Right leg limits
H2_RIGHT_HIP_PITCH_LIMITS: list[float] = [-2.4525975, 2.7754225]
H2_RIGHT_HIP_ROLL_LIMITS: list[float] = [-2.1688605, 0.467441]
H2_RIGHT_HIP_YAW_LIMITS: list[float] = [-2.827, 2.827]
H2_RIGHT_KNEE_LIMITS: list[float] = [-0.08725, 2.53025]
H2_RIGHT_ANKLE_ROLL_LIMITS: list[float] = [-0.296705972783, 0.3490658503]
H2_RIGHT_ANKLE_PITCH_LIMITS: list[float] = [-1.13446401358, 0.61086523808]
