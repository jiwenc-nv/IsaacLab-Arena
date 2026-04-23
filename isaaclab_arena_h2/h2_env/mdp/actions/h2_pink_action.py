# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""H2 PinkIK action term with Dex3-1 hand support.

Upper body arms are controlled via PinkIK from wrist SE3 targets.
Hands are driven by binary open/close gripper commands mapped to 7-DOF finger poses.
Lower body (legs + waist) is held at fixed joint positions -- no WBC policy.

Action layout (16D):
    [left_gripper(1), right_gripper(1),
     left_wrist_pos(3), left_wrist_quat(4),
     right_wrist_pos(3), right_wrist_quat(4)]
"""

from __future__ import annotations

import numpy as np
import torch
from collections.abc import Sequence
from scipy.spatial.transform import Rotation as R
from typing import TYPE_CHECKING

from isaaclab.assets.articulation import Articulation
from isaaclab.managers.action_manager import ActionTerm

# TODO: These imports reach into isaaclab_arena_g1 for classes that are actually
# robot-agnostic. Once RobotModel, ReducedRobotModel, and the IK solver are
# refactored into a shared package, update these imports.
from isaaclab_arena_g1.g1_env.robot_model import ReducedRobotModel
from isaaclab_arena_g1.g1_whole_body_controller.wbc_policy.g1_wbc_upperbody_ik.g1_wbc_upperbody_controller import (
    G1BodyIKSolver,
    G1BodyIKSolverSettings,
)
from isaaclab_arena_h2.h2_env.robot_model_utils import instantiate_h2_robot_model

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

    from isaaclab_arena_h2.h2_env.mdp.actions.h2_pink_action_cfg import H2PinkActionCfg

# Indices into the 16D action vector
LEFT_HAND_STATE_IDX = 0
RIGHT_HAND_STATE_IDX = 1
LEFT_WRIST_POS_START = 2
LEFT_WRIST_POS_END = 5
LEFT_WRIST_QUAT_START = 5
LEFT_WRIST_QUAT_END = 9
RIGHT_WRIST_POS_START = 9
RIGHT_WRIST_POS_END = 12
RIGHT_WRIST_QUAT_START = 12
RIGHT_WRIST_QUAT_END = 16

LEFT_WRIST_LINK_NAME = "left_wrist_yaw_link"
RIGHT_WRIST_LINK_NAME = "right_wrist_yaw_link"

ACTION_DIM = 16


def _get_hand_joint_pos(hand_state: float) -> np.ndarray:
    """Map a binary gripper state (0=open, 1=close) to 7-DOF Dex3-1 finger targets.

    Replicates G1WBCUpperbodyController.get_hand_joint_pos for the Dex3-1 hand.
    """
    q = np.deg2rad([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    if hand_state >= 0.5:
        amp = 0.7
        q[1] += amp
        q[2] += amp
        ampA = 0.6
        ampB = 1.2
        q[3] -= ampA
        q[4] -= ampB
        q[5] -= ampA
        q[6] -= ampB
    return q


class H2PinkAction(ActionTerm):
    """PinkIK action term for the H2 with Dex3-1 hands. No WBC -- lower body is fixed."""

    cfg: H2PinkActionCfg
    _asset: Articulation

    def __init__(self, cfg: H2PinkActionCfg, env: ManagerBasedEnv):
        super().__init__(cfg, env)

        # TODO: Support multi-env by batching IK solves (currently Pink/Pinocchio
        # runs on CPU for a single env). Requires either vectorized IK or per-env
        # solver instances.
        assert self.num_envs == 1, "H2 PinkIK controller currently only supports a single environment"

        self._raw_actions = torch.zeros(self.num_envs, ACTION_DIM, device=self.device)

        self._joint_ids, self._joint_names = self._asset.find_joints(
            self.cfg.joint_names, preserve_order=self.cfg.preserve_order
        )
        self._num_joints = len(self._joint_ids)
        if self._num_joints == self._asset.num_joints and not self.cfg.preserve_order:
            self._joint_ids = slice(None)

        self._processed_actions = torch.zeros([self.num_envs, self._num_joints], device=self.device)

        self.robot_model = instantiate_h2_robot_model()

        self.body = ReducedRobotModel.from_active_groups(self.robot_model, ["arms"])
        self.full_robot = self.body.full_robot

        ik_settings = G1BodyIKSolverSettings()
        self.ik_solver = G1BodyIKSolver(ik_settings)
        self.ik_solver.register_robot(self.body)

        self._in_warmup = True

        self._fixed_joint_values = self.robot_model.pinocchio_wrapper.q0.copy()

        # Build a mapping from Pinocchio joint order to Isaac Sim joint order.
        sim_joint_names = self._asset.data.joint_names
        pin_joint_names = list(self.robot_model.joint_to_dof_index.keys())
        self._pin_to_sim_idx = []
        for pin_name in pin_joint_names:
            assert pin_name in sim_joint_names, (
                f"Pinocchio joint '{pin_name}' not found in sim joints: {sim_joint_names}"
            )
            self._pin_to_sim_idx.append(sim_joint_names.index(pin_name))

    @property
    def action_dim(self) -> int:
        return ACTION_DIM

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    @staticmethod
    def _safe_quat(quat: np.ndarray) -> np.ndarray:
        """Return identity quaternion [0,0,0,1] when the input has zero norm.

        TODO: Initialize raw_actions with identity quaternions in the wrist slots
        so the IK solver always receives valid orientations, making this unnecessary.
        """
        if np.linalg.norm(quat) < 1e-8:
            return np.array([0.0, 0.0, 0.0, 1.0])
        return quat

    def process_actions(self, actions: torch.Tensor):
        self._raw_actions[:] = actions[:, :ACTION_DIM]
        actions_clone = actions.clone()

        # Extract binary gripper states
        left_hand_state = actions_clone[:, LEFT_HAND_STATE_IDX].squeeze(0).cpu().item()
        right_hand_state = actions_clone[:, RIGHT_HAND_STATE_IDX].squeeze(0).cpu().item()

        # Parse wrist SE3 targets
        left_pos = actions_clone[:, LEFT_WRIST_POS_START:LEFT_WRIST_POS_END].squeeze(0).cpu().numpy()
        left_quat = self._safe_quat(
            actions_clone[:, LEFT_WRIST_QUAT_START:LEFT_WRIST_QUAT_END].squeeze(0).cpu().numpy()
        )
        right_pos = actions_clone[:, RIGHT_WRIST_POS_START:RIGHT_WRIST_POS_END].squeeze(0).cpu().numpy()
        right_quat = self._safe_quat(
            actions_clone[:, RIGHT_WRIST_QUAT_START:RIGHT_WRIST_QUAT_END].squeeze(0).cpu().numpy()
        )

        left_rotmat = R.from_quat(left_quat).as_matrix()
        right_rotmat = R.from_quat(right_quat).as_matrix()

        left_pose = np.eye(4)
        left_pose[:3, :3] = left_rotmat
        left_pose[:3, 3] = left_pos

        right_pose = np.eye(4)
        right_pose[:3, :3] = right_rotmat
        right_pose[:3, 3] = right_pos

        body_targets = {LEFT_WRIST_LINK_NAME: left_pose, RIGHT_WRIST_LINK_NAME: right_pose}

        # TODO: Profile whether 50 warmup iterations are needed for H2 or if fewer suffice.
        if self._in_warmup:
            for _ in range(50):
                self.ik_solver(body_targets)
            self._in_warmup = False

        reduced_q = self.ik_solver(body_targets)

        # Expand reduced (arms-only) IK solution back to full joint vector (Pinocchio order)
        full_q_pin = self.body.reduced_to_full_configuration(reduced_q)

        # Overwrite lower-body + head joints with fixed values
        lower_body_indices = self.robot_model.get_joint_group_indices({"lower_body", "head"})
        for idx in lower_body_indices:
            full_q_pin[idx] = self._fixed_joint_values[idx]

        # Set hand joint targets from binary gripper commands
        left_hand_q = _get_hand_joint_pos(left_hand_state)
        right_hand_q = -_get_hand_joint_pos(right_hand_state)

        left_hand_indices = self.robot_model.get_hand_actuated_joint_indices(side="left")
        right_hand_indices = self.robot_model.get_hand_actuated_joint_indices(side="right")
        full_q_pin[left_hand_indices] = left_hand_q
        full_q_pin[right_hand_indices] = right_hand_q

        # Remap from Pinocchio joint order to Isaac Sim joint order
        full_q_sim = np.zeros(self._num_joints)
        for pin_idx, sim_idx in enumerate(self._pin_to_sim_idx):
            full_q_sim[sim_idx] = full_q_pin[pin_idx]

        self._processed_actions = torch.tensor(full_q_sim, dtype=torch.float32, device=self.device).unsqueeze(0)

    def apply_actions(self):
        self._asset.set_joint_position_target(self._processed_actions, self._joint_ids)

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        self._raw_actions[env_ids] = torch.zeros(ACTION_DIM, device=self.device)
        self._in_warmup = True
