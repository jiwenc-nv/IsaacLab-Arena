# Copyright (c) 2026, The Isaac Lab Arena Project Developers (https://github.com/isaac-sim/IsaacLab-Arena/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Debug visualizer for hand (wrist) poses emitted by an IsaacTeleop session.

Drops a pair of triaxis frame markers into the viewport that follow the
left and right wrist poses returned by ``IsaacTeleopDevice.advance()``
(equivalently, ``TeleopSession.step()``).  Intended for verifying that
the hand-tracking -> retargeting -> action-tensor chain is producing the
poses you expect, prior to feeding them into Pink IK.

Default slices match the H2 PinkIK + Sharpa Wave pipeline layout::

    [left_pos(3), left_quat_xyzw(4), right_pos(3), right_quat_xyzw(4), ...]

For other pipelines, pass alternative ``left_pose_slice`` /
``right_pose_slice`` and ``quat_order`` to :meth:`update`.
"""

from __future__ import annotations

import logging

import numpy as np
import torch

logger = logging.getLogger(__name__)


class TeleopHandPoseVisualizer:
    """Frame-marker debug visualizer for IsaacTeleop wrist poses.

    Spawns one :class:`~isaaclab.markers.VisualizationMarkers` instance
    with two frame prototypes (one per hand).  Call :meth:`update` each
    sim tick with the flat action tensor returned by
    :meth:`~isaaclab_teleop.IsaacTeleopDevice.advance`.

    Frame rebasing:
        If the active pipeline rebases output poses into a non-world
        frame (i.e. :attr:`~isaaclab_teleop.IsaacTeleopCfg.target_frame_prim_path`
        is set), pass the parent prim's world transform as
        ``parent_T_world`` to :meth:`update` so the markers land in the
        world frame.  When the pipeline emits world-frame poses (the
        default for H2), leave ``parent_T_world=None``.

    Example::

        viz = TeleopHandPoseVisualizer()
        while running:
            action = teleop.advance()
            if action is not None:
                viz.update(action)
                env.step(action.repeat(env.num_envs, 1))
    """

    LEFT_MARKER_INDEX = 0
    RIGHT_MARKER_INDEX = 1

    def __init__(
        self,
        prim_path: str = "/Visuals/TeleopHandPoses",
        marker_scale: tuple[float, float, float] = (0.1, 0.1, 0.1),
    ) -> None:
        """Create the marker prims and register them with Isaac Sim.

        Args:
            prim_path: USD path under which the ``PointInstancer`` is
                created.  If a prim already exists at this path, a free
                sibling path is used.
            marker_scale: ``(sx, sy, sz)`` applied to the frame prototype
                USD.  The shipped frame prim has unit length axes; smaller
                values keep the marker readable next to a robot hand.
        """
        from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
        from isaaclab.markers.config import FRAME_MARKER_CFG

        frame_proto = FRAME_MARKER_CFG.markers["frame"].copy()
        frame_proto.scale = marker_scale

        cfg = VisualizationMarkersCfg(
            prim_path=prim_path,
            markers={"left_hand": frame_proto, "right_hand": frame_proto},
        )
        self._markers = VisualizationMarkers(cfg)
        self._marker_indices = np.array(
            [self.LEFT_MARKER_INDEX, self.RIGHT_MARKER_INDEX], dtype=np.int32
        )

    def set_visibility(self, visible: bool) -> None:
        """Show or hide the frame markers."""
        self._markers.set_visibility(visible)

    def update(
        self,
        action: torch.Tensor,
        *,
        left_pose_slice: slice = slice(0, 7),
        right_pose_slice: slice = slice(7, 14),
        quat_order: str = "xyzw",
        parent_T_world: np.ndarray | torch.Tensor | None = None,
    ) -> None:
        """Update marker poses from a teleop action tensor.

        Args:
            action: Flat action tensor produced by the teleop pipeline.
                Either 1D ``(D,)`` or 2D ``(N, D)`` -- when 2D, the first
                row is used (the pipeline emits the same action for every
                env).
            left_pose_slice: Slice into ``action`` for the left wrist
                pose, length 7: ``[x, y, z, q*, q*, q*, q*]``.  Defaults
                match the H2 pipeline.
            right_pose_slice: Slice into ``action`` for the right wrist
                pose, length 7.  Defaults match the H2 pipeline.
            quat_order: ``"xyzw"`` (H2 pipeline default) or ``"wxyz"``.
                Internally converted to ``xyzw`` for the marker, matching
                the convention used by Isaac Lab math utilities and
                :meth:`isaaclab.markers.VisualizationMarkers.visualize`.
            parent_T_world: Optional ``(4, 4)`` world transform of the
                frame in which ``action`` is expressed.  When provided,
                markers are rebased into world via
                ``world_pose = parent_T_world @ action_pose`` so they
                visualize correctly in the viewport.  Leave ``None`` when
                the pipeline already emits world-frame poses.
        """
        if action.dim() == 2:
            action_1d = action[0]
        else:
            action_1d = action

        left_pose = action_1d[left_pose_slice].detach().cpu().numpy().astype(np.float32)
        right_pose = action_1d[right_pose_slice].detach().cpu().numpy().astype(np.float32)
        if left_pose.size != 7 or right_pose.size != 7:
            raise ValueError(
                f"Expected 7-D wrist poses, got left={left_pose.size}, right={right_pose.size}. "
                "Override left_pose_slice / right_pose_slice for non-H2 pipelines."
            )

        positions = np.stack([left_pose[:3], right_pose[:3]], axis=0)
        quats_in = np.stack([left_pose[3:7], right_pose[3:7]], axis=0)

        if quat_order == "xyzw":
            quats_xyzw = quats_in
        elif quat_order == "wxyz":
            quats_xyzw = np.roll(quats_in, shift=-1, axis=1)
        else:
            raise ValueError(f"quat_order must be 'xyzw' or 'wxyz', got {quat_order!r}")

        if parent_T_world is not None:
            T = self._as_4x4_numpy(parent_T_world)
            positions, quats_xyzw = self._rebase_poses(positions, quats_xyzw, T)

        self._markers.visualize(
            translations=positions,
            orientations=quats_xyzw,
            marker_indices=self._marker_indices,
        )

    @staticmethod
    def _as_4x4_numpy(mat: np.ndarray | torch.Tensor) -> np.ndarray:
        if isinstance(mat, torch.Tensor):
            mat = mat.detach().cpu().numpy()
        mat = np.asarray(mat, dtype=np.float32)
        if mat.shape != (4, 4):
            raise ValueError(f"parent_T_world must be (4, 4), got {mat.shape}")
        return mat

    @staticmethod
    def _rebase_poses(
        positions: np.ndarray,
        quats_xyzw: np.ndarray,
        parent_T_world: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Apply ``parent_T_world`` to each (pos, quat_xyzw) pair."""
        from scipy.spatial.transform import Rotation

        R_parent = parent_T_world[:3, :3]
        t_parent = parent_T_world[:3, 3]

        out_pos = positions @ R_parent.T + t_parent

        R_local = Rotation.from_quat(quats_xyzw).as_matrix()
        R_world = R_parent @ R_local
        out_quats_xyzw = Rotation.from_matrix(R_world).as_quat().astype(np.float32)

        return out_pos.astype(np.float32), out_quats_xyzw
