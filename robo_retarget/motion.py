# Copyright (c) 2026 Jihwan Shin
# SPDX-License-Identifier: MIT
"""Robot motions: a qpos trajectory in the robot's MJCF order, plus its frame rate.

Two file formats, chosen by extension:

``.csv``  one row per frame: root ``x y z``, root quaternion ``qx qy qz qw``, then
          joint angles.  This is the Unitree LAFAN1 / Booster motion layout.  The
          joint order comes from the header row when there is one, else from the
          robot's ``robot.json`` (``csv.joints``).  The frame rate is not stored in
          the file, so it comes from ``csv.fps`` unless given.
``.pkl``  GMR's layout: ``fps``, ``root_pos``, ``root_rot`` (xyzw), ``dof_pos`` in
          MJCF order, plus ``joint_names`` and provenance keys when written here.
"""

from __future__ import annotations

import pickle
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .robots import Robot


@dataclass
class Motion:
    robot: str
    fps: float
    qpos: np.ndarray  # (T, nq): root pos, root quat wxyz, joints in MJCF order
    meta: dict = field(default_factory=dict)

    def __len__(self) -> int:
        return len(self.qpos)

    def slice(self, start: int = 0, frames: int | None = None) -> Motion:
        end = None if frames is None else start + frames
        return Motion(self.robot, self.fps, self.qpos[start:end],
                      {**self.meta, "start_frame": start})


def load_motion(path: str | Path, robot: Robot, fps: float | None = None) -> Motion:
    path = Path(path)
    if path.suffix == ".pkl":
        with open(path, "rb") as f:
            d = pickle.load(f)
        qpos = np.concatenate([d["root_pos"], np.asarray(d["root_rot"])[:, [3, 0, 1, 2]],
                               d["dof_pos"]], axis=1)
        return Motion(robot.name, fps or float(d["fps"]), qpos, {"source_file": str(path)})

    first = path.open().readline().strip().split(",")
    has_header = not _is_number(first[0])
    raw = np.loadtxt(path, delimiter=",", skiprows=int(has_header), ndmin=2)
    joints = first[7:] if has_header else robot.spec["csv"]["joints"]
    if raw.shape[1] != 7 + len(joints):
        raise ValueError(f"{path}: {raw.shape[1]} columns, expected 7 + {len(joints)} joints")
    qpos = np.tile(robot.model.qpos0, (len(raw), 1))
    qpos[:, :3] = raw[:, :3]
    qpos[:, 3:7] = raw[:, [6, 3, 4, 5]]  # xyzw -> wxyz
    for k, joint in enumerate(joints):
        qpos[:, robot.qpos_index(joint)] = raw[:, 7 + k]
    return Motion(robot.name, fps or float(robot.spec["csv"]["fps"]), qpos,
                  {"source_file": str(path)})


def save_motion(path: str | Path, motion: Motion, robot: Robot) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    q = motion.qpos
    root_rot = q[:, [4, 5, 6, 3]]  # wxyz -> xyzw
    if path.suffix == ".csv":
        header = ["root_pos_x", "root_pos_y", "root_pos_z",
                  "root_quat_x", "root_quat_y", "root_quat_z", "root_quat_w", *robot.joint_names]
        np.savetxt(path, np.concatenate([q[:, :3], root_rot, q[:, 7:]], axis=1),
                   delimiter=",", header=",".join(header), comments="", fmt="%.6f")
        return
    with open(path, "wb") as f:
        pickle.dump({"fps": motion.fps, "robot": motion.robot, "joint_names": robot.joint_names,
                     "root_pos": q[:, :3], "root_rot": root_rot, "dof_pos": q[:, 7:],
                     **motion.meta}, f)


def _is_number(s: str) -> bool:
    try:
        float(s)
        return True
    except ValueError:
        return False
