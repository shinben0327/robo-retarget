# Copyright (c) 2026 Jihwan Shin
# SPDX-License-Identifier: MIT
"""Robots: an MJCF plus a ``robot.json`` spec, under ``assets/robots/<name>/``.

``robot.json`` maps the canonical keypoints (see ``retarget.KEYPOINT_WEIGHTS``) to
the robot's bodies and gives a rest pose in which every robot looks alike: upper
arms down, forearms forward, legs straight.  Adding a robot means adding such a
directory; nothing in the code names a robot.
"""

from __future__ import annotations

import json
from functools import cached_property
from pathlib import Path

import mujoco as mj
import numpy as np

ROBOT_ROOT = Path(__file__).resolve().parents[1] / "assets" / "robots"


def available_robots() -> list[str]:
    return sorted(p.parent.name for p in ROBOT_ROOT.glob("*/robot.json"))


class Robot:
    """One robot model: its spec, its MJCF, and forward kinematics on it."""

    def __init__(self, name: str):
        spec_path = ROBOT_ROOT / name / "robot.json"
        if not spec_path.is_file():
            raise ValueError(f"unknown robot {name!r}; available: {', '.join(available_robots())}")
        self.name = name
        self.spec = json.loads(spec_path.read_text())
        self.xml_path = spec_path.parent / self.spec["mjcf"]
        self.model = mj.MjModel.from_xml_path(str(self.xml_path))
        self.data = mj.MjData(self.model)
        self.keypoint_bodies: dict[str, str] = self.spec["keypoints"]

    @cached_property
    def joint_names(self) -> list[str]:
        """Actuated joints in qpos order (the free root joint excluded)."""
        return [mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_JOINT, j)
                for j in range(self.model.njnt) if self.model.jnt_type[j] != mj.mjtJoint.mjJNT_FREE]

    def qpos_index(self, joint: str) -> int:
        # Booster prefixes some MJCF joint names with "aa" (to fix their sort
        # order) but not the matching names in its motion CSVs.
        for name in (joint, "aa" + joint):
            jid = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, name)
            if jid >= 0:
                return int(self.model.jnt_qposadr[jid])
        raise KeyError(f"{self.name} has no joint {joint!r}")

    def body_poses(self, qpos) -> dict:
        """``{body: (pos, quat_wxyz)}`` in the world frame."""
        self.data.qpos[:] = qpos
        mj.mj_kinematics(self.model, self.data)
        return {mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_BODY, i): (self.data.xpos[i].copy(),
                                                                     self.data.xquat[i].copy())
                for i in range(1, self.model.nbody)}

    def keypoints(self, qpos) -> dict:
        """``{canonical keypoint: (pos, quat_wxyz)}``."""
        poses = self.body_poses(qpos)
        return {k: poses[b] for k, b in self.keypoint_bodies.items()}

    def lowest_point(self, qpos) -> float:
        """Lowest world z over all geoms; exact for meshes, boxes, spheres and capsules."""
        m, d = self.model, self.data
        d.qpos[:] = qpos
        mj.mj_forward(m, d)
        zmin = np.inf
        for g in range(m.ngeom):
            if m.geom_bodyid[g] == 0:  # the floor
                continue
            xmat, z, size = d.geom_xmat[g].reshape(3, 3), d.geom_xpos[g][2], m.geom_size[g]
            gtype = m.geom_type[g]
            if gtype == mj.mjtGeom.mjGEOM_MESH:
                v = m.mesh_vert[m.mesh_vertadr[m.geom_dataid[g]]:][:m.mesh_vertnum[m.geom_dataid[g]]]
                z += (v @ xmat[2]).min()
            elif gtype == mj.mjtGeom.mjGEOM_BOX:
                z -= np.abs(xmat[2] * size).sum()
            elif gtype == mj.mjtGeom.mjGEOM_SPHERE:
                z -= size[0]
            elif gtype == mj.mjtGeom.mjGEOM_CAPSULE:
                z -= abs(xmat[2, 2]) * size[1] + size[0]
            else:  # bounding sphere: only a fallback, it can sit well below the geom
                z -= m.geom_rbound[g]
            zmin = min(zmin, z)
        return zmin

    def rest_qpos(self) -> np.ndarray:
        """The spec's rest pose, standing on the floor at the origin, facing +x."""
        q = self.model.qpos0.copy()
        q[:7] = [0, 0, 0, 1, 0, 0, 0]
        for joint, angle in self.spec.get("rest", {}).items():
            q[self.qpos_index(joint)] = angle
        q[2] = -self.lowest_point(q)
        return q
