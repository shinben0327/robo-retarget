# Copyright (c) 2026 Jihwan Shin
# SPDX-License-Identifier: MIT
"""Robot-to-robot retargeting on GMR's IK.

The source robot's motion is run through its own forward kinematics and read out
at its canonical keypoints; GMR's IK then makes the target's keypoint bodies
track them, exactly as it would track human bodies.  The GMR IK config of a
(source, target) pair is computed, not hand-tuned.  At the two rest poses (both
robots standing at the origin, facing +x), for every keypoint:

* rotation offset   R_src^-1 R_tgt, so the offset source orientation is the
                    target body's orientation at rest;
* scale             |p_tgt - p_tgt_pelvis| / |p_src - p_src_pelvis|, and for the
                    pelvis itself the height ratio (GMR's ``human_scale_table``);
* position offset   the residual left at rest after scaling, in the target body
                    frame where GMR applies it.

So the source rest pose retargets exactly onto the target rest pose, and only the
body mapping (``robot.json``) and the weights below are hand-written.

A robot that cannot reach the shared rest pose with some limb lists those
keypoints as ``approximate_keypoints`` (ROBOTIS OP3's forearms and hands).  For a
pair involving it they are tracked by scaled position only: no orientation and
no rest-pose offset, since its rest differs from the other robot's there.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as R
from tqdm import tqdm

from .gmr import GeneralMotionRetargeting
from .motion import Motion
from .robots import Robot

ROOT = "pelvis"

# Canonical keypoints in priority order, with [position, rotation] weights for
# GMR's two IK passes.  When two keypoints land on the same target body (K1 has
# one trunk for pelvis and torso) the first wins.  Arms track position in both
# passes: rotation-only arm targets let a joint-limited shoulder (K1 roll) flip
# to the arm-overhead branch.
KEYPOINT_WEIGHTS = {
    "pelvis":          ([100, 10], [100, 5]),
    "torso":           ([0, 10],   [0, 5]),
    "head":            ([0, 2],    [0, 2]),
    "left_hip":        ([0, 10],   [10, 5]),
    "left_knee":       ([0, 10],   [10, 5]),
    "left_foot":       ([100, 10], [100, 50]),
    "right_hip":       ([0, 10],   [10, 5]),
    "right_knee":      ([0, 10],   [10, 5]),
    "right_foot":      ([100, 10], [100, 50]),
    "left_upper_arm":  ([10, 2],   [10, 1]),
    "left_forearm":    ([10, 2],   [10, 1]),
    "left_hand":       ([10, 0],   [10, 0]),
    "right_upper_arm": ([10, 2],   [10, 1]),
    "right_forearm":   ([10, 2],   [10, 1]),
    "right_hand":      ([10, 0],   [10, 0]),
}
# Hand orientation is tracked whenever the target has a wrist.  A wristless
# source's hand is rigid on its forearm, so its orientation reads as "wrist
# straight" and keeps the target wrist from drifting in the IK null space.
HAND_WEIGHTS_WITH_WRIST = ([10, 5], [10, 5])


def approximate_keypoints(src: Robot, tgt: Robot) -> set:
    return set(src.spec.get("approximate_keypoints", [])) | set(tgt.spec.get("approximate_keypoints", []))


def pair_keypoints(src: Robot, tgt: Robot) -> dict:
    """``{keypoint: (target body, pass-1 weights, pass-2 weights)}`` for one pair."""
    out, used, approx = {}, set(), approximate_keypoints(src, tgt)
    for k, (w1, w2) in KEYPOINT_WEIGHTS.items():
        if k not in src.keypoint_bodies or k not in tgt.keypoint_bodies:
            continue
        if k == "head" and not tgt.spec.get("head_actuated", True):
            continue  # a head link fixed to the torso would only drag the torso around
        body = tgt.keypoint_bodies[k]
        if body in used:
            continue
        used.add(body)
        if k.endswith("_hand") and tgt.spec.get("has_wrist", False):
            w1, w2 = HAND_WEIGHTS_WITH_WRIST
        if k in approx:
            w1, w2 = [w1[0], 0], [w2[0], 0]
        out[k] = (body, w1, w2)
    return out


def ik_config(src: Robot, tgt: Robot) -> dict:
    """The GMR IK config for retargeting ``src`` onto ``tgt``."""
    src_kp = src.keypoints(src.rest_qpos())
    tgt_poses = tgt.body_poses(tgt.rest_qpos())
    p_src_root = src_kp[ROOT][0]
    p_tgt_root = tgt_poses[tgt.keypoint_bodies[ROOT]][0]
    root_scale = p_tgt_root[2] / p_src_root[2]

    scales, table1, table2 = {}, {}, {}
    approx = approximate_keypoints(src, tgt)
    for k, (body, w1, w2) in pair_keypoints(src, tgt).items():
        (p_s, q_s), (p_t, q_t) = src_kp[k], tgt_poses[body]
        R_s, R_t = R.from_quat(q_s, scalar_first=True), R.from_quat(q_t, scalar_first=True)
        if k == ROOT:
            scale, scaled = root_scale, root_scale * p_s
        elif w1[0] == 0 and w2[0] == 0:
            # Orientation-only (head): never position-tracked, and the link can sit
            # anywhere (G1's head_link is at the pelvis), so no scale or offset.
            scale, scaled = root_scale, p_t
        else:
            d_s, d_t = p_s - p_src_root, p_t - p_tgt_root
            scale = np.linalg.norm(d_t) / max(np.linalg.norm(d_s), 1e-6)
            scaled = root_scale * p_src_root + scale * d_s
        offset = [0.0, 0.0, 0.0] if k in approx else R_t.inv().apply(p_t - scaled).tolist()
        rot_offset = (R_s.inv() * R_t).as_quat(scalar_first=True).tolist()
        scales[k] = float(scale)
        table1[body] = [k, *w1, offset, rot_offset]
        table2[body] = [k, *w2, offset, rot_offset]

    return {
        "robot_root_name": tgt.keypoint_bodies[ROOT],
        "human_root_name": ROOT,
        "ground_height": 0.0,
        "human_height_assumption": 1.0,  # unused: no actual_human_height is passed
        "use_ik_match_table1": True,
        "use_ik_match_table2": True,
        "human_scale_table": scales,
        "ik_match_table1": table1,
        "ik_match_table2": table2,
    }


def initial_qpos(src: Robot, tgt: Robot, src_keypoints: dict) -> np.ndarray:
    """IK seed: the target's upright rest pose under the source pelvis, turned to its heading.

    GMR otherwise starts from the MJCF's qpos0 (for Booster robots a T-pose with the
    trunk in the air), and a clip that opens in a squat can drag the legs into a
    folded IK branch that the per-frame solve never leaves.
    """
    q = tgt.rest_qpos()
    p_rest, q_rest = src.keypoints(src.rest_qpos())[ROOT]
    p0, q0 = src_keypoints[ROOT]
    rel = R.from_quat(q0, scalar_first=True) * R.from_quat(q_rest, scalar_first=True).inv()
    heading = R.from_euler("z", rel.as_euler("ZYX")[0])
    q[3:7] = (heading * R.from_quat(q[3:7], scalar_first=True)).as_quat(scalar_first=True)
    scale = tgt.keypoints(q)[ROOT][0][2] / p_rest[2]
    q[:2] += scale * p0[:2]
    return q


def retarget(motion: Motion, src: Robot, tgt: Robot, warmup_iters: int = 50,
             progress: bool = True) -> Motion:
    """Retarget ``motion`` (of robot ``src``) onto robot ``tgt``."""
    gmr = GeneralMotionRetargeting(tgt.xml_path, ik_config(src, tgt), verbose=False)
    out = []
    frames = tqdm(motion.qpos, desc=f"{src.name} -> {tgt.name}", disable=not progress)
    for t, q in enumerate(frames):
        kp = src.keypoints(q)
        if t == 0:
            gmr.configuration.update(initial_qpos(src, tgt, kp))
        # GMR runs only a few IK iterations per frame; settle on the first one.
        for _ in range(warmup_iters if t == 0 else 1):
            q_tgt = gmr.retarget(dict(kp))
        out.append(q_tgt)
    meta = {"source_robot": src.name, "source_file": motion.meta.get("source_file"),
            "source_start_frame": motion.meta.get("start_frame", 0)}
    return Motion(tgt.name, motion.fps, np.asarray(out), meta)
