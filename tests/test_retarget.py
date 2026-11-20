# Copyright (c) 2026 Jihwan Shin
# SPDX-License-Identifier: MIT
"""Sanity checks that need no data beyond the shipped assets (~30 s)."""

import itertools
from functools import lru_cache
from pathlib import Path

import numpy as np
import pytest

from robo_retarget import Robot, available_robots, ik_config, load_motion, retarget, save_motion
from robo_retarget.gmr import GeneralMotionRetargeting
from robo_retarget.retarget import approximate_keypoints

MOTIONS = Path(__file__).resolve().parents[1] / "assets" / "motions"
ROBOTS = available_robots()


@lru_cache(maxsize=None)
def robot(name: str) -> Robot:
    return Robot(name)


def sample_motion(name: str):
    files = sorted((MOTIONS / name).glob("*.csv"))
    if not files:
        pytest.skip(f"no sample motion for {name}")
    return load_motion(files[0], robot(name))


def test_all_robots_present():
    assert set(ROBOTS) >= {"g1", "h1", "h2", "ai_sapiens", "k1", "t1", "t2", "op3", "pi_plus", "wolfgang"}


@pytest.mark.parametrize("src,tgt", list(itertools.product(ROBOTS, repeat=2)))
def test_rest_pose_maps_to_rest_pose(src, tgt):
    """The calibration is exact at rest: the source rest pose lands on the target rest pose
    (for robots with approximate keypoints: on every other keypoint)."""
    s, t = robot(src), robot(tgt)
    kp = s.keypoints(s.rest_qpos())
    gmr = GeneralMotionRetargeting(t.xml_path, ik_config(s, t), verbose=False)
    for _ in range(200):
        q = gmr.retarget(dict(kp))
    ref = t.rest_qpos()
    np.testing.assert_allclose(q[:3], ref[:3], atol=5e-3)
    approx = approximate_keypoints(s, t)
    if not approx:
        np.testing.assert_allclose(q[7:], ref[7:], atol=2e-2)
    # Approximate keypoints still pull on their limb (and, through a waist, lean the torso), so
    # the rest of the robot lands within ~3 cm.
    got, want = t.keypoints(q), t.keypoints(ref)
    for k in set(t.keypoint_bodies) - approx:
        np.testing.assert_allclose(got[k][0], want[k][0], atol=3e-2 if approx else 1e-2, err_msg=k)


@pytest.mark.parametrize("name", ROBOTS)
def test_identity_retarget_reproduces_motion(name):
    """X -> X on a real clip gives back (nearly) the source motion."""
    motion = sample_motion(name).slice(0, 150)
    out = retarget(motion, robot(name), robot(name), progress=False)
    assert np.median(np.abs(out.qpos[:, 7:] - motion.qpos[:, 7:])) < 0.02
    assert np.abs(out.qpos[:, :3] - motion.qpos[:, :3]).max() < 0.03


@pytest.mark.parametrize("suffix", [".csv", ".pkl"])
def test_save_load_roundtrip(tmp_path, suffix):
    k1 = robot("k1")
    motion = sample_motion("k1").slice(0, 20)
    save_motion(tmp_path / f"m{suffix}", motion, k1)
    back = load_motion(tmp_path / f"m{suffix}", k1, fps=motion.fps)
    np.testing.assert_allclose(back.qpos, motion.qpos, atol=1e-5)
