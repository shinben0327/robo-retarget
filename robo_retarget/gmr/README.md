# `robo_retarget/gmr/` — vendored fork of GMR

Fork of [GMR (General Motion Retargeting)](https://github.com/YanjieZe/GMR) by Yanjie Ze,
baseline **`25f56db`, 2026-10-23**. It provides the differential-IK solver (mink on
MuJoCo) that robo-retarget drives with another robot's keypoints instead of a human's.

**Licence: files under `robo_retarget/gmr/` are MIT (`LICENSE` here), as is robo-retarget
itself (`render.py` excepted, see `THIRD_PARTY_NOTICES.md`).** Robot assets and sample motions carry their own licences (see
`THIRD_PARTY_NOTICES.md`).

| File | Status | Change |
|---|---|---|
| `LICENSE` | Verbatim | GMR's MIT text |
| `motion_retarget.py` | Modified | Constructor takes the MJCF path and the IK-config dict instead of names looked up in `params.py`; DoF/body/motor listing only when `verbose`; dropped `from rich import print` |
| `__init__.py` | New | Exports `GeneralMotionRetargeting` |

**Removed:** everything else — the SMPL-X / BVH / FBX / XRobot / OptiTrack source readers,
`params.py` and the robot registry, the IK configs, the viewer, `kinematics_model.py` and
the torch utilities, scripts, and the robot assets (the ones used here live in `assets/robots/`).
robo-retarget's sources are robots, read through their own MJCF forward kinematics, so
none of the human-side code is needed and the torch / smplx dependencies go with it.

The IK-config format (`ik_match_table1/2`, `human_scale_table`, ...) is unchanged;
robo-retarget generates one per robot pair at run time (`robo_retarget/retarget.py`),
naming the canonical keypoints where GMR names human bodies.
