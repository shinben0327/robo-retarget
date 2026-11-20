# Copyright (c) 2026 Jihwan Shin
# SPDX-License-Identifier: MIT
"""robo-retarget: retarget motions between humanoid robots."""

import os

# Rendering is offscreen only.  MuJoCo picks its GL backend when it is first
# imported, so this has to be set before any submodule imports it.
os.environ.setdefault("MUJOCO_GL", "egl")

from .motion import Motion, load_motion, save_motion
from .retarget import ik_config, retarget
from .robots import Robot, available_robots

__all__ = ["Motion", "Robot", "available_robots", "ik_config", "load_motion", "retarget",
           "save_motion"]
