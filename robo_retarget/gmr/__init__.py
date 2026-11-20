# Derived from GMR (General Motion Retargeting).
# Source: https://github.com/YanjieZe/GMR @ 25f56db (2026-10-23)
# Copyright 2025 Yanjie Ze
#
# SPDX-License-Identifier: MIT
"""Vendored fork of GMR's IK solver (MIT, see ``LICENSE``); ``README.md`` lists the changes."""

from .motion_retarget import GeneralMotionRetargeting

__all__ = ["GeneralMotionRetargeting"]
