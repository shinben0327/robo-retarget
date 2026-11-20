# Copyright (c) 2026 Jihwan Shin
# SPDX-License-Identifier: MIT
"""Render every robot's rest pose, front and side, into one image.

    uv run python tools/render_rest_poses.py            # -> docs/rest_poses.png
    uv run python tools/render_rest_poses.py k1 h1      # a subset, e.g. to check a new robot

Each robot gets a card (front and side view under its name, height and DoF), framed
to its own size so that a kid-size robot is as easy to inspect as an adult-size one;
cards go tallest first.
"""

from __future__ import annotations

import argparse
import math

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from robo_retarget import Robot, available_robots
from robo_retarget.render import HIDDEN_GEOM_GROUPS, Renderer

FILL = 0.9  # share of the view height the robot spans
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans{}.ttf"


def bounds(robot: Robot, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """World-frame box around the robot's visible geoms in pose ``q``."""
    import mujoco as mj  # after robo_retarget, which selects the EGL backend

    m = robot.model
    d = mj.MjData(m)
    d.qpos[:] = q
    mj.mj_forward(m, d)
    pts = []
    for g in range(m.ngeom):
        if m.geom_bodyid[g] == 0 or m.geom_group[g] in HIDDEN_GEOM_GROUPS:
            continue
        if m.geom_type[g] == mj.mjtGeom.mjGEOM_MESH:
            mid = m.geom_dataid[g]
            v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
            pts.append(v @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g])
        else:
            pts.append(d.geom_xpos[g] + np.array([[-1], [1]]) * m.geom_rbound[g])
    pts = np.concatenate(pts)
    return pts.min(0), pts.max(0)


def font(size: int, bold: bool = False):
    try:
        return ImageFont.truetype(FONT.format("-Bold" if bold else ""), size)
    except OSError:
        return ImageFont.load_default()


def card(robot: Robot, width: int, height: int) -> tuple[np.ndarray, float]:
    """Front and side view of the rest pose under a title; returns (image, robot height)."""
    q = robot.rest_qpos()
    lo, hi = bounds(robot, q)
    size = hi - lo
    views = []
    for azimuth, across, depth in ((180, size[1], size[0]), (90, size[0], size[1])):
        renderer = Renderer(robot.xml_path, style="white", size=width, height=height, supersample=2,
                            cam_azimuth=azimuth, cam_elevation=-6, lookat_z=(lo[2] + hi[2]) / 2)
        half_fov = math.radians(renderer.model.vis.global_.fovy / 2)
        half = max(size[2] / 2 / FILL, across / 2 / FILL * height / width)
        renderer.cam.distance = half / math.tan(half_fov) + depth / 2
        # A catalogue look: no shadows across the views and no floor edge behind the robot.
        renderer.model.light_castshadow[:] = 0
        renderer.model.geom_rgba[renderer.model.geom_bodyid == 0, 3] = 0
        views.append(renderer.render(q, (lo[:2] + hi[:2]) / 2))
        renderer.close()
    img = Image.fromarray(np.concatenate(views, 1))
    title = Image.new("RGB", (img.width, height // 6), (255, 255, 255))
    draw = ImageDraw.Draw(title)
    name = robot.spec["name"].split(" (")[0]
    detail = f"{size[2]:.2f} m  ·  {robot.model.nq - 7} DoF  ·  front / side"
    draw.text((title.width // 2, title.height * 0.38), name, fill=(28, 30, 34), anchor="mm",
              font=font(height // 17, bold=True))
    draw.text((title.width // 2, title.height * 0.78), detail, fill=(110, 114, 122), anchor="mm",
              font=font(height // 24))
    return np.concatenate([np.asarray(title), np.asarray(img)], 0), size[2]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("robots", nargs="*", help="default: all")
    ap.add_argument("--out", default="docs/rest_poses.png")
    ap.add_argument("--size", type=int, default=420, help="pixel height of a view (width: 2/3 of it)")
    ap.add_argument("--cols", type=int, default=5, help="robots per row")
    args = ap.parse_args()

    cards = [card(Robot(name), args.size * 2 // 3, args.size) for name in args.robots or available_robots()]
    tiles = [img for img, _ in sorted(cards, key=lambda c: -c[1])]
    gap = args.size // 14
    tiles = [np.pad(t, ((gap, gap), (gap, gap), (0, 0)), constant_values=255) for t in tiles]
    tiles += [np.full_like(tiles[0], 255)] * (-len(tiles) % args.cols)
    rows = [np.concatenate(tiles[i:i + args.cols], 1) for i in range(0, len(tiles), args.cols)]
    Image.fromarray(np.concatenate(rows, 0)).save(args.out, optimize=True)
    print(f"[+] {args.out}")


if __name__ == "__main__":
    main()
