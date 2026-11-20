# Copyright (c) 2026 Jihwan Shin
# SPDX-License-Identifier: MIT
"""Render the README header: one G1 clip, retargeted onto every other robot, in one scene.

    uv run python tools/render_hero.py                       # every layout -> outputs/hero/
    uv run python tools/render_hero.py --layout troupe --motion assets/motions/g1/gangnam_style.csv
    uv run python tools/render_hero.py --layout instructor --motion assets/motions/g1/gangnam_style.csv \
        --start 720 --cam z=0.6,distance=7.9   # reframed for a clip that moves more

All robots share one MuJoCo scene (so they cast shadows on one floor), each placed at its
slot in the layout; their own root motion is kept, scaled by the retargeting.  Writes an
mp4 and a GIF per layout.
"""

from __future__ import annotations

import argparse
import math
import subprocess
import tempfile
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

from robo_retarget import Robot, load_motion, retarget
from robo_retarget.render import Renderer, write_video

# Slots (x towards the camera, y to the right in the image, optional yaw in degrees) and
# camera; the camera looks at (x, y, z) from straight ahead (or from ``azimuth``) with a narrow
# field of view, for a compact group shot.  Pairs left and right are matched by height.
# The nine followers as a class: rows rise in height towards the back and leave a lane down
# the middle (where G1 stands in front) with only H1, the tallest, at the end of it.
GRID = {"wolfgang": (0.0, -1.6), "op3": (0.0, -0.6), "pi_plus": (0.0, 0.6), "k1": (0.0, 1.6),
        "t1": (-1.1, -1.1), "ai_sapiens": (-1.1, 1.1),
        "t2": (-2.2, -2.0), "h1": (-2.2, 0.0), "h2": (-2.2, 2.0)}


def grid(dx: float, dy: float) -> dict:
    return {name: (x + dx, y + dy) for name, (x, y) in GRID.items()}


def facing(at, target) -> float:
    """Yaw (degrees) that turns a robot standing at ``at`` towards ``target``."""
    return math.degrees(math.atan2(target[1] - at[1], target[0] - at[0]))


def arc(centre) -> dict:
    """The class in three tight arcs around ``centre``, all facing it: four kid-size robots in
    front, three mid-size robots in their gaps, and the two tallest in the gaps behind."""
    rows = ((("wolfgang", "op3", "pi_plus", "k1"), 2.3), (("t1", "t2", "ai_sapiens"), 3.3),
            (("h1", "h2"), 4.3))
    out = {}
    for names, radius in rows:
        for i, name in enumerate(names):
            a = math.radians(180 + 22 * (i - (len(names) - 1) / 2))
            at = (centre[0] + radius * math.cos(a), centre[1] + radius * math.sin(a))
            out[name] = (*at, facing(at, centre))
    return out


LAYOUTS = {
    # G1 front and centre between the kid-size robots; behind, in the gaps, taller robots
    # towards the middle, and H1 at the back in the centre.
    "troupe": dict(slots={"k1": (0.8, -2.0), "op3": (0.9, -1.0), "g1": (1.0, 0.0),
                          "pi_plus": (0.9, 1.0), "wolfgang": (0.8, 2.0),
                          "t1": (-0.3, -1.5), "ai_sapiens": (-0.3, 1.5),
                          "t2": (-0.9, -0.75), "h2": (-0.9, 0.75), "h1": (-1.6, 0.0)},
                   cam=dict(distance=5.6, elevation=-9, x=0.2, z=0.9), size=(1280, 560)),
    # G1 in front of the 3 x 3 block.
    "block": dict(slots={"g1": (1.6, 0.0), **grid(0.4, 0.0)},
                  cam=dict(distance=7.0, elevation=-20, x=0.0, z=0.6), size=(1280, 640)),
    # The source alone on the left, the nine retargets in the block on the right.
    "leader": dict(slots={"g1": (0.2, -3.4), **grid(0.4, 0.8)},
                   cam=dict(distance=7.2, elevation=-14, x=-0.6, y=-0.5, z=0.75), size=(1280, 600)),
    # A flying-V with the G1 at the point, smallest robots nearest it, H1 closing the V.
    "chevron": dict(slots={"g1": (1.6, 0.0), "op3": (1.0, -0.75), "pi_plus": (1.0, 0.75),
                           "wolfgang": (0.4, -1.45), "k1": (0.4, 1.45), "t1": (-0.2, -2.15),
                           "ai_sapiens": (-0.2, 2.15), "t2": (-0.8, -2.85), "h2": (-0.8, 2.85),
                           "h1": (-0.9, 0.0)},
                    cam=dict(distance=6.2, elevation=-12, x=0.3, z=0.8), size=(1280, 560)),
    # G1 teaches the class, seen over its shoulder from behind, standing off to the left; the
    # class stands in arcs around it, each robot turned towards it, doing the same moves.
    "instructor": dict(slots={"g1": (2.6, -1.3, facing((2.6, -1.3), (-0.6, 0.0))),
                              **arc((2.6, 0.0))},
                       cam=dict(distance=7.3, elevation=-16, azimuth=192, x=0.3, y=-0.53, z=0.45),
                       size=(1280, 600)),
    # One dancer, centre stage, whose body changes ten times over the clip, shortest to tallest.
    "relay": dict(slots={n: (0.0, 0.0) for n in ("op3", "pi_plus", "wolfgang", "k1", "t1", "g1",
                                                 "ai_sapiens", "t2", "h2", "h1")},
                  cam=dict(distance=4.3, elevation=-8, z=1.0), size=(960, 720), relay=True),
}
FOVY = 30.0


def scene_xml(slots: dict, directory: Path) -> Path:
    """One MJCF with every robot attached under a name prefix; floors and lights dropped."""
    import mujoco as mj  # after robo_retarget, which selects the EGL backend

    world = mj.MjSpec()
    world.compiler.degree = False
    for name in slots:
        robot = Robot(name)
        spec = mj.MjSpec.from_file(str(robot.xml_path))
        root = robot.xml_path.parent
        for asset in [*spec.meshes, *spec.textures]:
            if asset.file:
                base = spec.meshdir if asset in spec.meshes else spec.texturedir
                asset.file = str((root / base / asset.file).resolve())
        spec.meshdir = spec.texturedir = ""
        for geom in spec.worldbody.geoms:
            spec.delete(geom)
        for light in spec.worldbody.lights:
            spec.delete(light)
        for key in spec.keys:
            spec.delete(key)
        world.worldbody.add_frame().attach_body(spec.worldbody.first_body(), f"{name}/", "")
    path = directory / "scene.xml"
    path.write_text(world.to_xml())
    return path


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--layout", choices=[*LAYOUTS, "all"], default="all")
    ap.add_argument("--motion", default="assets/motions/g1/dance_102.csv")
    ap.add_argument("--fps", type=float, default=60, help="source frame rate")
    ap.add_argument("--start", type=int, default=150, help="source frame (dance_102 starts moving at ~170)")
    ap.add_argument("--frames", type=int, default=480)
    ap.add_argument("--out-fps", type=float, default=30)
    ap.add_argument("--gif-fps", type=float, default=12)
    ap.add_argument("--gif-width", type=int, default=800)
    ap.add_argument("--style", default="studio")
    ap.add_argument("--cam", default="", help="camera overrides, e.g. z=0.6,distance=7.9")
    ap.add_argument("--supersample", type=int, default=2)
    ap.add_argument("--out", default="outputs/hero")
    args = ap.parse_args()

    g1 = Robot("g1")
    motion = load_motion(args.motion, g1, fps=args.fps).slice(args.start, args.frames)
    step = max(1, round(args.fps / args.out_fps))
    motions = {"g1": motion.qpos[::step]}
    for name in ("h1", "h2", "ai_sapiens", "k1", "t1", "t2", "op3", "pi_plus", "wolfgang"):
        motions[name] = retarget(motion, g1, Robot(name), progress=False).qpos[::step]

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    for layout in LAYOUTS if args.layout == "all" else [args.layout]:
        cfg = LAYOUTS[layout]
        with tempfile.TemporaryDirectory() as tmp:
            xml = scene_xml(cfg["slots"], Path(tmp))
            width, height = cfg["size"]
            overrides = (kv.split("=") for kv in args.cam.split(",") if kv)
            cam = {**cfg["cam"], **{k: float(v) for k, v in overrides}}
            renderer = Renderer(xml, style=args.style, size=width, height=height,
                                supersample=args.supersample, cam_distance=cam["distance"],
                                cam_azimuth=cam.get("azimuth", 180), cam_elevation=cam["elevation"],
                                lookat_z=cam["z"])
            renderer.model.vis.global_.fovy = FOVY
        # Each robot's block of qpos, in attach order; its motion is moved to its slot.
        q = np.zeros((len(motions["g1"]), renderer.model.nq))
        adr, turn = 0, np.arange(len(q)) * len(cfg["slots"]) // len(q)
        for i, (name, (x, y, *yaw)) in enumerate(cfg["slots"].items()):
            m = _place(motions[name], x, y, yaw[0] if yaw else 0.0)
            if cfg.get("relay"):
                m[turn != i, 1] += 100.0  # off stage while it is not its turn
            q[:, adr:adr + m.shape[1]] = m
            adr += m.shape[1]
        assert adr == renderer.model.nq
        lookat = np.array([cam.get("x", 0.0), cam.get("y", 0.0)])
        frames = np.stack([renderer.render(qi, lookat, anchor_xy=lookat) for qi in q])
        renderer.close()

        mp4 = out / f"{layout}.mp4"
        write_video(mp4, frames, args.out_fps)
        gif = mp4.with_suffix(".gif")
        _to_gif(mp4, gif, args.gif_fps, args.gif_width)
        print(f"[+] {mp4}  {gif} ({gif.stat().st_size / 1e6:.1f} MB)")


def _place(qpos: np.ndarray, x: float, y: float, yaw_deg: float) -> np.ndarray:
    """Move a motion so it starts at (x, y), turned by ``yaw_deg`` about its start."""
    q = qpos.copy()
    turn = R.from_euler("z", yaw_deg, degrees=True)
    q[:, :2] = turn.apply(np.c_[q[:, :2] - q[0, :2], np.zeros(len(q))])[:, :2] + [x, y]
    q[:, 3:7] = (turn * R.from_quat(q[:, 3:7], scalar_first=True)).as_quat(scalar_first=True)
    return q


def _to_gif(mp4: Path, gif: Path, fps: float, width: int):
    import imageio_ffmpeg

    vf = (f"fps={fps},scale={width}:-1:flags=lanczos,split[a][b];"
          "[a]palettegen=max_colors=128:stats_mode=diff[p];[b][p]paletteuse=dither=sierra2_4a")
    subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), "-y", "-loglevel", "error", "-i", str(mp4),
                    "-vf", vf, str(gif)], check=True)


if __name__ == "__main__":
    main()
