# Copyright (c) 2026 Jihwan Shin
# SPDX-License-Identifier: MIT
"""``robo-retarget``: retarget a motion from one humanoid robot to another.

    robo-retarget --src g1 --tgt k1 --motion assets/motions/g1/dance1_subject2.csv \\
                  --out outputs/g1_to_k1/dance1_subject2.pkl --render

The output format follows the extension (``.pkl`` or ``.csv``, see ``motion.py``);
``--render`` writes a side-by-side mp4 (source left, target right) next to it.
"""

import argparse
from pathlib import Path

from .motion import load_motion, save_motion
from .render import STYLES
from .retarget import retarget
from .robots import Robot, available_robots


def main(argv=None) -> None:
    robots = available_robots()
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--src", required=True, choices=robots, help="source robot")
    ap.add_argument("--tgt", required=True, choices=robots, help="target robot")
    ap.add_argument("--motion", required=True, help="source motion (.csv or .pkl)")
    ap.add_argument("--out", default=None,
                    help="output .pkl or .csv (default: outputs/<src>_to_<tgt>/<motion>.pkl)")
    ap.add_argument("--fps", type=float, default=None,
                    help="source frame rate (default: from the .pkl, or the robot's csv.fps)")
    ap.add_argument("--start", type=int, default=0, help="first source frame")
    ap.add_argument("--frames", type=int, default=None, help="number of frames (default: all)")
    ap.add_argument("--render", action="store_true", help="also write a side-by-side mp4")
    ap.add_argument("--hq", action="store_true", help="2x supersampled render: smoother, ~4x slower")
    ap.add_argument("--style", default="studio", choices=list(STYLES), help="render style")
    ap.add_argument("--azimuth", type=float, default=135.0,
                    help="camera azimuth in degrees (180: facing the robot, 90: its right side)")
    args = ap.parse_args(argv)

    src, tgt = Robot(args.src), Robot(args.tgt)
    out = Path(args.out or f"outputs/{src.name}_to_{tgt.name}/{Path(args.motion).stem}.pkl")
    motion = load_motion(args.motion, src, fps=args.fps).slice(args.start, args.frames)

    result = retarget(motion, src, tgt)
    save_motion(out, result, tgt)
    print(f"[robo-retarget] {len(result)} frames at {result.fps:g} fps -> {out}")

    if args.render:
        from .render import render_comparison

        video = out.with_suffix(".mp4")
        render_comparison(src.xml_path, motion.qpos, tgt.xml_path, result.qpos, motion.fps, video,
                          captions=(f"source: {src.name}   {Path(args.motion).stem}",
                                    f"retargeted: {tgt.name}"),
                          style=args.style, supersample=2 if args.hq else 1, cam_azimuth=args.azimuth)
        print(f"[robo-retarget] video -> {video}")


if __name__ == "__main__":
    main()
