# Copyright (c) 2026 Jihwan Shin
# SPDX-License-Identifier: MIT
"""Convert a robot URDF into an MJCF that robo-retarget can load.

    uv run python tools/urdf_to_mjcf.py robot.urdf assets/robots/wolfgang/wolfgang.xml \\
        --meshes path/to/meshes

MuJoCo compiles the URDF itself; on top of that this script points every
``package://`` mesh at ``meshes/`` next to the output (copying the files), keeps
the visual geoms and the fixed links (hand and sole frames are useful keypoints),
gives the root link a free joint, and puts visual geoms in group 2 and collision
geoms in group 3, as in the other robots' MJCFs.
"""

from __future__ import annotations

import argparse
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco as mj


def convert(urdf: Path, out: Path, mesh_dir: Path) -> Path:
    robot = ET.parse(urdf).getroot()
    for tag in ("gazebo", "transmission", "ros2_control"):
        for el in robot.findall(tag):
            robot.remove(el)
    (out.parent / "meshes").mkdir(parents=True, exist_ok=True)
    for mesh in robot.iter("mesh"):
        name = Path(mesh.get("filename").split("://")[-1]).name
        shutil.copy(mesh_dir / name, out.parent / "meshes" / name)
        mesh.set("filename", name)
    ET.SubElement(ET.SubElement(robot, "mujoco"), "compiler", meshdir="meshes",
                  discardvisual="false", fusestatic="false", balanceinertia="true")
    tmp = out.parent / f".{out.stem}.urdf"
    ET.ElementTree(robot).write(tmp)
    try:
        spec = mj.MjSpec.from_file(str(tmp))
    finally:
        tmp.unlink()

    spec.modelname = out.stem
    root = spec.worldbody.first_body()
    root.add_freejoint()
    for geom in spec.geoms:
        visual = geom.contype == 0 and geom.conaffinity == 0
        geom.group = 2 if visual else 3
    spec.compile()
    out.write_text(spec.to_xml())
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("urdf", type=Path)
    ap.add_argument("out", type=Path, help="output MJCF; meshes go to meshes/ next to it")
    ap.add_argument("--meshes", type=Path, required=True, help="directory with the URDF's meshes")
    args = ap.parse_args()
    print(f"[+] {convert(args.urdf, args.out, args.meshes)}")


if __name__ == "__main__":
    main()
