# Copyright (c) 2026 Jihwan Shin
# SPDX-License-Identifier: MIT
"""Add massless ``{left,right}_hand_tip`` bodies to a robot MJCF, in place.

The IK targets bodies, and robots without a hand link (Booster K1/T1, ROBOTIS
OP3 end in a forearm) have no frame at the fist.  The tip bodies give every robot
a comparable "hand" keypoint.  The shipped MJCFs were made with:

    python tools/add_hand_tips.py assets/robots/k1/K1_22dof.xml left_elbow_yaw_link 0.2
    python tools/add_hand_tips.py assets/robots/t1/T1_23dof.xml left_elbow_yaw_link 0.21
    python tools/add_hand_tips.py assets/robots/t2/T2_31dof.xml left_wrist_roll_link 0.08
    python tools/add_hand_tips.py assets/robots/op3/op3.xml l_el_link 0.13 --right r_el_link
    python tools/add_hand_tips.py assets/robots/h1/h1.xml left_elbow_link 0.29 --axis x
    python tools/add_hand_tips.py assets/robots/pi_plus/pi_plus.xml l_wrist_link 0.09 --axis=-z \
        --right r_wrist_link

The parent body is given for the left side; the right one is its mirror (``left``
-> ``right`` unless ``--right`` names it).  The offset is along the forearm in the
link frame: +-y (left/right) for robots whose zero pose is a T-pose, or the same
axis on both sides with ``--axis x`` (H1's forearms point forward at zero) or
``--axis=-z`` (Pi+'s arms hang down).
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path


def add_tips(xml: str, left_parent: str, offset: float, right_parent: str | None = None,
             axis: str = "y") -> str:
    parents = {"left": left_parent, "right": right_parent or left_parent.replace("left", "right")}
    for side, sign in (("left", 1), ("right", -1)):
        parent = parents[side]
        m = re.search(rf'<body name="{parent}"[^>]*?(/?)>', xml)
        if m is None:
            raise ValueError(f"body {parent!r} not found")
        if m.group(1):
            raise ValueError(f"body {parent!r} is self-closing")
        pos = {"y": f"0 {sign * offset:g} 0", "x": f"{offset:g} 0 0", "-z": f"0 0 {-offset:g}"}[axis]
        tip = (f'\n<!-- robo-retarget: hand keypoint frame -->'
               f'\n<body name="{side}_hand_tip" pos="{pos}"/>')
        xml = xml[:m.end()] + tip + xml[m.end():]
    return xml


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("xml")
    ap.add_argument("left_parent", help="body the left tip is attached to")
    ap.add_argument("offset", type=float, help="metres along the forearm")
    ap.add_argument("--right", default=None, help="right parent body, if not the mirrored name")
    ap.add_argument("--axis", choices=("y", "x", "-z"), default="y",
                    help="forearm direction in the link frame: +-y (left/right), +x or -z")
    args = ap.parse_args()

    path = Path(args.xml)
    path.write_text(add_tips(path.read_text(), args.left_parent, args.offset, args.right, args.axis))
    print(f"[+] {path}")


if __name__ == "__main__":
    main()
