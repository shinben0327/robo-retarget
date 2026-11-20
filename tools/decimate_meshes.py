# Copyright (c) 2026 Jihwan Shin
# SPDX-License-Identifier: MIT
"""Decimate meshes to binary STL: at most ``--max-faces`` and ``--ratio`` of the input faces.

CAD exports carry far more triangles than a render or an exact ground-height
query needs.  STL inputs are rewritten in place; other formats (OBJ) are written
as ``<stem>.stl`` next to them, and the MJCF must then name the ``.stl`` files.
The shipped meshes were reduced with:

    uv run --with pymeshlab python tools/decimate_meshes.py assets/robots/pi_plus/meshes/*.STL
    uv run --with pymeshlab python tools/decimate_meshes.py --ratio 0.35 --min-faces 1500 \\
        assets/robots/ai_sapiens/meshes/visual/obj/*.obj

MeshLab's quadric edge collapse, after welding duplicate vertices, preserving
boundaries and topology: CAD exports are rarely watertight, and plain quadric
decimation tears them open.  An STL already within the limits is left untouched;
every file written says what was done in its STL header ("robo-retarget: decimated"
or "robo-retarget: converted to STL"), so a changed third-party mesh is marked as such.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import pymeshlab


def decimate(path: Path, max_faces: int, ratio: float, min_faces: int) -> tuple[int, int]:
    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(str(path))
    ms.meshing_merge_close_vertices(threshold=pymeshlab.PercentageValue(0.05))
    ms.meshing_remove_duplicate_faces()
    ms.meshing_remove_null_faces()
    before = ms.current_mesh().face_number()
    target = max(min(max_faces, int(ratio * before)), min_faces)
    is_stl = path.suffix.lower() == ".stl"
    if before <= target and is_stl:
        return before, before  # nothing to do: keep the file as it is
    if before > target:
        ms.meshing_decimation_quadric_edge_collapse(
            targetfacenum=target, preserveboundary=True, boundaryweight=2.0,
            preservenormal=True, preservetopology=True, planarquadric=True, qualitythr=0.3)
    out = path if is_stl else path.with_suffix(".stl")
    ms.save_current_mesh(str(out), binary=True, save_face_color=False)
    stamp(out, "robo-retarget: decimated" if before > target else "robo-retarget: converted to STL")
    return before, ms.current_mesh().face_number()


def stamp(path: Path, note: str):
    """Write ``note`` into a binary STL's 80-byte header (which must not start with "solid")."""
    with open(path, "r+b") as f:
        f.write(note.encode("ascii").ljust(80)[:80])


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("meshes", nargs="+", type=Path)
    ap.add_argument("--max-faces", type=int, default=30000)
    ap.add_argument("--ratio", type=float, default=1.0, help="keep at most this share of faces")
    ap.add_argument("--min-faces", type=int, default=0, help="never reduce below this")
    args = ap.parse_args()
    for path in args.meshes:
        before, after = decimate(path, args.max_faces, args.ratio, args.min_faces)
        print(f"[+] {path}: {before} -> {after} faces")


if __name__ == "__main__":
    main()
