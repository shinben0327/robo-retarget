# Copyright (c) 2026, ETH Zurich (render styles adapted from hoi-retarget; licence text in
# THIRD_PARTY_NOTICES.md)
# Copyright (c) 2026 Jihwan Shin
# SPDX-License-Identifier: BSD-3-Clause
"""Offscreen MuJoCo renders and side-by-side source/target videos.

The look is hoi-retarget's render styles, condensed to the robot-only parts: a
key/fill/rim light rig that follows the robot, a two-tone robot recolour, and the
same fixed camera distance and lookat height in every panel, so a small robot
looks small next to a big one.  Six styles: ``studio`` (light-grey set, the
default), ``white`` (white cyclorama, for figures), ``stage`` (dark, glossy floor) and
``daylight``, ``night`` and ``slate`` (light, dark and blue-grey; matte, soft
shadows, no horizon).  Frames render at the
output size by default; ``supersample=2`` renders at twice the size and
downscales, which looks smoother and is ~4x slower.
"""

from __future__ import annotations

import os
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco as mj
import numpy as np
from PIL import Image, ImageDraw, ImageFont

STYLES = {
    "studio": dict(sky=("0.44 0.52 0.64", "0.92 0.94 0.96"), floor="0.66 0.67 0.69",
                   mark="0.56 0.57 0.60", ambient=0.34, diffuse=(0.62, 0.28, 0.38),
                   robot=((0.82, 0.83, 0.85), (0.17, 0.18, 0.20))),
    "white": dict(sky=("1 1 1", "1 1 1"), floor="0.985 0.985 0.99", mark="0.985 0.985 0.99",
                  ambient=0.40, diffuse=(0.60, 0.32, 0.30),
                  robot=((0.72, 0.735, 0.765), (0.19, 0.20, 0.22))),
    "stage": dict(sky=("0.05 0.055 0.065", "0.20 0.215 0.24"), floor="0.16 0.165 0.18",
                  mark="0.24 0.25 0.27", ambient=0.30, diffuse=(0.75, 0.25, 0.55),
                  robot=((0.86, 0.87, 0.89), (0.38, 0.39, 0.42)), reflectance=0.12),
    "daylight": dict(sky=("0.70 0.705 0.715", "0.70 0.705 0.715"), floor="0.66 0.665 0.68", haze=0.35,
                     mark="0.60 0.605 0.62", ambient=0.40, diffuse=(0.50, 0.24, 0.28),
                     robot=((0.93, 0.935, 0.945), (0.18, 0.19, 0.21))),
    "night": dict(sky=("0.085 0.09 0.10", "0.085 0.09 0.10"), floor="0.16 0.165 0.18", haze=0.35,
                  mark="0.21 0.215 0.23", ambient=0.30, diffuse=(0.70, 0.22, 0.45),
                  robot=((0.86, 0.87, 0.89), (0.42, 0.43, 0.46))),
    "slate": dict(sky=("0.38 0.44 0.52", "0.38 0.44 0.52"), floor="0.42 0.47 0.54", haze=0.35,
                  mark="0.37 0.42 0.49", ambient=0.36, diffuse=(0.58, 0.24, 0.34),
                  robot=((0.92, 0.925, 0.935), (0.16, 0.17, 0.19))),
}
# key / fill / rim: offset from the robot, casts shadow
LIGHTS = {"rr_key": ((2.2, -2.6, 3.4), True), "rr_fill": ((-3.0, -1.6, 1.7), False),
          "rr_rim": ((-1.4, 3.2, 2.6), False)}
HIDDEN_GEOM_GROUPS = (0, 3)  # collision copies (group 3 on Booster T2)


def _styled_mjcf(xml_path: Path, size: int, style: dict) -> str:
    """Write a restyled copy of the MJCF next to it (so mesh paths resolve); return its path."""
    root = ET.parse(xml_path).getroot()

    def sub(parent, tag):
        el = parent.find(tag)
        return el if el is not None else ET.SubElement(parent, tag)

    vis = sub(root, "visual")
    sub(vis, "global").attrib.update(offwidth=str(size), offheight=str(size))
    sub(vis, "quality").attrib.update(offsamples="8", shadowsize="8192")
    sub(vis, "map").attrib.update(znear="0.06", zfar="400", shadowclip="4", shadowscale="0.3",
                                      haze=str(style.get("haze", 0)))
    a = style["ambient"]
    sub(vis, "headlight").attrib.update(diffuse="0.05 0.05 0.05", ambient=f"{a} {a} {a}",
                                        specular="0 0 0")
    sub(vis, "rgba").set("haze", style["sky"][1] + " 1")

    assets = root.findall("asset") or [ET.SubElement(root, "asset")]
    for asset in assets:  # drop skyboxes; their ground textures go with their floors below
        for tex in asset.findall("texture"):
            if tex.get("type") == "skybox":
                asset.remove(tex)
    ET.SubElement(assets[-1], "texture", type="skybox", builtin="gradient", rgb1=style["sky"][0],
                  rgb2=style["sky"][1], width="512", height="512")
    ET.SubElement(assets[-1], "texture", name="rr_ground", type="2d", builtin="checker",
                  mark="edge", markrgb=style["mark"], rgb1=style["floor"], rgb2=style["floor"],
                  width="1024", height="1024")
    ET.SubElement(assets[-1], "material", name="rr_ground", texture="rr_ground",
                  texuniform="true", texrepeat="1 1", reflectance=str(style.get("reflectance", 0)),
                  specular="0.04")
    for name in ("rr_light", "rr_dark"):
        ET.SubElement(assets[-1], "material", name=name, specular="0.22", shininess="0.3")

    worldbody = root.findall("worldbody")[-1]
    floors = []
    for wb in root.findall("worldbody"):
        for light in wb.findall("light"):
            wb.remove(light)
        floors += [g for g in wb.findall("geom") if g.get("type") == "plane"]
    if not floors:  # menagerie models (H1, OP3) come without a scene
        floors.append(ET.SubElement(worldbody, "geom", name="rr_floor", type="plane",
                                    size="0 0 0.05", contype="0", conaffinity="0"))
    for geom in floors:
        geom.attrib.update(material="rr_ground", group="2")
    for (name, (offset, shadow)), diffuse in zip(LIGHTS.items(), style["diffuse"]):
        off = np.array(offset)
        ET.SubElement(worldbody, "light", name=name, directional="true",
                      pos=" ".join(map(str, off)), dir=" ".join(map(str, -off / np.linalg.norm(off))),
                      diffuse=f"{diffuse} {diffuse} {diffuse}", specular="0.1 0.1 0.1",
                      castshadow=str(shadow).lower())
    _drop_unused_textures(root)

    fd, out = tempfile.mkstemp(prefix=".styled_", suffix=".xml", dir=xml_path.parent)
    os.close(fd)
    ET.ElementTree(root).write(out)
    return out


def _drop_unused_textures(root):
    """Remove materials whose texture is gone and textures nothing references."""
    textures = {t.get("name") for a in root.findall("asset") for t in a.findall("texture")}
    for asset in root.findall("asset"):
        for mat in asset.findall("material"):
            if mat.get("texture") and mat.get("texture") not in textures:
                asset.remove(mat)


class Renderer:
    """Renders one MJCF scene in a studio style; frames are ``size`` square unless ``height``."""

    def __init__(self, xml_path, style: str = "studio", size: int = 1024, supersample: int = 1,
                 cam_distance: float = 3.0, cam_azimuth: float = 135.0,
                 cam_elevation: float = -4.0, lookat_z: float = 0.6, height: int | None = None):
        self.width, self.height = size, height or size
        self.ss, self.style = supersample, STYLES[style]
        styled = _styled_mjcf(Path(xml_path), max(self.width, self.height) * supersample, self.style)
        try:
            self.model = mj.MjModel.from_xml_path(styled)
        finally:
            os.remove(styled)
        self.data = mj.MjData(self.model)
        self._recolour()
        self.light_ids = {n: mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_LIGHT, n) for n in LIGHTS}
        self.opt = mj.MjvOption()
        for g in HIDDEN_GEOM_GROUPS:
            self.opt.geomgroup[g] = 0
        self.renderer = mj.Renderer(self.model, self.height * supersample, self.width * supersample)
        self.cam = mj.MjvCamera()
        self.cam.distance, self.cam.azimuth, self.cam.elevation = cam_distance, cam_azimuth, cam_elevation
        self.lookat_z = lookat_z

    def _recolour(self):
        m = self.model
        light = mj.mj_name2id(m, mj.mjtObj.mjOBJ_MATERIAL, "rr_light")
        dark = mj.mj_name2id(m, mj.mjtObj.mjOBJ_MATERIAL, "rr_dark")
        light_rgb, dark_rgb = self.style["robot"]
        for g in range(m.ngeom):
            if m.geom_bodyid[g] == 0:
                continue
            is_light = m.geom_rgba[g][:3].mean() > 0.45
            m.geom_rgba[g][:3] = light_rgb if is_light else dark_rgb
            m.geom_matid[g] = light if is_light else dark

    def render(self, qpos, lookat_xy, anchor_xy=None) -> np.ndarray:
        """``anchor_xy``: where the light rig is centred (default: the first root)."""
        self.data.qpos[:] = qpos
        anchor = np.array([*(qpos[:2] if anchor_xy is None else anchor_xy), 0.0])
        for name, lid in self.light_ids.items():
            self.model.light_pos[lid] = anchor + LIGHTS[name][0]
        mj.mj_forward(self.model, self.data)
        self.cam.lookat[:] = [lookat_xy[0], lookat_xy[1], self.lookat_z]
        self.renderer.update_scene(self.data, camera=self.cam, scene_option=self.opt)
        self.renderer.scene.flags[mj.mjtRndFlag.mjRND_SHADOW] = 1
        self.renderer.scene.flags[mj.mjtRndFlag.mjRND_REFLECTION] = "reflectance" in self.style
        img = self.renderer.render()
        if self.ss > 1:
            img = np.asarray(Image.fromarray(img).resize((self.width, self.height), Image.LANCZOS))
        return img

    def render_motion(self, qpos: np.ndarray, smooth: int = 21) -> np.ndarray:
        """Follow the root in xy, smoothed so the gait's sway does not shake the frame."""
        track = _moving_average(qpos[:, :2], smooth)
        return np.stack([self.render(q, xy) for q, xy in zip(qpos, track)])

    def close(self):
        self.renderer.close()


def _moving_average(x: np.ndarray, win: int) -> np.ndarray:
    win = min(win, len(x) // 2 * 2 - 1)
    if win < 3:
        return x
    xp = np.pad(x, ((win // 2, win // 2), (0, 0)), mode="edge")
    k = np.ones(win) / win
    return np.stack([np.convolve(xp[:, i], k, mode="valid") for i in range(x.shape[1])], axis=1)


def caption(width: int, text: str) -> np.ndarray:
    size = max(16, width // 40)
    pad = size // 2
    im = Image.new("RGB", (width, (size + 2 * pad + 1) // 2 * 2), (18, 20, 24))  # even height for x264
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", size)
    except OSError:
        font = ImageFont.load_default()
    ImageDraw.Draw(im).text((pad, pad), text, fill=(245, 245, 245), font=font)
    return np.asarray(im)


def side_by_side(panels: list[np.ndarray], captions: list[str]) -> np.ndarray:
    """Stack ``(T, H, W, 3)`` panels left to right, each under its caption."""
    n = max(len(p) for p in panels)
    cols = []
    for p, text in zip(panels, captions):
        p = np.concatenate([p, np.repeat(p[-1:], n - len(p), axis=0)])  # hold the last frame
        bar = caption(p.shape[2], text)
        cols.append(np.concatenate([np.broadcast_to(bar, (n, *bar.shape)), p], axis=1))
    return np.concatenate(cols, axis=2)


def write_video(path, frames: np.ndarray, fps: float) -> None:
    import imageio.v2 as imageio

    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with imageio.get_writer(path, fps=fps, codec="libx264", macro_block_size=1,
                            pixelformat="yuv420p", ffmpeg_params=["-crf", "18"]) as w:
        for f in frames:
            w.append_data(f)


def render_comparison(src_xml, src_qpos, tgt_xml, tgt_qpos, fps: float, path,
                      captions: tuple[str, str], **renderer_kwargs) -> None:
    """Source on the left, retargeted target on the right, same camera on both."""
    panels = []
    for xml, qpos in ((src_xml, src_qpos), (tgt_xml, tgt_qpos)):
        r = Renderer(xml, **renderer_kwargs)
        panels.append(r.render_motion(qpos))
        r.close()
    write_video(path, side_by_side(panels, list(captions)), fps)
