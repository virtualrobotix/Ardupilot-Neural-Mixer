#!/usr/bin/env python3
"""Build the articulated MuJoCo PhoneBot from the official preview GLB.

    .venv/bin/python tools/robots/phonebot_mjcf.py              # robots/phonebot/robot/phonebot.xml
    .venv/bin/python tools/robots/phonebot_mjcf.py --reversed   # robots/phonebot_rev/robot/phonebot_rev.xml

--reversed (catalog robot phonebot_rev) turns pelvis and legs 180° about z under the torso: the phone and the IMU face
what was the back of the legs, the knees bend the other way, and the leg that was on the
left is now on the right (it takes the right joint names; its servos keep their limits and
signs, only the bus IDs swap). The torso mass then sits behind the hips instead of in front.

phonebot.dev publishes PhoneBot-preview.glb, a rigid assembly without joints.
Every joint here sits on the output shaft of its DYNAMIXEL XL430 in that file:
the 13 servo bodies and their horns were located in the mesh, and each connected
part of the assembly is attached to the link that its servo or horn drives.
Collision geometry and masses are boxes around those parts.
"""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import robot_dir  # noqa: E402

# Soles of the GLB are at z = -0.2955 in the mesh frame (straight legs).
GLB_Z = 0.2955
XL430_TAU = 1.4
KP = 10.0
KD = 0.5

# Horn centres of the XL430s, pelvis frame (x forward, y left, z up), left leg.
# The right leg is the mirror image in y.
LEG_JOINTS = (
    ("hip_yaw", "0 0 1", (0.0058, 0.0525, -0.0413)),
    ("hip_roll", "1 0 0", (0.0058, 0.0524, -0.0670)),
    ("hip_pitch", "0 1 0", (0.0228, 0.0525, -0.0911)),
    ("knee_pitch", "0 1 0", (0.0228, 0.0525, -0.1686)),
    ("ankle_pitch", "0 1 0", (0.0227, 0.0525, -0.2394)),
    ("ankle_roll", "1 0 0", (0.0227, 0.0523, -0.2634)),
)
TORSO_JOINT = (0.0, 0.0, 0.0023)

# Collision box (centre, half size, mass) per link, pelvis frame, left leg.
LEG_BOXES = {
    "hip_yaw": ((0.006, 0.065, -0.045), (0.030, 0.010, 0.025), 0.030),
    "hip_roll": ((0.004, 0.053, -0.079), (0.035, 0.016, 0.023), 0.140),
    "hip_pitch": ((0.030, 0.053, -0.140), (0.022, 0.020, 0.040), 0.085),
    "knee_pitch": ((0.018, 0.052, -0.204), (0.015, 0.023, 0.035), 0.030),
    "ankle_pitch": ((0.040, 0.053, -0.251), (0.035, 0.016, 0.023), 0.140),
    "ankle_roll": ((0.035, 0.053, -0.2865), (0.055, 0.030, 0.009), 0.050),
}
FOOT_SITE = (0.035, 0.053, -0.2955)

# Official preview colours, one per extracted mesh (node order of PhoneBot-preview.glb).
GLB_RGBA = {
    "m0_0": "0.75 0.75 0.75 1", "m1_0": "0.74 0.72 0.67 1", "m2_0": "0.20 0.50 0.35 1",
    "m3_0": "0.47 0.68 0.82 1", "m4_0": "0.09 0.11 0.13 1", "m5_0": "0.22 0.49 0.71 1",
    "m6_0": "0.77 0.49 0.10 1", "m7_0": "0.74 0.77 0.80 1", "m8_0": "0.20 0.24 0.27 1",
    "m9_0": "0.11 0.11 0.11 1", "m10_0": "0.06 0.07 0.09 1", "m11_0": "1.00 0.76 0.05 1",
}

# Joint order matches the hardware figure: right J1-J6, left J7-J12, torso yaw J13.
JOINTS = (
    "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee_pitch",
    "right_ankle_pitch", "right_ankle_roll", "left_hip_yaw", "left_hip_roll",
    "left_hip_pitch", "left_knee_pitch", "left_ankle_pitch", "left_ankle_roll",
    "torso_yaw",
)
LINK = {
    "hip_yaw": "hip_yaw_link", "hip_roll": "hip_roll_link", "hip_pitch": "hip_pitch_link",
    "knee_pitch": "knee_link", "ankle_pitch": "ankle_pitch_link", "ankle_roll": "ankle_roll_link",
}


def _f(values) -> str:
    return " ".join(f"{float(value):.5g}" for value in values)


def _mirror(point, side: str) -> np.ndarray:
    p = np.array(point, dtype=np.float64)
    if side == "right":
        p[1] = -p[1]
    return p


def body_origins() -> dict[str, np.ndarray]:
    """Link origins in the pelvis frame at every joint angle zero."""
    origins = {"pelvis": np.zeros(3), "torso_yaw_link": np.array(TORSO_JOINT)}
    for side in ("right", "left"):
        for joint, _axis, point in LEG_JOINTS:
            origins[f"{side}_{LINK[joint]}"] = _mirror(point, side)
    return origins


def _range(side: str, joint: str) -> str:
    if joint == "hip_pitch":
        return "-2.1468 1.5708" if side == "left" else "-1.5708 2.1468"
    return "-1.5708 1.5708"


def _ctrlrange(name: str, reversed_: bool = False) -> str:
    side, _, joint = name.partition("_")
    if reversed_ and side in ("left", "right"):
        side = {"left": "right", "right": "left"}[side]
    return _range(side, joint) if joint == "hip_pitch" else "-1.5708 1.5708"


def _owner(center: np.ndarray, extent: np.ndarray) -> str:
    """Link that carries one connected part, from its centre in the GLB frame.

    GLB frame: x to the robot's left, y backward, z up, pelvis at the origin.
    A servo body rides on the link before its joint and its horn on the link
    after it; roll and pitch servos of the hip and of the ankle are bolted in pairs.
    """
    x, y, z = center
    if z > 0.0:
        return "torso_yaw_link"
    ax = abs(x)
    if ax < 0.045 and z > -0.045:
        return "pelvis"
    side = "left" if x > 0 else "right"
    # Idler hubs and their screws on the inner face turn with the horn on the outer face.
    if z < -0.05 and extent.max() < 0.012 and 0.030 < ax < 0.042 and abs(y + 0.023) < 0.012:
        ax = 0.072
    if z > -0.05 or (y > 0.028 and z > -0.11):
        link = "hip_yaw_link"
    elif z > -0.105 and ax < 0.068:
        link = "hip_roll_link"
    elif z > -0.105 or (z > -0.185 and ax < 0.068):
        link = "hip_pitch_link"
    elif z > -0.225 or (ax >= 0.068 and z > -0.25):
        link = "knee_link"
    elif z > -0.275 and y > -0.072:
        link = "ankle_pitch_link"
    else:
        link = "ankle_roll_link"
    return f"{side}_{link}"


def _load_obj(path: Path) -> tuple[np.ndarray, np.ndarray]:
    verts, faces = [], []
    for line in path.read_text().splitlines():
        if line.startswith("v "):
            verts.append([float(x) for x in line.split()[1:4]])
        elif line.startswith("f "):
            faces.append([int(tok.split("/")[0]) - 1 for tok in line.split()[1:4]])
    return np.asarray(verts, dtype=np.float64), np.asarray(faces, dtype=np.int64)


def _components(verts: np.ndarray, faces: np.ndarray) -> np.ndarray:
    """Connected-part label of every face (vertices welded at 10 µm)."""
    _, weld = np.unique(np.round(verts, 5), axis=0, return_inverse=True)
    weld = weld.ravel()
    parent = np.arange(weld.max() + 1)

    def find(a: int) -> int:
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    welded = weld[faces]
    for a, b in np.concatenate([welded[:, [0, 1]], welded[:, [1, 2]]]):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb
    return np.array([find(a) for a in welded[:, 0]])


def split_glb(mesh_dir: Path) -> dict[str, list[tuple[str, str]]]:
    """One visual mesh per link and GLB material, in that link's frame."""
    origins = body_origins()
    for stale in mesh_dir.glob("*__*.obj"):
        stale.unlink()
    owned: dict[str, list[tuple[str, str]]] = {name: [] for name in origins}
    for src, rgba in GLB_RGBA.items():
        path = mesh_dir / f"{src}.obj"
        if not path.is_file():
            continue
        verts, faces = _load_obj(path)
        if len(faces) == 0:
            continue
        labels = _components(verts, faces)
        face_owner = np.empty(len(faces), dtype=object)
        for label in np.unique(labels):
            sel = labels == label
            points = verts[faces[sel]].reshape(-1, 3)
            lo, hi = points.min(axis=0), points.max(axis=0)
            face_owner[sel] = _owner((lo + hi) / 2, hi - lo)
        # GLB (x left, y back) to MuJoCo (x forward, y left).
        mj = np.stack([-verts[:, 1], verts[:, 0], verts[:, 2]], axis=1)
        for body, origin in origins.items():
            chosen = faces[face_owner == body]
            if len(chosen) == 0:
                continue
            local = mj[chosen].reshape(-1, 3) - origin
            if len(np.unique(np.round(local, 5), axis=0)) < 4:
                continue
            mesh_name = f"{body}__{src}"
            lines = [f"v {v[0]:.5f} {v[1]:.5f} {v[2]:.5f}" for v in local]
            lines += [f"f {3 * i + 1} {3 * i + 2} {3 * i + 3}" for i in range(len(chosen))]
            (mesh_dir / f"{mesh_name}.obj").write_text("\n".join(lines) + "\n")
            owned[body].append((mesh_name, rgba))
    return owned


def _visual_lines(parts: list[tuple[str, str]], indent: str) -> str:
    return "\n".join(
        f'{indent}<geom type="mesh" class="visual" mesh="{name}" material="{name}"/>'
        for name, _rgba in parts
    )


def leg_xml(side: str, owned: dict[str, list[tuple[str, str]]], reversed_: bool = False) -> str:
    """Serial leg yaw, roll, pitch, knee, ankle pitch, ankle roll on the servo shafts.

    side is the GLB leg; reversed_ turns it 180° about the pelvis z axis and names it after
    the side it lands on.
    """
    name = {"left": "right", "right": "left"}[side] if reversed_ else side
    opening, closing = [], []
    parent = np.zeros(3)
    for depth, (joint, axis, point) in enumerate(LEG_JOINTS):
        pad = "      " + "  " * depth
        here = _mirror(point, side)
        mesh_body = f"{side}_{LINK[joint]}"
        body = f"{name}_{LINK[joint]}"
        centre, half, mass = LEG_BOXES[joint]
        centre = _mirror(centre, side) - here
        pos = here - parent
        turn = ""
        if reversed_ and depth == 0:
            pos = pos * np.array([-1.0, -1.0, 1.0])
            turn = ' quat="0 0 0 1"'
        lines = [
            f'{pad}<body name="{body}" pos="{_f(pos)}"{turn}>',
            f'{pad}  <joint name="{name}_{joint}" type="hinge" axis="{axis}" range="{_range(side, joint)}"/>',
            _visual_lines(owned.get(mesh_body, []), pad + "  "),
            f'{pad}  <geom type="box" class="collision" pos="{_f(centre)}" size="{_f(half)}" mass="{mass}"/>',
        ]
        if joint == "ankle_roll":
            lines.append(f'{pad}  <site name="{name}_foot_site" pos="{_f(_mirror(FOOT_SITE, side) - here)}" size="0.008" group="5"/>')
        opening.append("\n".join(line for line in lines if line))
        closing.insert(0, f"{pad}</body>")
        parent = here
    return "\n".join(opening + closing)


def build(owned: dict[str, list[tuple[str, str]]], reversed_: bool = False) -> str:
    actuators = "\n".join(
        f'    <position name="{name}" joint="{name}" kp="{KP}" kv="{KD}" '
        f'forcerange="{-XL430_TAU} {XL430_TAU}" ctrlrange="{_ctrlrange(name, reversed_)}"/>'
        for name in JOINTS
    )
    pelvis_visuals = _visual_lines(owned.get("pelvis", []), "      ")
    pelvis_box = (0.01, 0.0, -0.02)
    if reversed_:
        pelvis_visuals = pelvis_visuals.replace('class="visual"', 'class="visual" quat="0 0 0 1"')
        pelvis_box = (-0.01, 0.0, -0.02)
    layout = ("Reversed layout: pelvis and legs turned 180 deg under the torso (phonebot_mjcf.py --reversed)."
              if reversed_ else "GLB layout.")
    assets = []
    for parts in owned.values():
        for name, rgba in parts:
            assets.append(f'    <mesh name="{name}" file="{name}.obj" inertia="shell"/>')
            assets.append(f'    <material name="{name}" rgba="{rgba}"/>')
    asset_xml = "\n".join(assets)
    torso = np.array(TORSO_JOINT)
    return f"""<mujoco model="phonebot">
  <!-- Visual meshes from the official PhoneBot preview GLB (phonebot.dev, CC BY-NC 4.0).
       Joints sit on the XL430 output shafts of that assembly; collision boxes are approximate.
       {layout} -->
  <compiler angle="radian" meshdir="meshes" autolimits="true"/>
  <option timestep="0.005" integrator="implicitfast"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="160" elevation="-20"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4"
             rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
{asset_xml}
  </asset>
  <default>
    <default class="visual">
      <geom type="mesh" contype="0" conaffinity="0" group="2" density="0"/>
    </default>
    <default class="collision">
      <geom group="3" contype="1" conaffinity="0" condim="3" friction="0.8 0.01 0.001"/>
    </default>
    <joint damping="0.02" armature="0.0002"/>
  </default>
  <sensor>
    <framequat name="orientation" objtype="site" objname="imu"/>
    <gyro name="imu_ang_vel" site="imu"/>
    <accelerometer name="imu_accel" site="imu"/>
    <velocimeter name="imu_lin_vel" site="imu"/>
    <subtreeangmom name="root_angmom" body="pelvis"/>
  </sensor>
  <worldbody>
    <light pos="0 0 1.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" type="plane" size="0 0 0.05" material="groundplane"/>
    <body name="pelvis" pos="0 0 {GLB_Z}">
      <freejoint name="floating_base"/>
      <site name="imu" pos="0 0 0" size="0.006" group="5"/>
{pelvis_visuals}
      <geom type="box" class="collision" pos="{_f(pelvis_box)}" size="0.03 0.0675 0.02" mass="0.25"/>
{leg_xml("right", owned, reversed_)}
{leg_xml("left", owned, reversed_)}
      <body name="torso_yaw_link" pos="{_f(torso)}">
        <joint name="torso_yaw" type="hinge" axis="0 0 1" range="-1.5708 1.5708"/>
{_visual_lines(owned.get("torso_yaw_link", []), "        ")}
        <geom type="box" class="collision" pos="{_f(np.array((0.02, 0.0, 0.085)) - torso)}" size="0.03 0.053 0.09" mass="0.55"/>
      </body>
    </body>
  </worldbody>
  <actuator>
{actuators}
  </actuator>
  <keyframe>
    <key name="INIT" qpos="0 0 {GLB_Z} 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0"
         ctrl="0 0 0 0 0 0 0 0 0 0 0 0 0"/>
  </keyframe>
</mujoco>
"""


def main() -> None:
    reversed_ = "--reversed" in sys.argv[1:]
    robot = robot_dir("phonebot") / "robot"
    mesh_dir = robot / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    owned = split_glb(mesh_dir)
    xml = build(owned, reversed_)
    if reversed_:
        # catalog robot phonebot_rev: same meshes, read from the phonebot tree
        out = robot_dir("phonebot_rev") / "robot"
        out.mkdir(parents=True, exist_ok=True)
        xml = xml.replace('meshdir="meshes"', 'meshdir="../../phonebot/robot/meshes"')
        path = out / "phonebot_rev.xml"
    else:
        path = robot / "phonebot.xml"
    path.write_text(xml)
    counts = {body: len(parts) for body, parts in owned.items() if parts}
    print(f"wrote {path}")
    print("visual parts:", counts)


if __name__ == "__main__":
    main()
