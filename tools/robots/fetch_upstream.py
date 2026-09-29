#!/usr/bin/env python3
# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""Fetch the upstream robot repository and build the MuJoCo scene used for training.

    python tools/robots/fetch_upstream.py --robot microban
    python tools/robots/fetch_upstream.py --robot rex          # URDF -> robots/rex/robot/scene.xml

What it does, from the "upstream" block of robots/<id>/robot/profile.json:
- shallow clone of upstream.repo into third_party/<id> (git, no LFS)
- native MJCF (upstream.sim_model ends with .xml): used in place
- URDF only (upstream.urdf): MuJoCo compiles the URDF, a free joint, a floor,
  IMU sensors and position actuators are added, result written to
  robots/<id>/robot/scene.xml
- no upstream model (sim.mjcf): the hand-made MJCF versioned in
  robots/<id>/robot/ is used; the clone only provides the reference code
- optional published policy (upstream.policy_url): downloaded next to the
  robot's policies/ as <name>.onnx and converted to .nnm when the ONNX follows
  the NNMixer contract
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import urllib.request
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import THIRD_PARTY, load_profile, robot_dir, upstream  # noqa: E402


def git_clone(repo: str, dest: Path, branch: str | None) -> None:
    if (dest / ".git").exists():
        print(f"already cloned: {dest}")
        return
    dest.parent.mkdir(parents=True, exist_ok=True)
    cmd = ["git", "clone", "--depth", "1"]
    if branch:
        cmd += ["--branch", branch]
    cmd += [repo, str(dest)]
    print("+", " ".join(cmd))
    subprocess.check_call(cmd, env={"GIT_LFS_SKIP_SMUDGE": "1", **__import__("os").environ})


def urdf_to_scene(urdf: Path, profile: dict, out: Path) -> None:
    import mujoco

    sim = profile.get("sim", {})
    spec = mujoco.MjSpec.from_file(str(urdf))
    spec.compiler.discardvisual = False
    spec.option.timestep = 0.005
    world = spec.worldbody
    base_name = sim.get("trunk_body")
    base = spec.body(base_name) if base_name else None
    if base is None:
        base = next(b for b in world.bodies)
        print(f"trunk body not set; using first body {base.name!r}")
    # URDF bodies hang from world with a fixed joint; give the trunk a free joint
    if not any(j.type == mujoco.mjtJoint.mjJNT_FREE for j in base.joints):
        base.add_freejoint(name=sim.get("freejoint", "root"))
    base.pos = [0.0, 0.0, float(sim.get("home_z", 0.3))]
    world.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[5, 5, 0.1],
                   rgba=[0.8, 0.8, 0.8, 1.0])
    world.add_light(pos=[0, 0, 3], dir=[0, 0, -1])
    site = base.add_site(name="nnm_imu")
    spec.add_sensor(name=sim.get("gyro_sensor", "nnm_gyro"), type=mujoco.mjtSensor.mjSENS_GYRO,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname=site.name)
    spec.add_sensor(name=sim.get("accel_sensor", "nnm_accel"), type=mujoco.mjtSensor.mjSENS_ACCELEROMETER,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname=site.name)
    kp = float(sim.get("position_kp", 10.0))
    kv = float(sim.get("position_kv", 0.3))
    frc = float(sim.get("force_limit", 3.0))
    for name in profile["joint_names"]:
        act = spec.add_actuator(name=f"{name}_pos", target=name, trntype=mujoco.mjtTrn.mjTRN_JOINT)
        act.set_to_position(kp=kp, kv=kv)
        act.forcelimited = True
        act.forcerange = [-frc, frc]
    model = spec.compile()
    out.parent.mkdir(parents=True, exist_ok=True)
    # mesh paths in the written XML must resolve from robots/<id>/robot/
    spec.meshdir = str(urdf.parent.resolve())
    out.write_text(spec.to_xml())
    print(f"wrote {out}: nq={model.nq} nu={model.nu}")


def native_to_scene(mjcf: Path, profile: dict, out: Path) -> None:
    """A native upstream MJCF whose joint names differ from the profile (upstream.joint_map:
    profile name -> MJCF name) and/or whose actuators lack a torque limit (sim.force_limit):
    rename joints (and the actuators, sensors, keyframes that reference them), set forcerange,
    write robots/<id>/robot/scene.xml with absolute mesh paths. Physics otherwise untouched."""
    import mujoco

    sim = profile.get("sim", {})
    jmap = upstream(profile).get("joint_map") or {}
    spec = mujoco.MjSpec.from_file(str(mjcf))
    old_to_new = {old: new for new, old in jmap.items()}
    for j in spec.joints:
        if j.name in old_to_new:
            j.name = old_to_new[j.name]
    for a in spec.actuators:
        if a.target in old_to_new:
            a.target = old_to_new[a.target]
    for s in spec.sensors:
        if s.objname in old_to_new:
            s.objname = old_to_new[s.objname]
    frc = sim.get("force_limit")
    if frc:
        for a in spec.actuators:
            a.forcelimited = True
            a.forcerange = [-float(frc), float(frc)]
    # a torque-limited servo needs a realistic joint damping: max speed = force_limit / damping. Applied to
    # the actuated joints only (the free joint and unactuated ones keep the upstream values).
    actuated = {a.target for a in spec.actuators}
    for key, attr in (("joint_damping", "damping"), ("joint_frictionloss", "frictionloss"),
                      ("joint_armature", "armature")):
        if key in sim:
            for j in spec.joints:
                if j.name in actuated:
                    cur = getattr(j, attr)
                    if isinstance(cur, np.ndarray):           # damping is a [3] array in MjSpec
                        cur = cur.copy(); cur[0] = float(sim[key]); setattr(j, attr, cur)
                    else:
                        setattr(j, attr, float(sim[key]))
    # foot sites the env needs for foot position / air time, when the upstream model has none
    have_sites = {s.name for s in spec.sites}
    for f in sim.get("feet", []):
        if f.get("site") not in have_sites and f.get("pos") is not None:
            spec.body(f["body"]).add_site(name=f["site"], pos=[float(x) for x in f["pos"]], size=[0.005, 0, 0])
    integ = sim.get("integrator")
    if integ:
        spec.option.integrator = getattr(mujoco.mjtIntegrator, f"mjINT_{integ.upper()}")
    missing = [n for n in profile["joint_names"] if all(j.name != n for j in spec.joints)]
    if missing:
        raise SystemExit(f"joints not in {mjcf}: {missing}")
    model = spec.compile()
    spec.meshdir = str((mjcf.parent / (spec.meshdir or "")).resolve())
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(spec.to_xml())
    print(f"wrote {out}: nq={model.nq} nu={model.nu} (joints renamed: {len(old_to_new)}, "
          f"forcerange ±{frc if frc else 'upstream'})")


def fetch_policy(url: str, robot_id: str, name: str) -> Path:
    dst = robot_dir(robot_id) / "policies" / f"{name}.onnx"
    dst.parent.mkdir(parents=True, exist_ok=True)
    print(f"download {url} -> {dst}")
    urllib.request.urlretrieve(url, dst)
    return dst


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", required=True)
    ap.add_argument("--no-policy", action="store_true")
    args = ap.parse_args()
    profile = load_profile(args.robot)
    u = upstream(profile)
    if not u.get("repo"):
        raise SystemExit(f"{args.robot}: no upstream.repo in profile")
    for key in ("repo", "model_repo"):
        if u.get(key):
            dest = THIRD_PARTY / args.robot if key == "repo" else THIRD_PARTY / f"{args.robot}_model"
            git_clone(u[key], dest, u.get("branch") if key == "repo" else u.get("model_branch"))
    note = u.get("sim_model_note")
    sim_model = u.get("sim_model")
    root = THIRD_PARTY / (f"{args.robot}_model" if u.get("model_repo") else args.robot)
    own = profile.get("sim", {}).get("mjcf")
    if own:
        p = robot_dir(args.robot) / "robot" / own
        print(f"MJCF in this repo: {p} ({'found' if p.is_file() else 'MISSING'})")
    elif sim_model and sim_model.endswith(".xml"):
        p = root / sim_model
        print(f"native MJCF: {p} ({'found' if p.is_file() else 'MISSING'})")
        sim_keys = ("force_limit", "joint_damping", "joint_frictionloss", "joint_armature", "integrator")
        needs_scene = (u.get("joint_map") or any(k in profile.get("sim", {}) for k in sim_keys)
                       or any(f.get("pos") is not None for f in profile.get("sim", {}).get("feet", [])))
        if p.is_file() and needs_scene:
            native_to_scene(p, profile, robot_dir(args.robot) / "robot" / "scene.xml")
    elif u.get("urdf"):
        urdf_to_scene(root / u["urdf"], profile, robot_dir(args.robot) / "robot" / "scene.xml")
    else:
        print(f"no MuJoCo model can be built automatically for {args.robot}.")
    if note:
        print(f"note: {note}")
    if u.get("policy_url") and not args.no_policy:
        onnx_path = fetch_policy(u["policy_url"], args.robot, u.get("policy_name", "upstream"))
        cmd = [sys.executable, str(Path(__file__).parent / "export_nnm.py"), str(onnx_path),
               "--robot", args.robot, "--parity"]
        print("+", " ".join(cmd))
        subprocess.call(cmd)


if __name__ == "__main__":
    main()
