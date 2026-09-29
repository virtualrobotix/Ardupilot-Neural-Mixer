# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""MuJoCo velocity-tracking environment that is already the ArduPilot deployment.

One step() = one AP_NNMixer policy tick. Inside it the physics runs at the
autopilot loop rate (200 Hz), the simulated IMU feeds the same gravity filter
the firmware runs, and the action reaches the joints through the PWM wire
encoding. See deploy_contract.py for the list of matched details.

The robot comes entirely from robots/<id>/robot/profile.json ("sim" block):
MJCF path, trunk body, free joint, IMU sensors, actuator model. Missing IMU
sensors are added to the trunk at load time.
"""

from __future__ import annotations

import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import deploy_contract as dc  # noqa: E402
from common import REPO_ROOT, load_profile, resolve_mjcf  # noqa: E402

SIM_DT = 1.0 / dc.AUTOPILOT_LOOP_HZ


@dataclass
class RewardWeights:
    track_lin_vel: float = 2.0
    track_ang_vel: float = 2.0
    upright: float = 2.0
    pose: float = 1.0
    action_rate: float = -0.8
    alive: float = 0.0
    # exp(-err²/sigma): with 0.25 a robot commanded 0.3 m/s kept 70% of the reward by standing still
    tracking_sigma: float = 0.05
    tracking_sigma_ang: float = 0.25
    # time constant of a low-pass on the base twist used by the tracking terms (0 = instantaneous). A small
    # trotting robot swings its speed within every step: the instantaneous error punished a gait at the right
    # mean speed more than standing still.
    tracking_filter_s: float = 0.0
    # extra error folded into the tracking kernel, on top of the command error. A bouncing or wobbling
    # step then drives exp() to zero, so the policy gets no gradient toward the command. Set to 0 when
    # vertical speed and roll/pitch already have their own penalties.
    tracking_lin_z: float = 2.0
    tracking_ang_rp: float = 0.05
    # gait terms of the MicroDuck mjlab task; active only when the profile lists sim.feet
    air_time: float = 0.0
    air_time_min_s: float = 0.1
    air_time_max_s: float = 0.5
    air_time_mode: str = "touchdown"        # "in_range": mjlab feet_air_time, per step while airborne in range
    # touchdown mode: flights shorter than this are contact bounces of a foot in stance, not steps; they
    # neither pay nor cost (walk_v18: each 0.02 s bounce cost 0.5 and the clipped step lost its gradient)
    air_time_debounce_s: float = 0.0
    # per step and foot airborne longer than air_time_max_s while commanded to move (negative weight):
    # walk_v19 turned by pivoting on two diagonal feet with a rear foot held in the air for whole seconds
    foot_hold: float = 0.0
    foot_clearance: float = 0.0
    swing_height_m: float = 0.03
    foot_swing_height: float = 0.0          # mjlab: (peak / target - 1)^2 at landing
    foot_slip: float = 0.0
    # phase-free gait term: one foot on the ground, the other in the air, while commanded to move.
    # (the phase-based feet_gait needs a gait phase in the observation, which AP_NNMixer does not send)
    single_stance: float = 0.0
    # positive reward for lifting the swing foot, linear in height up to swing_height_m, while moving
    foot_lift: float = 0.0
    # per touchdown after a real swing (>= alternation_min_air_s): +w if it is the other foot than the
    # previous touchdown, -w if the same foot steps again (a policy stepped with one foot only)
    foot_alternation: float = 0.0
    alternation_min_air_s: float = 0.05
    alternation_min_height_frac: float = 0.0     # step must reach this fraction of swing_height_m
    # per step, penalty on the height / air-time difference with the previous step of the other foot
    foot_symmetry: float = 0.0
    # quadruped trot, phase-free: each diagonal pair (sim.trot_pairs) shares its contact state and the two
    # pairs are in opposite states, while commanded to move
    trot: float = 0.0
    # sum of (actuator force / force limit)^2: small hobby servos run close to stall
    joint_torque: float = 0.0
    # quadruped four-beat walk, phase-free. footfall_sequence: per touchdown after a real swing, +w if the
    # foot follows the previous touchdown in sim.footfall_cycle (reversed when commanded backward), -w/2 if
    # the same foot steps again. three_stance: exactly one foot airborne (for less than air_time_max_s)
    # while moving. all_stance: all four feet down while moving (negative weight).
    footfall_sequence: float = 0.0
    three_stance: float = 0.0
    all_stance: float = 0.0
    # fewer than under_stance_feet feet on the ground (hops, flight phases), commanded or not (negative weight)
    under_stance: float = 0.0
    under_stance_feet: int = 2
    # per floor contact of a robot geom that is not a foot geom (shank, thigh, trunk lying on the floor)
    undesired_contacts: float = 0.0
    # sum of joint velocities squared (legged_gym dof_vel): slower, smoother leg motion
    joint_vel: float = 0.0
    # one-off reward when the episode ends by a fall (negative): falling never pays to end a costly episode
    termination: float = 0.0
    # quadratic posture penalty mean(((q - q0) / std)^2) with the pose_std tables (negative weight): unlike the
    # exponential pose term it keeps a gradient far from q0, where exp() is already zero
    pose_l2: float = 0.0
    # Go1 / Go2 (MuJoCo Playground, unitree_rl_lab) terms:
    # only_positive: the step reward is clipped at 0, so ending an episode never pays (legged_gym
    # only_positive_rewards). Tracking must stay inside the clip: with tracking added after it, every
    # penalty was clipped away and walk_v13 splayed its rear hips to the joint limit. orientation:
    # |g_xy|^2 of the true trunk tilt; stand_still: sum |q - q0| with a
    # zero command; air_time_variance: variance over the feet of the last air and contact times (clipped at
    # 0.5 s), which asks for a regular gait
    only_positive: bool = False
    orientation: float = 0.0
    stand_still: float = 0.0
    air_time_variance: float = 0.0
    # standing still while commanded to move (negative weight): 1 - progress, with progress the share of the
    # commanded velocity actually achieved (filtered twist projected on the command, clipped to [0, 1]),
    # averaged over the commanded linear and yaw parts
    no_progress: float = 0.0
    no_progress_min_cmd: float = 0.03
    # exp(-((trunk height - target) / std)^2): keeps the stand height instead of crouching
    base_height: float = 0.0
    base_height_target_m: float = 0.1
    base_height_std_m: float = 0.01
    # MicroDuck (mjlab velocity task) shapes, off by default
    upright_std: float | None = None        # exp(-|g_xy|^2 / std^2) on the true trunk orientation
    pose_std_standing: dict | None = None   # variable_posture: per-joint std (regex -> std)
    pose_std_walking: dict | None = None
    walking_threshold: float = 0.01
    self_collisions: float = 0.0
    dof_pos_limits: float = 0.0
    soft_limit_factor: float = 0.9
    body_ang_vel: float = 0.0
    angular_momentum: float = 0.0
    # exp(-mean(((q - q_ref) / std)^2)) on the joints of a scripted gesture (a wave, arms raised).
    # q_ref replaces q0 in the posture terms for those joints. The reference is a function of the
    # measured joint angle, so a feedforward policy can track it: the observation already has q and qd.
    gesture_pose: float = 0.0
    gesture_std: float = 0.12


def _quat_rotate_inverse(q, v):
    w, x, y, z = q
    qv = np.array([x, y, z])
    return v * (2.0 * w * w - 1.0) - np.cross(qv, v) * w * 2.0 + qv * np.dot(qv, v) * 2.0


class NNMixerEnv:
    def __init__(self, robot_id: str, ppo: dict[str, Any] | None = None, mjcf: Path | None = None,
                 seed: int = 0, actuator: str | None = None):
        self.profile = load_profile(robot_id)
        self.ppo = ppo or {}
        self.contract = dc.Contract.from_profile(self.profile, self.ppo)
        sim = self.profile.get("sim", {})
        self.sim = sim
        path = mjcf or resolve_mjcf(robot_id, self.profile)
        if path is None:
            raise FileNotFoundError(
                f"{robot_id}: no MJCF. Run tools/robots/fetch_upstream.py --robot {robot_id} "
                f"or set NNMIXER_MJCF")
        self.rng = np.random.default_rng(seed)
        env_cfg = self.ppo.get("env", {})
        self.cmd_ranges = env_cfg.get("command_ranges", {"vx": [-0.4, 0.4], "vy": [-0.3, 0.3], "wz": [-1.0, 1.0]})
        self.p_zero_cmd = float(env_cfg.get("p_zero_command", 0.2))
        self.p_no_vy = float(env_cfg.get("p_no_lateral", 0.3))
        self.p_single_axis = float(env_cfg.get("p_single_axis", 0.0))
        # single-axis command mix: probabilities of [+vx, -vx, +vy, -vy, +wz, -wz]; None = uniform
        saw = env_cfg.get("single_axis_weights")
        self.single_axis_p = np.array(saw, float) / np.sum(saw) if saw else None
        self.penalty_scale = 1.0
        self.terminate_illegal = bool(env_cfg.get("terminate_on_illegal_contact", False))
        # hard posture limits that end the episode: trunk lower than min_height_m, or a joint further than
        # max_dev (regex -> rad) from q0
        self.posture_limits = env_cfg.get("posture_limits") or {}
        self.episode_s = float(env_cfg.get("episode_s", 20.0))
        self.gyro_noise = float(env_cfg.get("gyro_noise", 0.02))
        self.accel_noise = float(env_cfg.get("accel_noise", 0.05))
        self.gravity_source = str(env_cfg.get("gravity_source", "ap_imu_filter"))
        self.ahrs_noise = float(env_cfg.get("ahrs_noise_rad", 0.01))
        self.obs_delay_max = int(env_cfg.get("obs_delay_steps_max", 1))
        self.fall_tilt_deg = float(env_cfg.get("fall_tilt_deg", 60.0))
        rs = env_cfg.get("resample_s")          # MicroDuck: new command every 3-8 s inside the episode
        self.resample_s = tuple(rs) if rs else None
        push = env_cfg.get("push") or {}        # random trunk velocity kicks (mjlab push_robot)
        self.push_interval = tuple(push["interval_s"]) if push else None
        self.push_vel = float(push.get("vel_xy", 0.0)) if push else 0.0
        self.w = RewardWeights(**env_cfg.get("reward", {}))
        self._setup_gesture(env_cfg.get("gesture"))
        self.actuator = actuator or sim.get("actuator", "position")
        self._load(Path(path))
        self.max_steps = int(self.episode_s * self.contract.rate_hz)
        self.reset()

    # ------------------------------------------------------------------ model
    def _load(self, path: Path) -> None:
        names = self.profile["joint_names"]
        trunk = self.sim.get("trunk_body")
        spec = mujoco.MjSpec.from_file(str(path))
        gyro_name = self.sim.get("gyro_sensor", "nnm_gyro")
        acc_name = self.sim.get("accel_sensor", "nnm_accel")
        have = {s.name for s in spec.sensors}
        if gyro_name not in have or acc_name not in have:
            body = spec.body(trunk) if trunk else None
            if body is None:
                raise ValueError("profile sim.trunk_body missing and MJCF has no IMU sensors")
            site = body.add_site(name="nnm_imu")
            if gyro_name not in have:
                spec.add_sensor(name=gyro_name, type=mujoco.mjtSensor.mjSENS_GYRO,
                                objtype=mujoco.mjtObj.mjOBJ_SITE, objname=site.name)
            if acc_name not in have:
                spec.add_sensor(name=acc_name, type=mujoco.mjtSensor.mjSENS_ACCELEROMETER,
                                objtype=mujoco.mjtObj.mjOBJ_SITE, objname=site.name)
        self.bam = None
        if self.actuator == "bam_xl330":
            sys.path.insert(0, str(REPO_ROOT / "plant"))
            from mujoco_json_plant import BAM_KP_FW, BAM_VIN, BAM_VIN_MIN, load_with_bam  # noqa: E402
            # load_with_bam compiles from file; write the augmented spec next to it
            import os
            tmp = path.parent / f".nnm_{path.stem}_sensors_{os.getpid()}_{id(self)}.xml"
            tmp.write_text(spec.to_xml())
            try:
                kp_fw = float(self.sim.get("bam_kp_fw", BAM_KP_FW))
                self.model, self.data, self.bam = load_with_bam(tmp, BAM_VIN, kp_fw)
            finally:
                tmp.unlink(missing_ok=True)
            _ = BAM_VIN_MIN
        else:
            self.model = spec.compile()
            self.model.opt.timestep = SIM_DT
            self.data = mujoco.MjData(self.model)
        m = self.model
        self.model.opt.timestep = SIM_DT
        jid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in names]
        if min(jid) < 0:
            missing = [n for n, j in zip(names, jid) if j < 0]
            raise ValueError(f"joints not in MJCF: {missing}")
        self.qpos_idx = np.array([m.jnt_qposadr[j] for j in jid])
        self.qvel_idx = np.array([m.jnt_dofadr[j] for j in jid])
        # actuator index for each profile joint
        act_for_joint = {}
        for a in range(m.nu):
            act_for_joint[int(m.actuator_trnid[a, 0])] = a
        self.act_idx = np.array([act_for_joint.get(j, -1) for j in jid])
        if (self.act_idx < 0).any() and self.bam is None:
            raise ValueError("every profile joint needs an actuator in the MJCF")
        fj_name = self.sim.get("freejoint")
        if fj_name:
            fj = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, fj_name)
        else:
            fj = next(j for j in range(m.njnt) if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE)
        self.free_qpos = int(m.jnt_qposadr[fj])
        self.free_qvel = int(m.jnt_dofadr[fj])
        self.trunk_id = int(m.jnt_bodyid[fj])
        self.gyro_adr = int(m.sensor_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, gyro_name)])
        self.acc_adr = int(m.sensor_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, acc_name)])
        self.home_z = float(self.sim.get("home_z", 0.3))
        # feet: site for position, body for contacts with the floor
        self.feet = []
        floor = {g for g in range(m.ngeom) if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE}
        self.floor_geoms = floor
        for f in self.sim.get("feet", []):
            sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, f["site"])
            bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f["body"])
            if sid < 0 or bid < 0:
                raise ValueError(f"foot {f} not in MJCF")
            self.feet.append((sid, bid))
        self.foot_body_ids = {bid for _, bid in self.feet}
        # optional foot geom: only that geom counts as the foot touching the floor (a shank lying on the
        # floor is not a foot contact)
        self.foot_geom_index = {}
        for i, f in enumerate(self.sim.get("feet", [])):
            if f.get("geom"):
                gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f["geom"])
                if gid < 0:
                    raise ValueError(f"foot geom {f['geom']} not in MJCF")
                self.foot_geom_index[gid] = i
        self.trot_pairs = [tuple(p) for p in self.sim.get("trot_pairs", [])]
        cycle = list(self.sim.get("footfall_cycle", []))
        self.footfall_next = {a: cycle[(k + 1) % len(cycle)] for k, a in enumerate(cycle)}
        self.footfall_prev = {a: cycle[k - 1] for k, a in enumerate(cycle)}
        lim = m.actuator_forcerange[np.maximum(self.act_idx, 0), 1]
        self.force_limit = np.where(lim > 0, lim, 1.0)
        # joint soft limits (mjlab joint_pos_limits) and per-joint posture std
        rng_lo, rng_hi = [], []
        for j in jid:
            lo, hi = m.jnt_range[j] if m.jnt_limited[j] else (-np.inf, np.inf)
            mid, half = (lo + hi) / 2, (hi - lo) / 2 * self.w.soft_limit_factor
            rng_lo.append(mid - half); rng_hi.append(mid + half)
        self.soft_lo, self.soft_hi = np.array(rng_lo), np.array(rng_hi)

        def stds(table):
            if not table:
                return None
            out = []
            for n in names:
                hit = [v for pat, v in table.items() if re.fullmatch(pat, n)]
                out.append(hit[0] if hit else 1.0)
            return np.array(out)
        self.pose_std_stand = stds(self.w.pose_std_standing)
        self.pose_std_walk = stds(self.w.pose_std_walking)
        dev = self.posture_limits.get("max_dev")
        self.max_dev = None
        if dev:
            self.max_dev = np.array([next((v for pat, v in dev.items() if re.fullmatch(pat, n)), np.inf)
                                     for n in names])
        # geoms of the robot (every body under the free-joint body) for self-collision counting
        robot_bodies = {b for b in range(m.nbody) if self._is_under(b, self.trunk_id)}
        self.robot_geom = np.array([m.geom_bodyid[g] in robot_bodies for g in range(m.ngeom)])
        # everything above the shank (the foot body) ends the episode when terminate_on_illegal_contact is set
        self.illegal_geom = self.robot_geom & np.array([m.geom_bodyid[g] not in self.foot_body_ids
                                                        for g in range(m.ngeom)])
        # torque actuators driven by a PD loop at the autopilot rate: kp/kd are a scalar or a
        # {regex: value} table per joint (Booster T1: hips and knees 200/5, ankles 50/1 N.m/rad)
        pd = self.sim.get("pd", {})

        def gains(v):
            if isinstance(v, dict):
                return np.array([next((g for pat, g in v.items() if re.fullmatch(pat, n)), 0.0) for n in names])
            return float(v or 0.0)
        self.kp = gains(pd.get("kp", 0.0))
        self.kd = gains(pd.get("kd", 0.0))

    # ------------------------------------------------------------------ gesture
    # Pose-command layout after the clock (extra[2:10]): offsets from q0 unless noted.
    POSE_ARM_NAMES = (
        "right_shoulder_pitch", "right_shoulder_roll", "right_elbow",
        "left_shoulder_pitch", "left_shoulder_roll", "left_elbow",
    )
    # knee_bend k -> knee +k, hip_pitch -0.48k, ankle_pitch -0.56k (ballet ratios on the MJCF)
    POSE_KNEE_HIP = -0.48
    POSE_KNEE_ANKLE = -0.56
    POSE_KNEE_DROP_M_PER_RAD = 0.0145   # trunk height lost per rad of knee_bend (flat foot)
    POSE_RANGES = np.array([
        [-1.55, 0.15], [-1.15, 0.05], [-1.35, -0.05],
        [-1.55, 0.15], [-0.05, 1.15], [-1.35, -0.05],
        [0.0, 0.55], [-0.15, 0.15],
    ], np.float32)
    POSE_TAU_S = 0.15

    def _setup_gesture(self, name: str | None) -> None:
        """Motion imitation of a short clip while standing (DeepMimic style, as the G1 dances).

        The clip is a few keyframes of arm joints over one period, interpolated with a cosine.
        The phase advances at clip_hz; sin and cos of the phase go into the two extra observation
        channels (the firmware fills them at NNM_CLOCK_HZ). The reward tracks the reference pose of
        the current phase, dense and unambiguous, so the policy has nothing to discover about timing.
        Joint signs are from the Microban MJCF: negative shoulder pitch brings the hand forward and
        up; the right shoulder roll abducts outward for negative angles.

        pose_cmd: the target is an 8-channel pose (6 arm offsets, knee_bend, sway) held as episode
        state and streamed into the extra observation channels after the clock — what a MAVLink
        teleop GCS will send. Random poses, zero-command, and clip-as-stream cover the domain."""
        self.gesture_spec = None
        self.gesture_ids: np.ndarray | None = None
        self.clip_phase = 0.0
        self.pose_cmd = False
        self.pose_external: np.ndarray | None = None
        if not name:
            return
        if name == "pose_cmd":
            self._setup_pose_cmd()
            return
        presets = {
            # keyframes: phase in [0, 1) -> joint angle; the clip is periodic
            "arms_up": {
                "hz": 0.0,
                "keys": {
                    "right_shoulder_pitch": [(0.0, -1.05)], "left_shoulder_pitch": [(0.0, -1.05)],
                    "right_elbow": [(0.0, -0.70)], "left_elbow": [(0.0, -0.70)],
                },
            },
            "wave_right": {
                # hand up (pitch + bent elbow) for the whole period, the shoulder roll carries it out
                # to the right and back in: an arc of about 6 cm at 0.26 m height. One wave per 2 s.
                "hz": 0.5,
                "keys": {
                    "right_shoulder_pitch": [(0.0, -1.10)],
                    "right_elbow": [(0.0, -1.00)],
                    "right_shoulder_roll": [(0.0, -0.25), (0.5, -1.05)],
                },
            },
            "dance_arms": {
                # both arms pump in alternation: one hand goes up and out while the other comes
                # down and in, elbows bending with them. Period 1.6 s. Legs stay on q0.
                "hz": 0.625,
                "keys": {
                    "right_shoulder_pitch": [(0.0, -1.40), (0.5, -0.20)],
                    "left_shoulder_pitch": [(0.0, -0.20), (0.5, -1.40)],
                    "right_shoulder_roll": [(0.0, -0.75), (0.5, -0.25)],
                    "left_shoulder_roll": [(0.0, 0.25), (0.5, 0.75)],
                    "right_elbow": [(0.0, -1.20), (0.5, -0.45)],
                    "left_elbow": [(0.0, -0.45), (0.5, -1.20)],
                },
            },
            "dance": {
                # dance_arms plus the legs: a knee bob twice per period (knee 0.15 -> 0.55 rad with
                # the hip and ankle pitch that keep the foot flat and under the hip, measured on the
                # MJCF: hip -0.24, ankle -0.28 per 0.5 rad of knee) and a lateral sway once per
                # period (hip roll +d, ankle roll -d on both legs shifts the trunk ~1.8 cm per 0.15
                # rad, feet flat). The trunk leans toward the raised arm.
                "hz": 0.625,
                "keys": {
                    "right_shoulder_pitch": [(0.0, -1.40), (0.5, -0.20)],
                    "left_shoulder_pitch": [(0.0, -0.20), (0.5, -1.40)],
                    "right_shoulder_roll": [(0.0, -0.75), (0.5, -0.25)],
                    "left_shoulder_roll": [(0.0, 0.25), (0.5, 0.75)],
                    "right_elbow": [(0.0, -1.20), (0.5, -0.45)],
                    "left_elbow": [(0.0, -0.45), (0.5, -1.20)],
                    "right_knee": [(0.0, 0.15), (0.25, 0.55), (0.5, 0.15), (0.75, 0.55)],
                    "left_knee": [(0.0, 0.15), (0.25, 0.55), (0.5, 0.15), (0.75, 0.55)],
                    "right_hip_pitch": [(0.0, -0.245), (0.25, -0.435), (0.5, -0.245), (0.75, -0.435)],
                    "left_hip_pitch": [(0.0, -0.245), (0.25, -0.435), (0.5, -0.245), (0.75, -0.435)],
                    "right_ankle_pitch": [(0.0, -0.06), (0.25, -0.30), (0.5, -0.06), (0.75, -0.30)],
                    "left_ankle_pitch": [(0.0, -0.06), (0.25, -0.30), (0.5, -0.06), (0.75, -0.30)],
                    "right_hip_roll": [(0.0, 0.033), (0.5, -0.207)],
                    "left_hip_roll": [(0.0, 0.207), (0.5, -0.033)],
                    "right_ankle_roll": [(0.0, -0.033), (0.5, 0.207)],
                    "left_ankle_roll": [(0.0, -0.207), (0.5, 0.033)],
                },
            },
        }
        if name not in presets:
            raise ValueError(f"unknown gesture {name!r}; known: {', '.join(presets)}")
        spec = presets[name]
        names = self.profile["joint_names"]
        index = {n: i for i, n in enumerate(names)}
        missing = [n for n in spec["keys"] if n not in index]
        if missing:
            raise ValueError(f"{name}: joints not on this robot: {missing}")
        if spec["hz"] and self.contract.extra_cmd_dim < 2:
            raise ValueError(f"{name}: a timed clip needs two extra observation channels "
                             f"(extra_cmd_dim >= 2), this robot has {self.contract.extra_cmd_dim}")
        self.gesture_spec = spec
        self.gesture_index = index
        self.gesture_ids = np.array([index[n] for n in spec["keys"]])
        self.p_zero_cmd = 1.0
        self.push_vel = 0.0
        # a radian of arm motion must not be smoothed away, and the walk curriculum would raise this
        self.w.action_rate = -0.02
        if not self.w.gesture_pose:
            self.w.gesture_pose = 8.0

    def _setup_pose_cmd(self) -> None:
        """Stand and track an 8-channel pose command (MAVLink teleop domain)."""
        if self.contract.extra_cmd_dim < 10:
            raise ValueError(f"pose_cmd needs extra_cmd_dim >= 10, this robot has {self.contract.extra_cmd_dim}")
        names = self.profile["joint_names"]
        index = {n: i for i, n in enumerate(names)}
        tracked = list(self.POSE_ARM_NAMES)
        for side in ("right", "left"):
            tracked += [f"{side}_knee", f"{side}_hip_pitch", f"{side}_ankle_pitch",
                        f"{side}_hip_roll", f"{side}_ankle_roll"]
        missing = [n for n in tracked if n not in index]
        if missing:
            raise ValueError(f"pose_cmd: joints not on this robot: {missing}")
        # keys dict keeps the green overlay and gesture_ids path working
        self.gesture_spec = {"mode": "pose_cmd", "hz": 0.0, "keys": {n: [(0.0, 0.0)] for n in tracked}}
        self.gesture_index = index
        self.gesture_ids = np.array([index[n] for n in tracked])
        self.pose_cmd = True
        self.pose_cmd_cur = np.zeros(8, np.float32)
        self.pose_cmd_tgt = np.zeros(8, np.float32)
        self.pose_cmd_filt = np.zeros(8, np.float32)
        self.pose_cmd_rate = 1.0
        self.pose_delay_buf: list[np.ndarray] = []
        self.pose_delay_ticks = 0
        self.next_pose_resample = 0
        self.pose_stream = "zero"  # zero | random | clip
        self.pose_clip_keys = None
        self.p_zero_cmd = 1.0
        self.push_vel = 0.0
        self.w.action_rate = -0.02
        if not self.w.gesture_pose:
            self.w.gesture_pose = 8.0
        if not self.w.base_height:
            # gentle: the leg joints are already tracked by the gesture term
            self.w.base_height = 0.5
            self.w.base_height_std_m = 0.02

    def _sample_pose_target(self) -> np.ndarray:
        """Random pose in the contract ranges, or zero (rest)."""
        lo, hi = self.POSE_RANGES[:, 0], self.POSE_RANGES[:, 1]
        return (lo + self.rng.random(8) * (hi - lo)).astype(np.float32)

    def _pose_from_clip(self, clip_name: str, phase: float) -> np.ndarray:
        """Convert a scripted clip at `phase` into the 8-channel pose command (offsets from q0)."""
        # local minimal key tables (same numbers as the dance/wave presets)
        clips = {
            "wave_right": {
                "right_shoulder_pitch": [(0.0, -1.10)],
                "right_elbow": [(0.0, -1.00)],
                "right_shoulder_roll": [(0.0, -0.25), (0.5, -1.05)],
            },
            "dance": {
                "right_shoulder_pitch": [(0.0, -1.40), (0.5, -0.20)],
                "left_shoulder_pitch": [(0.0, -0.20), (0.5, -1.40)],
                "right_shoulder_roll": [(0.0, -0.75), (0.5, -0.25)],
                "left_shoulder_roll": [(0.0, 0.25), (0.5, 0.75)],
                "right_elbow": [(0.0, -1.20), (0.5, -0.45)],
                "left_elbow": [(0.0, -0.45), (0.5, -1.20)],
                "right_knee": [(0.0, 0.15), (0.25, 0.55), (0.5, 0.15), (0.75, 0.55)],
                "left_knee": [(0.0, 0.15), (0.25, 0.55), (0.5, 0.15), (0.75, 0.55)],
                "right_hip_roll": [(0.0, 0.033), (0.5, -0.207)],
                "left_hip_roll": [(0.0, 0.207), (0.5, -0.033)],
            },
        }
        keys = clips[clip_name]
        q0 = np.asarray(self.contract.q0, dtype=np.float64)
        idx = self.gesture_index
        out = np.zeros(8, np.float32)
        for i, n in enumerate(self.POSE_ARM_NAMES):
            if n in keys:
                out[i] = self._clip_value(keys[n], phase) - q0[idx[n]]
        if "right_knee" in keys:
            rk = self._clip_value(keys["right_knee"], phase) - q0[idx["right_knee"]]
            lk = self._clip_value(keys["left_knee"], phase) - q0[idx["left_knee"]]
            out[6] = 0.5 * (rk + lk)
        if "right_hip_roll" in keys:
            # sway ≈ mean hip-roll offset (dance uses opposite signs that average near the sway)
            rr = self._clip_value(keys["right_hip_roll"], phase) - q0[idx["right_hip_roll"]]
            lr = self._clip_value(keys["left_hip_roll"], phase) - q0[idx["left_hip_roll"]]
            out[7] = float(np.clip(0.5 * (rr + lr), -0.15, 0.15))
        return np.clip(out, self.POSE_RANGES[:, 0], self.POSE_RANGES[:, 1]).astype(np.float32)

    def _apply_pose_offsets(self, ref: np.ndarray, pose: np.ndarray) -> None:
        """Write q0 + pose mapping (arms direct, legs from knee_bend/sway) into `ref`."""
        idx = self.gesture_index
        q0 = np.asarray(self.contract.q0, dtype=np.float64)
        for i, n in enumerate(self.POSE_ARM_NAMES):
            ref[idx[n]] = q0[idx[n]] + float(pose[i])
        k = float(pose[6])
        d = float(pose[7])
        for side in ("right", "left"):
            ref[idx[f"{side}_knee"]] = q0[idx[f"{side}_knee"]] + k
            ref[idx[f"{side}_hip_pitch"]] = q0[idx[f"{side}_hip_pitch"]] + self.POSE_KNEE_HIP * k
            ref[idx[f"{side}_ankle_pitch"]] = q0[idx[f"{side}_ankle_pitch"]] + self.POSE_KNEE_ANKLE * k
            ref[idx[f"{side}_hip_roll"]] = q0[idx[f"{side}_hip_roll"]] + d
            ref[idx[f"{side}_ankle_roll"]] = q0[idx[f"{side}_ankle_roll"]] - d

    def set_pose_external(self, values: np.ndarray | None) -> None:
        """GCS / play_policy injects an 8-vector (or None to resume env sampling)."""
        if values is None:
            self.pose_external = None
            return
        v = np.asarray(values, dtype=np.float32).reshape(-1)[:8]
        if v.size < 8:
            pad = np.zeros(8, np.float32)
            pad[:v.size] = v
            v = pad
        self.pose_external = np.clip(v, self.POSE_RANGES[:, 0], self.POSE_RANGES[:, 1]).astype(np.float32)

    def _advance_pose_cmd(self) -> None:
        """Move the episode pose toward its target (or follow an external/clip stream)."""
        dt = 1.0 / self.contract.rate_hz
        if self.pose_external is not None:
            self.pose_cmd_cur = self.pose_external.copy()
            self.pose_cmd_tgt = self.pose_cmd_cur.copy()
        elif self.pose_stream == "clip":
            hz = 0.5 if self.pose_clip_name == "wave_right" else 0.625
            self.clip_phase = (self.clip_phase + hz * dt) % 1.0
            self.pose_cmd_cur = self._pose_from_clip(self.pose_clip_name, self.clip_phase)
            self.pose_cmd_tgt = self.pose_cmd_cur.copy()
        else:
            if self.steps >= self.next_pose_resample:
                if self.pose_stream == "zero":
                    self.pose_cmd_tgt = np.zeros(8, np.float32)
                else:
                    self.pose_cmd_tgt = self._sample_pose_target()
                # sometimes snap (webcam jitter), usually ease at 0.3–3 rad/s
                if self.rng.random() < 0.15:
                    self.pose_cmd_cur = self.pose_cmd_tgt.copy()
                    self.pose_cmd_rate = 10.0
                else:
                    self.pose_cmd_rate = float(self.rng.uniform(0.3, 3.0))
                self.next_pose_resample = self.steps + int(self.rng.uniform(1.0, 4.0) * self.contract.rate_hz)
            delta = self.pose_cmd_tgt - self.pose_cmd_cur
            step = self.pose_cmd_rate * dt
            nrm = float(np.linalg.norm(delta))
            if nrm <= step:
                self.pose_cmd_cur = self.pose_cmd_tgt.copy()
            else:
                self.pose_cmd_cur = self.pose_cmd_cur + delta * (step / nrm)
        # firmware-style low-pass; observation uses the delayed filtered value
        alpha = min(1.0, dt / self.POSE_TAU_S)
        self.pose_cmd_filt += alpha * (self.pose_cmd_cur - self.pose_cmd_filt)
        self.pose_delay_buf.append(self.pose_cmd_filt.copy())
        self.pose_delay_buf = self.pose_delay_buf[-(self.pose_delay_ticks + 1):]

    def _pose_observed(self) -> np.ndarray:
        buf = self.pose_delay_buf
        delayed = buf[max(0, len(buf) - 1 - self.pose_delay_ticks)] if buf else self.pose_cmd_filt
        noise = self.rng.normal(0.0, 0.01, 8).astype(np.float32)
        return np.clip(delayed + noise, self.POSE_RANGES[:, 0], self.POSE_RANGES[:, 1]).astype(np.float32)

    @staticmethod
    def _clip_value(keys: list[tuple[float, float]], phase: float) -> float:
        """Periodic cosine interpolation between keyframes (phase, value) sorted by phase."""
        if len(keys) == 1:
            return keys[0][1]
        n = len(keys)
        for k in range(n):
            p0, v0 = keys[k]
            p1, v1 = keys[(k + 1) % n]
            span = (p1 - p0) % 1.0 or 1.0
            t = (phase - p0) % 1.0
            if t <= span:
                s = 0.5 - 0.5 * math.cos(math.pi * t / span)
                return v0 + (v1 - v0) * s
        return keys[-1][1]

    def _gesture_extra(self) -> np.ndarray | None:
        if self.gesture_spec is None or self.contract.extra_cmd_dim < 2:
            return None
        extra = np.zeros(self.contract.extra_cmd_dim, np.float32)
        if self.pose_cmd:
            # clock channels stay at zero; pose fills extra[2:10]
            if self.contract.extra_cmd_dim >= 10:
                extra[2:10] = self._pose_observed()
            return extra
        if self.gesture_spec["hz"]:
            extra[0] = math.sin(2 * math.pi * self.clip_phase)
            extra[1] = math.cos(2 * math.pi * self.clip_phase)
        return extra

    def _gesture_reference(self, q: np.ndarray) -> np.ndarray:
        ref = np.array(self.contract.q0, dtype=np.float64, copy=True)
        spec = self.gesture_spec
        if spec is None:
            return ref
        if self.pose_cmd:
            self._apply_pose_offsets(ref, self.pose_cmd_cur)
            return ref
        for n, keys in spec["keys"].items():
            ref[self.gesture_index[n]] = self._clip_value(keys, self.clip_phase)
        return ref

    # ------------------------------------------------------------------ helpers
    def _is_under(self, body: int, root: int) -> bool:
        m = self.model
        while body > 0:
            if body == root:
                return True
            body = int(m.body_parentid[body])
        return body == root

    def _next_resample(self) -> int:
        if not self.resample_s:
            return 10 ** 9
        return self.steps + int(self.rng.uniform(*self.resample_s) * self.contract.rate_hz)

    def _sample_command(self) -> np.ndarray:
        if self.gesture_spec is not None:
            return np.zeros(3, np.float32)
        if self.rng.random() < self.p_zero_cmd:
            return np.zeros(3, np.float32)
        r = self.cmd_ranges
        if self.rng.random() < self.p_single_axis:
            # one axis, one sign, between half and full range: every direction gets clear commands
            if self.single_axis_p is not None:
                k = int(self.rng.choice(6, p=self.single_axis_p))
                axis, positive = k // 2, k % 2 == 0
            else:
                axis, positive = int(self.rng.integers(3)), self.rng.random() < 0.5
            lo, hi = r[("vx", "vy", "wz")[axis]]
            limit = hi if positive else lo
            t = np.zeros(3, np.float32)
            t[axis] = limit * self.rng.uniform(0.5, 1.0)
            return dc.saturate_twist(self.contract, t)
        t = np.array([self.rng.uniform(*r["vx"]), self.rng.uniform(*r["vy"]), self.rng.uniform(*r["wz"])],
                     np.float32)
        if self.rng.random() < self.p_no_vy:
            t[1] = 0.0   # Rover autopilot modes provide no lateral speed
        return dc.saturate_twist(self.contract, t)

    def _apply_target(self, q_wire: np.ndarray) -> None:
        d = self.data
        if self.bam is not None:
            # actuators outside the policy (e.g. Microban's head) keep their reset target
            self.bam.q_target[self.act_idx] = q_wire
            self.bam.update()
            return
        if self.actuator == "pd":
            q = d.qpos[self.qpos_idx]
            qd = d.qvel[self.qvel_idx]
            d.ctrl[self.act_idx] = self.kp * (q_wire - q) - self.kd * qd
        else:
            d.ctrl[self.act_idx] = q_wire

    def _imu_frd(self) -> tuple[np.ndarray, np.ndarray]:
        d = self.data
        gyro_flu = d.sensordata[self.gyro_adr:self.gyro_adr + 3].copy()
        acc_flu = d.sensordata[self.acc_adr:self.acc_adr + 3].copy()
        gyro_flu += self.rng.normal(0.0, self.gyro_noise, 3)
        acc_flu += self.rng.normal(0.0, self.accel_noise, 3)
        return dc.frd_to_flu(gyro_flu), dc.frd_to_flu(acc_flu)

    def tilt_deg(self) -> float:
        w, x, y, z = self.data.xquat[self.trunk_id]
        return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))

    def _base_vel_body(self) -> tuple[np.ndarray, float]:
        d = self.data
        q = d.xquat[self.trunk_id]
        v_world = d.qvel[self.free_qvel:self.free_qvel + 3]
        v_body = _quat_rotate_inverse(q, v_world)
        wz = float(d.qvel[self.free_qvel + 5])   # free joint angular velocity is in the body frame
        return v_body, wz

    # ------------------------------------------------------------------ API
    def reset(self) -> np.ndarray:
        c = self.contract
        d, m = self.data, self.model
        mujoco.mj_resetData(m, d)
        d.qpos[self.free_qpos:self.free_qpos + 3] = [0.0, 0.0, self.home_z]
        d.qpos[self.free_qpos + 3:self.free_qpos + 7] = [1, 0, 0, 0]
        d.qpos[self.qpos_idx] = c.q0
        d.qvel[:] = 0.0
        if self.bam is not None:
            self.bam.q_target[:] = 0.0
            self.bam.q_target[self.act_idx] = c.q0
            self.bam.last_ts = d.time
            d.ctrl[:] = 0.0
        mujoco.mj_forward(m, d)
        self.gravity = dc.GravityFilter(c.att_tau)
        self.a_prev = dc.stand_action(c)
        self.q_wire = dc.apply_action(c, self.a_prev)[1]
        self.command = self._sample_command()
        # a random start phase: the policy must join the clip anywhere, as on the robot
        self.clip_phase = float(self.rng.random()) if self.gesture_spec else 0.0
        if self.pose_cmd:
            # 15% rest, 20% clip-as-stream, rest random poses (as the webcam will send)
            r = float(self.rng.random())
            if self.pose_external is not None:
                self.pose_stream = "external"
                self.pose_cmd_cur = self.pose_external.copy()
            elif r < 0.15:
                self.pose_stream = "zero"
                self.pose_cmd_cur = np.zeros(8, np.float32)
            elif r < 0.35:
                self.pose_stream = "clip"
                self.pose_clip_name = "wave_right" if self.rng.random() < 0.5 else "dance"
                self.pose_cmd_cur = self._pose_from_clip(self.pose_clip_name, self.clip_phase)
            else:
                self.pose_stream = "random"
                self.pose_cmd_cur = self._sample_pose_target()
            self.pose_cmd_tgt = self.pose_cmd_cur.copy()
            self.pose_cmd_filt = self.pose_cmd_cur.copy()
            self.pose_delay_ticks = int(self.rng.integers(0, 4))
            self.pose_delay_buf = [self.pose_cmd_filt.copy()]
            self.pose_cmd_rate = float(self.rng.uniform(0.3, 3.0))
            self.next_pose_resample = int(self.rng.uniform(1.0, 4.0) * self.contract.rate_hz)
        self.steps = 0
        self._ep_terms: dict[str, float] = {}
        self._ep_return = 0.0
        self._last_raw_action = np.zeros(c.n_joints, np.float32)
        self.twist_filt = np.zeros(4)
        nf = len(self.feet)
        self.foot_air = np.zeros(nf)
        self.foot_contact_prev = np.ones(nf, bool)
        self.foot_pos_prev = np.array([d.site_xpos[s].copy() for s, _ in self.feet]).reshape(nf, 3)
        self.foot_z0 = self.foot_pos_prev[:, 2].copy() if nf else np.zeros(0)
        self.foot_peak = np.zeros(nf)
        self.foot_air_prev = np.zeros(nf)
        self.foot_air_prev_step = np.zeros(nf)
        self.last_touchdown = -1
        self.last_footfall = -1
        self.foot_air_prev_seq = np.zeros(nf)
        self.last_air_t = np.zeros(nf)
        self.last_contact_t = np.zeros(nf)
        self.foot_contact_t = np.zeros(nf)
        self.foot_air_prev_var = np.zeros(nf)
        self.foot_contact_prev_var = np.ones(nf, bool)
        self.last_step_peak = 0.0
        self.last_step_air = 0.0
        self.next_resample = self._next_resample()
        self.next_push = (int(self.rng.uniform(*self.push_interval) * c.rate_hz) if self.push_interval
                          else 10 ** 9)
        self._obs_hist: list[np.ndarray] = []
        # disarmed phase: the trunk is held (as the SITL plant pins it before arming) while the
        # gravity filter converges; the policy starts from the upright stand like after "arm"
        # A held board is static: gyro 0, accelerometer = gravity only. Overwriting the root pose is
        # not a physical constraint, so the simulated accelerometer would read spurious accelerations.
        root = d.qpos[self.free_qpos:self.free_qpos + 7].copy()
        for _ in range(dc.AUTOPILOT_LOOP_HZ // 10):
            self._apply_target(self.q_wire)
            mujoco.mj_step(m, d)
            d.qpos[self.free_qpos:self.free_qpos + 7] = root
            d.qvel[self.free_qvel:self.free_qvel + 6] = 0.0
            mujoco.mj_forward(m, d)
            up_flu = -_quat_rotate_inverse(d.xquat[self.trunk_id], np.array([0.0, 0.0, -1.0]))
            self.gravity.update(np.zeros(3), dc.frd_to_flu(up_flu * dc.GRAVITY_MSS), SIM_DT)
        return self._observe()

    def _gravity_obs(self) -> np.ndarray:
        """Gravity direction in the body frame (FLU, -z when upright) as the autopilot will supply it.

        ap_imu_filter (default, NNM_ATT_SRC 0): the firmware complementary filter on gyro and
        accelerometer with att_tau. ahrs (NNM_ATT_SRC 1): the EKF attitude, modelled as the true
        gravity tilted by a small random angle (ahrs_noise_rad, default 0.01). A 30 kg humanoid walking
        at 0.5 m/s tilts the complementary estimate by ~7 deg and the Booster policy falls; with the
        AHRS it tracks the command."""
        if self.gravity_source != "ahrs":
            return self.gravity.gravity_flu()
        g = _quat_rotate_inverse(self.data.xquat[self.trunk_id], np.array([0.0, 0.0, -1.0]))
        if self.ahrs_noise:
            g = g + self.rng.normal(0.0, self.ahrs_noise, 3)
            g = g / np.linalg.norm(g)
        return g.astype(np.float32)

    def _observe(self) -> np.ndarray:
        c = self.contract
        d = self.data
        gyro_frd, _ = self._imu_frd()
        obs = dc.build_obs(c, dc.frd_to_flu(gyro_frd), self._gravity_obs(),
                           d.qpos[self.qpos_idx], d.qvel[self.qvel_idx], self.a_prev, self.command,
                           self._gesture_extra())
        self._obs_hist.append(obs)
        self._obs_hist = self._obs_hist[-(self.obs_delay_max + 1):]
        delay = int(self.rng.integers(0, self.obs_delay_max + 1)) if self.obs_delay_max else 0
        return self._obs_hist[max(0, len(self._obs_hist) - 1 - delay)].copy()

    def step(self, action: np.ndarray):
        c = self.contract
        self.a_prev, self.q_wire = dc.apply_action(c, action)
        m, d = self.model, self.data
        for _ in range(c.loop_steps_per_policy):
            self._apply_target(self.q_wire)
            mujoco.mj_step(m, d)
            g, a = self._imu_frd()
            self.gravity.update(g, a, SIM_DT)
        self.steps += 1
        if self.steps >= self.next_resample:
            self.command = self._sample_command()
            self.next_resample = self._next_resample()
        if self.gesture_spec is not None:
            self.command = np.zeros(3, np.float32)
            if self.pose_cmd:
                self._advance_pose_cmd()
            elif self.gesture_spec["hz"]:
                self.clip_phase = (self.clip_phase + self.gesture_spec["hz"] / c.rate_hz) % 1.0
        if self.steps >= self.next_push:
            d.qvel[self.free_qvel:self.free_qvel + 2] += self.rng.uniform(-self.push_vel, self.push_vel, 2)
            self.next_push = self.steps + int(self.rng.uniform(*self.push_interval) * c.rate_hz)
        reward, info = self._reward(action)
        fell = self.tilt_deg() > self.fall_tilt_deg
        if self.terminate_illegal and not fell and self._illegal_contact():
            fell = True
        if self.posture_limits and not fell and self._posture_violated():
            fell = True
        if fell and self.w.termination:
            info["terms"]["termination"] = self.w.termination
            reward += self.w.termination
        if self.w.only_positive:
            reward = max(reward, 0.0)
        timeout = self.steps >= self.max_steps
        for k, v in info["terms"].items():
            self._ep_terms[k] = self._ep_terms.get(k, 0.0) + v
        self._ep_return += reward
        if fell or timeout:
            # rsl_rl convention: Episode_Reward/<term> = episode sum / max episode length (s)
            info["episode"] = {k: v / self.episode_s for k, v in self._ep_terms.items()}
            info["episode_return"] = self._ep_return        # the reward the policy is trained on
            info["reason"] = "fell_over" if fell else "time_out"
        obs = self._observe()
        return obs, reward, fell, timeout, info

    def _feet_contact(self) -> np.ndarray:
        d, m = self.data, self.model
        touch = np.zeros(len(self.feet), bool)
        index = {bid: i for i, (_, bid) in enumerate(self.feet)}
        for k in range(d.ncon):
            con = d.contact[k]
            g1, g2 = con.geom1, con.geom2
            if g1 in self.floor_geoms:
                g = g2
            elif g2 in self.floor_geoms:
                g = g1
            else:
                continue
            if self.foot_geom_index:
                if g in self.foot_geom_index:
                    touch[self.foot_geom_index[g]] = True
            elif m.geom_bodyid[g] in index:
                touch[index[m.geom_bodyid[g]]] = True
        return touch

    def _illegal_contact(self) -> bool:
        """Trunk, head, hip or thigh on the floor (mjlab / unitree_rl_lab illegal_contact termination)."""
        d = self.data
        for k in range(d.ncon):
            con = d.contact[k]
            g1, g2 = con.geom1, con.geom2
            g = g2 if g1 in self.floor_geoms else (g1 if g2 in self.floor_geoms else -1)
            if g >= 0 and self.illegal_geom[g]:
                return True
        return False

    def _posture_violated(self) -> bool:
        d = self.data
        h = self.posture_limits.get("min_height_m")
        if h is not None and float(d.qpos[self.free_qpos + 2]) < h:
            return True
        if self.max_dev is not None:
            return bool(np.any(np.abs(d.qpos[self.qpos_idx] - self.contract.q0) > self.max_dev))
        return False

    def privileged(self) -> np.ndarray:
        """Critic-only state (asymmetric actor-critic, as the Playground Go1 privileged_state): true base
        twist, true gravity, trunk height, foot contacts and air times. Never reaches the policy."""
        d = self.data
        v_body, wz = self._base_vel_body()
        g_true = _quat_rotate_inverse(d.xquat[self.trunk_id], np.array([0.0, 0.0, -1.0]))
        z = float(d.qpos[self.free_qpos + 2]) - self.home_z
        parts = [v_body, [wz], g_true, [10.0 * z]]
        if self.feet:
            parts += [self._feet_contact().astype(float), np.clip(self.foot_air, 0.0, 1.0)]
        return np.concatenate([np.asarray(p, float) for p in parts]).astype(np.float32)

    def _undesired_contacts(self) -> int:
        """Robot geoms other than the feet touching the floor (legged_gym collision penalty)."""
        d = self.data
        n = 0
        for k in range(d.ncon):
            con = d.contact[k]
            g1, g2 = con.geom1, con.geom2
            g = g2 if g1 in self.floor_geoms else (g1 if g2 in self.floor_geoms else -1)
            if g >= 0 and self.robot_geom[g] and g not in self.foot_geom_index:
                n += 1
        return n

    def _gait_terms(self, dt: float) -> dict:
        """feet_air_time, foot_clearance and foot_slip of the MicroDuck mjlab velocity task."""
        w = self.w
        d = self.data
        contact = self._feet_contact()
        pos = np.array([d.site_xpos[s] for s, _ in self.feet])
        vel_xy = np.linalg.norm((pos - self.foot_pos_prev)[:, :2], axis=1) / dt
        self.foot_pos_prev = pos.copy()
        cmd_norm = float(np.linalg.norm(self.command[:2]) + abs(self.command[2]))
        moving = float(cmd_norm > (w.walking_threshold if w.air_time_mode == "in_range" else 0.05))
        first = contact & ~self.foot_contact_prev
        self.foot_air = np.where(contact, 0.0, self.foot_air + dt)
        if w.air_time_mode == "in_range":
            # mjlab feet_air_time(threshold_min, threshold_max): feet airborne for a time in range, every step
            air = float(np.sum((self.foot_air > w.air_time_min_s) & (self.foot_air < w.air_time_max_s)))
        else:
            air = 0.0
            for i in np.flatnonzero(first):
                flight = self.foot_air_prev[i] if hasattr(self, "foot_air_prev") else 0.0
                if flight < w.air_time_debounce_s:
                    continue
                air += min(flight, w.air_time_max_s) - w.air_time_min_s
        self.foot_air_prev = self.foot_air.copy()
        self.foot_contact_prev = contact
        # height above the foot's last stance position (flat floor: the mjlab height scan)
        self.foot_z0 = np.where(contact, pos[:, 2], self.foot_z0)
        height = pos[:, 2] - self.foot_z0
        # mjlab feet_clearance: |h - target| weighted by foot speed, only with a command
        clearance = float(np.sum(np.abs(height - w.swing_height_m) * vel_xy * ~contact)) * moving
        # mjlab feet_swing_height: peak height error at landing
        swing = float(np.sum(((self.foot_peak / w.swing_height_m - 1.0) ** 2) * first)) * moving
        peak_at_touch = self.foot_peak.copy()
        self.foot_peak = np.where(contact, 0.0, np.maximum(self.foot_peak, height))
        slip = float(np.sum((vel_xy ** 2) * contact)) * moving
        out = {"air_time": w.air_time * air * moving * dt,
               "foot_clearance": w.foot_clearance * clearance * dt,
               "foot_slip": w.foot_slip * slip * dt}
        if w.foot_hold:
            held = int(np.sum((~contact) & (self.foot_air > w.air_time_max_s)))
            out["foot_hold"] = w.foot_hold * held * moving * dt
        if w.single_stance:
            # one foot down, the other airborne for less than air_time_max_s (a step, not standing on one leg)
            step_now = contact.sum() == len(contact) - 1 and bool(np.all(self.foot_air[~contact] < w.air_time_max_s))
            out["single_stance"] = w.single_stance * float(step_now) * moving * dt
        if w.foot_alternation or w.foot_symmetry:
            alt = sym = 0.0
            for i in np.flatnonzero(first):
                air_i, peak_i = float(self.foot_air_prev_step[i]), float(peak_at_touch[i])
                # a step counts only with a real swing: long enough and high enough (a policy satisfied
                # alternation with 1 cm shuffles of one foot while the other took 5 cm steps)
                if air_i < w.alternation_min_air_s or peak_i < w.alternation_min_height_frac * w.swing_height_m:
                    continue
                if self.last_touchdown >= 0:
                    alt += 1.0 if i != self.last_touchdown else -1.0
                    if i != self.last_touchdown:
                        # left/right symmetry: this step against the previous step of the other foot
                        sym += ((peak_i - self.last_step_peak) / w.swing_height_m) ** 2
                        sym += ((air_i - self.last_step_air) / w.air_time_max_s) ** 2
                self.last_touchdown = int(i)
                self.last_step_peak, self.last_step_air = peak_i, air_i
            if w.foot_alternation:
                out["foot_alternation"] = w.foot_alternation * alt * moving
            if w.foot_symmetry:
                out["foot_symmetry"] = w.foot_symmetry * sym * moving
        self.foot_air_prev_step = self.foot_air.copy()
        if w.foot_lift:
            # only during a real step: this foot airborne for less than air_time_max_s while the other
            # stands. Without the bound a policy kept one foot up for good and leaned on the other leg.
            stepping = (~contact) & (self.foot_air < w.air_time_max_s) & (contact.sum() == len(contact) - 1)
            lift = np.clip(height / w.swing_height_m, 0.0, 1.0) * stepping
            out["foot_lift"] = w.foot_lift * float(np.sum(lift)) * moving * dt
        if w.foot_swing_height:
            out["foot_swing_height"] = w.foot_swing_height * swing * dt
        n_feet = len(contact)
        if w.air_time_variance:
            lifted = ~contact & self.foot_contact_prev_var
            self.last_air_t = np.where(first, np.minimum(self.foot_air_prev_var, 0.5), self.last_air_t)
            self.last_contact_t = np.where(lifted, np.minimum(self.foot_contact_t, 0.5), self.last_contact_t)
            self.foot_contact_t = np.where(contact, self.foot_contact_t + dt, 0.0)
            self.foot_air_prev_var = self.foot_air.copy()
            self.foot_contact_prev_var = contact.copy()
            out["air_time_variance"] = w.air_time_variance * float(
                np.var(self.last_air_t) + np.var(self.last_contact_t)) * dt
        if w.three_stance and n_feet == 4:
            one_up = contact.sum() == 3 and bool(np.all(self.foot_air[~contact] < w.air_time_max_s))
            out["three_stance"] = w.three_stance * float(one_up) * moving * dt
        if w.under_stance:
            out["under_stance"] = w.under_stance * float(contact.sum() < w.under_stance_feet) * dt
        if w.all_stance and n_feet == 4:
            out["all_stance"] = w.all_stance * float(contact.all()) * moving * dt
        if w.footfall_sequence and self.footfall_next:
            seq = 0.0
            backward = self.command[0] < 0
            for i in np.flatnonzero(first):
                if (self.foot_air_prev_seq[i] < w.alternation_min_air_s
                        or peak_at_touch[i] < w.alternation_min_height_frac * w.swing_height_m):
                    continue
                last = self.last_footfall
                if last >= 0:
                    expected = self.footfall_prev[last] if backward else self.footfall_next[last]
                    seq += 1.0 if i == expected else (-0.5 if i == last else 0.0)
                self.last_footfall = int(i)
            out["footfall_sequence"] = w.footfall_sequence * seq * moving
        self.foot_air_prev_seq = self.foot_air.copy()
        if w.trot and self.trot_pairs:
            (a, b), (c, e) = self.trot_pairs
            trot_now = contact[a] == contact[b] and contact[c] == contact[e] and contact[a] != contact[c]
            out["trot"] = w.trot * float(trot_now) * moving * dt
        return out

    def set_curriculum(self, action_rate: float | None = None, standing_envs: float | None = None,
                       penalty_scale: float | None = None) -> None:
        """MicroDuck curriculum knobs: action_rate_l2 weight and share of zero-command (standing) episodes."""
        if self.gesture_spec is not None:
            return
        if penalty_scale is not None:
            self.penalty_scale = float(penalty_scale)
        if action_rate is not None:
            self.w.action_rate = float(action_rate)
        if standing_envs is not None:
            self.p_zero_cmd = float(standing_envs)

    def _reward(self, action) -> tuple[float, dict]:
        w = self.w
        d, m = self.data, self.model
        v_body, wz = self._base_vel_body()
        cmd = self.command
        w_body = d.qvel[self.free_qvel + 3:self.free_qvel + 6]       # free-joint angular velocity, body frame
        # mjlab track_linear_velocity / track_angular_velocity
        if w.tracking_filter_s:
            alpha = min(1.0, (1.0 / self.contract.rate_hz) / w.tracking_filter_s)
            self.twist_filt += alpha * (np.array([*v_body, wz]) - self.twist_filt)
            v_track, wz_track = self.twist_filt[:3], float(self.twist_filt[3])
        else:
            v_track, wz_track = v_body, wz
        lin_err = float(np.sum((cmd[:2] - v_track[:2]) ** 2)) + w.tracking_lin_z * float(v_track[2] ** 2)
        ang_err = float((cmd[2] - wz_track) ** 2) + w.tracking_ang_rp * float(np.sum(w_body[:2] ** 2))
        q = d.qpos[self.qpos_idx]
        q_ref = self._gesture_reference(q)
        if w.upright_std:
            g_true = _quat_rotate_inverse(d.xquat[self.trunk_id], np.array([0.0, 0.0, -1.0]))
            up = math.exp(-float(np.sum(g_true[:2] ** 2)) / w.upright_std ** 2)
        else:
            up = -self._gravity_obs()[2]                   # 1 when upright
        moving = float(np.linalg.norm(cmd[:2]) + abs(cmd[2]))
        # arm joints are scored only by the gesture term. Leaving them in the body pose zeroed that
        # term a radian away (exp of a huge mean) and removed its gradient, so standing still paid
        # in full and the arm never moved.
        pose_sel = np.ones(q.shape[0], dtype=bool)
        if self.gesture_ids is not None:
            pose_sel[self.gesture_ids] = False
        q_pose = q[pose_sel]
        ref_pose = np.asarray(self.contract.q0, dtype=np.float64)[pose_sel]
        if self.pose_std_stand is not None:
            std = (self.pose_std_stand if moving < w.walking_threshold else self.pose_std_walk)[pose_sel]
            pose = math.exp(-float(np.mean(((q_pose - ref_pose) / std) ** 2)))
        else:
            pose = math.exp(-float(np.sum((q_pose - ref_pose) ** 2)))
        if w.pose_l2 and self.pose_std_stand is not None:
            std_l2 = (self.pose_std_stand if moving < w.walking_threshold else self.pose_std_walk)[pose_sel]
            pose_l2 = float(np.mean(((q_pose - ref_pose) / std_l2) ** 2))
        rate = float(np.sum((np.asarray(action) - self._last_raw_action) ** 2))
        self._last_raw_action = np.asarray(action, dtype=np.float32).copy()
        dt = 1.0 / self.contract.rate_hz
        terms = {
            "track_linear_velocity": w.track_lin_vel * math.exp(-lin_err / w.tracking_sigma) * dt,
            "track_angular_velocity": w.track_ang_vel * math.exp(-ang_err / w.tracking_sigma_ang) * dt,
            "upright": w.upright * up * dt,
            "pose": w.pose * pose * dt,
            "action_rate_l2": w.action_rate * rate * dt,
        }
        if w.alive:
            terms["alive"] = w.alive * dt
        if w.pose_l2 and self.pose_std_stand is not None:
            terms["pose_l2"] = w.pose_l2 * pose_l2 * dt
        if w.orientation:
            g_t = _quat_rotate_inverse(d.xquat[self.trunk_id], np.array([0.0, 0.0, -1.0]))
            terms["orientation"] = w.orientation * float(np.sum(g_t[:2] ** 2)) * dt
        if w.stand_still and float(np.linalg.norm(cmd)) < 0.01:
            terms["stand_still"] = w.stand_still * float(np.sum(np.abs(q - q_ref))) * dt
        if w.gesture_pose and self.gesture_ids is not None:
            # linear in the joint error: exp(-err²) with std 0.12 was already 0 at 1 rad, so the
            # arm had no gradient. 1 at the pose, 0 at 1.5 rad mean error, negative beyond that.
            # motion tracking: mean joint error to the reference pose of this phase. Linear so
            # the gradient reaches a lowered arm; a tighter exp() bonus rewards following the
            # clip closely (DeepMimic pose term), which is what makes the wave visible.
            err = q[self.gesture_ids] - q_ref[self.gesture_ids]
            gap = float(np.mean(np.abs(err)))
            terms["gesture"] = w.gesture_pose * (1.0 - gap / 1.5) * dt
            terms["gesture_track"] = 0.5 * w.gesture_pose * math.exp(-float(np.mean(err ** 2)) / 0.02) * dt
        if w.body_ang_vel:
            terms["body_ang_vel"] = w.body_ang_vel * float(np.sum(w_body[:2] ** 2)) * dt
        if w.angular_momentum:
            mujoco.mj_subtreeVel(m, d)
            terms["angular_momentum"] = w.angular_momentum * float(np.sum(d.subtree_angmom[self.trunk_id] ** 2)) * dt
        if w.no_progress:
            lack = []
            c_xy = float(np.linalg.norm(cmd[:2]))
            if c_xy > w.no_progress_min_cmd:
                lack.append(1.0 - float(np.clip(np.dot(v_track[:2], cmd[:2]) / c_xy ** 2, 0.0, 1.0)))
            if abs(cmd[2]) > 3 * w.no_progress_min_cmd:
                lack.append(1.0 - float(np.clip(wz_track / cmd[2], 0.0, 1.0)))
            if lack:
                terms["no_progress"] = w.no_progress * float(np.mean(lack)) * dt
        if w.joint_vel:
            terms["joint_vel"] = w.joint_vel * float(np.sum(d.qvel[self.qvel_idx] ** 2)) * dt
        if w.undesired_contacts:
            terms["undesired_contacts"] = w.undesired_contacts * self._undesired_contacts() * dt
        if w.base_height:
            target_z = w.base_height_target_m
            if self.pose_cmd:
                # knee_bend with the hip/ankle ratios keeps the foot flat: the trunk drops only
                # 0.8 cm at 0.55 rad (mj_kinematics on the MJCF), i.e. 1.45 cm/rad
                target_z = float(self.home_z) - self.POSE_KNEE_DROP_M_PER_RAD * float(self.pose_cmd_cur[6])
            dz = float(d.qpos[self.free_qpos + 2]) - target_z
            terms["base_height"] = w.base_height * math.exp(-(dz / w.base_height_std_m) ** 2) * dt
        if w.joint_torque:
            tau = d.actuator_force[self.act_idx] / self.force_limit
            terms["joint_torque"] = w.joint_torque * float(np.sum(tau ** 2)) * dt
        if w.dof_pos_limits:
            out = np.clip(self.soft_lo - q, 0, None) + np.clip(q - self.soft_hi, 0, None)
            terms["dof_pos_limits"] = w.dof_pos_limits * float(np.sum(out)) * dt
        if w.self_collisions:
            n_self = 0
            for k in range(d.ncon):
                con = d.contact[k]
                if self.robot_geom[con.geom1] and self.robot_geom[con.geom2]:
                    n_self += 1
            terms["self_collisions"] = w.self_collisions * n_self * dt
        if self.feet:
            terms.update(self._gait_terms(dt))
        if self.penalty_scale != 1.0:
            # curriculum: shaping penalties start small so a flailing policy is not better off falling;
            # action_rate has its own curriculum
            for k, v in terms.items():
                if v < 0.0 and k != "action_rate_l2":
                    terms[k] = v * self.penalty_scale
        info = {"terms": terms, "error_vel_xy": math.sqrt(lin_err), "error_vel_yaw": math.sqrt(ang_err),
                "vx": float(v_body[0]), "upright": float(up)}
        return sum(terms.values()), info


def load_ppo_config(robot_id: str) -> dict[str, Any]:
    import yaml

    from common import robot_dir
    p = robot_dir(robot_id) / "robot" / "ppo.yaml"
    if not p.is_file():
        raise FileNotFoundError(p)
    return yaml.safe_load(p.read_text())


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="Step the ArduPilot-contract env at zero action")
    ap.add_argument("--robot", required=True)
    ap.add_argument("--seconds", type=float, default=3.0)
    args = ap.parse_args()
    env = NNMixerEnv(args.robot, load_ppo_config(args.robot))
    obs = env.reset()
    n = int(args.seconds * env.contract.rate_hz)
    for _ in range(n):
        obs, r, fell, timeout, _ = env.step(np.zeros(env.contract.n_joints, np.float32))
        if fell:
            break
    print(f"robot={args.robot} obs_dim={obs.size} steps={env.steps} tilt={env.tilt_deg():.1f} deg "
          f"gravity_flu={np.round(env.gravity.gravity_flu(), 3).tolist()}")
