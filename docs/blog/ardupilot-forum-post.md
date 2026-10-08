# ArduPilot Neural Mixer: a reinforcement-learning walking policy running inside ArduRover (bipeds, quadrupeds, Pixhawk 6C)

<!--
Post for discuss.ardupilot.org (Discourse markdown).
Paste everything below this comment. Image and video URLs point at the main branch of the public repo;
Discourse turns a bare .mp4 URL on its own line into an inline player.
Suggested category: Blog (or Rover). Tags: rover, ai, machine-learning, legged, sitl, pixhawk6c
-->

![ArduPilot Neural Mixer](https://raw.githubusercontent.com/virtualrobotix/Ardupilot-Neural-Mixer/main/docs/blog/img/hero.jpg)

Hi all. Over the last months I have been working on something a bit unusual for ArduPilot: **legs**. Not a companion computer driving servos through MAVLink, but a neural network that walks a robot **from inside the ArduRover firmware**, as a normal scheduler task, reading ArduPilot's own IMU, RC and flight modes and writing ArduPilot's own servo outputs.

The project is open and on GitHub: **[virtualrobotix/Ardupilot-Neural-Mixer](https://github.com/virtualrobotix/Ardupilot-Neural-Mixer)** (tools, simulator, robot catalog, docs), with the firmware in the **[`microduck-ppo` branch of virtualrobotix/ardupilot](https://github.com/virtualrobotix/ardupilot/tree/microduck-ppo)** (library `AP_NNMixer`, two Pixhawk 6C board targets). This post is a tour of what works today, what I learned along the way, and where I would love help from this community.

## TL;DR

- `AP_NNMixer` is a new ArduRover library. It runs a PPO locomotion policy (an MLP, 512-256-128, ELU) at **50 Hz**, in plain C, with **int8 weights loaded from the microSD card**. No companion computer, no ROS.
- **MicroDuck** (the Pollen Robotics open-source biped, 14 servos) stands, walks and turns in SITL with a MuJoCo plant, and the same network runs on a real **Pixhawk 6C Mini**: **4.9 ms per forward pass, 33 % CPU**, 15 s standing hardware-in-the-loop.
- One firmware, **18 robots** in the catalog (bipeds, quadrupeds, a wheeled biped, a hexapod). Topology comes from `robot.bin` at boot; policies are `.nnm` files, two int8 slots in RAM, hot switch with a 0.5 s blend.
- The MuJoCo training environment **is** the ArduPilot deployment contract (same gravity filter, same action clipping, same PWM quantization) and the actor trains on the int8 grid from the first iteration, so the exported file is the trained network. Firmware vs training parity: 2.7e-7.
- Beyond walking: **gestures by clip imitation** (wave, dance), **pose teleoperation over MAVLink** from a webcam, and a humanoid that **gets up on its own after a fall**, with a small state machine in the firmware.
- Missing for a real robot: a **servo-bus backend** (Dynamixel / Feetech) for joint feedback. That is the next step and the part where I need the most input.

## Why inside the autopilot

Legged robots in research run their policy on a Jetson or a laptop. For small, cheap robots (a 350 g humanoid, a 3D-printed quadruped) that is heavier and more expensive than the robot itself. An STM32H7 autopilot already has the IMU, the RC link, the servo outputs, the SD card, the parameters, the logging and a ground station ecosystem. The only missing piece is the controller, and a 200 KB int8 MLP fits.

```
 GCS / MAVProxy ──MAVLink──▶ ArduRover ─ RC, modes ─▶ AP_NNMixer (PPO, 50 Hz) ─▶ SRV_Channels ─▶ joints
                         AP_InertialSensor ────▶ gravity filter + obs      ◀── joint feedback
                         microSD /APM/nnm/<robot>/robot.bin + policies/*.nnm  → 2 int8 slots in RAM
```

The observation is built in the trunk FLU frame: gyro, gravity, joint positions and velocities, previous action, commanded twist, plus a few extra channels (gesture clock, pose command). Gravity comes from a small IMU-only complementary filter at loop rate, as in training. The action is a joint offset from the standing pose, clipped, turned into PWM `1500 + q / 0.003 µs`. Flight modes map naturally: **MANUAL** = sticks are the twist, **HOLD** = twist zero, **GUIDED / AUTO / RTL / SMART_RTL** = Rover's desired speed and turn rate saturated to `NNM_VX_MAX` / `NNM_WZ_MAX`. Disarmed = servos idle.

## MicroDuck: SITL, then a real Pixhawk 6C

The first robot was [MicroDuck](https://github.com/virtualrobotix/Ardupilot-Neural-Mixer/blob/main/docs/robots/microduck.md), using Pollen's published PPO policy. The MuJoCo plant talks to SITL over the `SIM_JSON` protocol, and a scripted MAVProxy session arms, stands, walks forward, backward, sideways, turns 90°, goes to HOLD and disarms:

https://raw.githubusercontent.com/virtualrobotix/Ardupilot-Neural-Mixer/main/docs/media/demo_mavproxy_mlp.mp4

Then the same network on hardware. `Pixhawk6C-NNMixer` is a dedicated ArduRover target for the STM32H743; MuJoCo simulates only the robot body and exchanges state and `SERVO_OUTPUT_RAW` over MAVLink USB, while the board's real IMU is fed in as a disturbance:

https://raw.githubusercontent.com/virtualrobotix/Ardupilot-Neural-Mixer/main/docs/media/pixhawk6c_mlp_battery_imu.mp4

| Pixhawk 6C Mini, float32 MLP in flash | |
|---|---|
| Forward pass at 50 Hz | p50 **4.9 ms**, p99 5.0 ms |
| Autopilot CPU load | **32.8 %** (5.4 % with NNMixer disabled) |
| Flash | 1,947,204 B used, 18.9 KB free (`Pixhawk6C-NNMixerSD`, SD policies only: 811 KB free) |
| HIL battery (arm, stand, forward, lateral, turn, HOLD, disarm, parity) | 8/8 PASS in SITL |
| Firmware action vs training network | max difference 2.7e-7 |

**The lesson that cost a morning.** The first HIL run fell after one second although the observations and the network were bit-exact. The cause was ArduRover's default `INS_GYRO_FILTER` of 4 Hz, perfectly sensible for a wheeled vehicle. The gyro reached the policy tens of milliseconds late and at half amplitude. With `INS_GYRO_FILTER 0` the duck stands and walks. Any locomotion policy fed from ArduPilot's gyro will need the same rule.

## Training that deploys as is

A policy trained in a generic simulator and copied onto the autopilot sees different numbers than it was trained on. The MuJoCo environment in the repo ([`tools/robots/nnm_env.py`](https://github.com/virtualrobotix/Ardupilot-Neural-Mixer/blob/main/tools/robots/nnm_env.py)) replicates the firmware path instead: physics at the 200 Hz loop rate, gravity through the same complementary filter, the action clipped and PWM-quantized exactly like `AP_NNMixer`, the observation laid out from the robot profile, a random 0–1 tick delay on the gyro.

Why it matters: the Microban policy published by its authors, converted to int8 and run in this environment, stands with the simulator's exact gravity and **falls in under a second** with the gravity estimated by the firmware filter. It had never seen the filter. The only way to know before flying is to train and evaluate on the contract.

The second piece is **int8 from the first iteration** (quantization-aware training, [`nnm_qat.py`](https://github.com/virtualrobotix/Ardupilot-Neural-Mixer/blob/main/tools/robots/nnm_qat.py)). The firmware keeps two policies in RAM as int8 (~200 KB each). Quantizing after training introduced errors up to 0.13 rad on Microban actions. With QAT the actor's forward pass already uses the rounded per-row int8 weights, the gradient passes straight through, and the export copies the integers. `tests/test_nnm_pipeline.py` checks training network → `.nnm` file → firmware C forward give the same action.

```bash
.venv/bin/python tools/robots/fetch_upstream.py --robot rex             # original repo + MuJoCo scene
.venv/bin/python tools/robots/train_velocity.py --robot rex --name walk # int8 PPO on the deployment contract
.venv/bin/python tools/robots/train_velocity.py --robot rex --eval robots/rex/policies/walk.nnm
.venv/bin/python tools/robots/pack_robot_bin.py --robot rex             # robot.bin for the microSD
```

Full write-up: [docs/robots/training.md](https://github.com/virtualrobotix/Ardupilot-Neural-Mixer/blob/main/docs/robots/training.md) (Italian).

## Beyond walking: gestures, teleoperation, getting up

**Microban** (an 18-servo, 350 g open humanoid) became the test bench for everything a biped does besides walking. The network has no memory, so a periodic gesture needs a **clock**: two extra observation channels carry the sine and cosine of a phase, generated by the firmware from `NNM_CLOCK_HZ`. Training then imitates a reference clip (the DeepMimic recipe, as in Xue Bin Peng's [MimicKit](https://arxiv.org/abs/2510.13794)): the reward is how close the clip's joints are to the pose of *this* phase. A wave converged in 300 iterations, a full-body dance in 400.

[![Microban dance](https://raw.githubusercontent.com/virtualrobotix/Ardupilot-Neural-Mixer/main/docs/blog/img/microban_dance.jpg)](https://github.com/virtualrobotix/Ardupilot-Neural-Mixer/blob/main/docs/media/microban_dance_it0400.mp4)

**Pose teleoperation over MAVLink.** A ground station runs MediaPipe on a webcam, retargets the person's arms and knee bend to eight numbers and sends them in a `DEBUG_FLOAT_ARRAY` named `NNM_POSE`. The firmware low-passes them, applies a watchdog (`NNM_POSE_WD`) and copies them into the observation; `pose_cmd.nnm` executes the pose while keeping balance. All the intelligence stays on the GCS.

[![Microban pose teleop over MAVLink](https://raw.githubusercontent.com/virtualrobotix/Ardupilot-Neural-Mixer/main/docs/blog/img/microban_pose_teleop.jpg)](https://github.com/virtualrobotix/Ardupilot-Neural-Mixer/blob/main/docs/media/microban_pose_cmd_mavlink_wave.mp4)

**Getting up.** A biped that cannot stand up after a fall needs a human within arm's reach. `getup.nnm` is a one-shot policy: the firmware detects the fall from the trunk tilt, switches to it, runs a 14.2 s clock once (`NNM_GETUP_S`) and hands control back to the previous policy when the robot is upright. It was trained in two stages: a discovery policy finds a way to stand (simulation only, with a decaying helping force on the trunk), the best rollout is recorded, slowed down and blended into the standing pose, and a second policy imitates that clip in closed loop. Int8, from prone 100 % of the time (4–11 s), from supine 100 % (1.5–3 s), and the walking policy takes over 100 % of the time.

![Microban get-up storyboard](https://raw.githubusercontent.com/virtualrobotix/Ardupilot-Neural-Mixer/main/docs/blog/img/microban_getup_storyboard.jpg)

https://raw.githubusercontent.com/virtualrobotix/Ardupilot-Neural-Mixer/main/docs/media/microban_getup_walk_teleop.mp4

That run is one continuous 44 s sequence with the firmware state machine simulated: get up from prone, cross-fade to `walk_md.nnm`, walk, get pushed, fall, automatic get-up (side chosen from gravity), walk and turn again, stop, then `pose_cmd.nnm` waving. Details and the dead ends (joint-only imitation never stood up; a 10° upright gate taught the robot to lock its knees) are in [gesture_imitation.md](https://github.com/virtualrobotix/Ardupilot-Neural-Mixer/blob/main/docs/robots/gesture_imitation.md).

## A quadruped: Freenove Robot Dog

Same firmware, same trainer, four legs. The [Freenove Robot Dog](https://github.com/virtualrobotix/Ardupilot-Neural-Mixer/blob/main/docs/robots/freenove.md) (12 PWM servos, so it can plug straight into the autopilot outputs) went through 23 environment versions; `walk_v23.nnm` trots at about three touchdowns per leg per second with 150 ms flight phases, trunk oscillation 1.2°, 100 % survival.

| | |
|---|---|
| [![Freenove trot](https://raw.githubusercontent.com/virtualrobotix/Ardupilot-Neural-Mixer/main/docs/blog/img/freenove_trot.jpg)](https://github.com/virtualrobotix/Ardupilot-Neural-Mixer/blob/main/docs/media/freenove_walk_v23_it10000_full.mp4) | [![Freenove push recovery](https://raw.githubusercontent.com/virtualrobotix/Ardupilot-Neural-Mixer/main/docs/blog/img/freenove_push.jpg)](https://github.com/virtualrobotix/Ardupilot-Neural-Mixer/blob/main/docs/media/freenove_walk_v23_push.mp4) |
| Full sequence: 2.67 m, straight within ±2°, 180° turn in 4.6 s | Four 0.35 m/s impulses (7× the training ones): trunk back within 3° in 0.25 s |

https://raw.githubusercontent.com/virtualrobotix/Ardupilot-Neural-Mixer/main/docs/media/freenove_walk_v23_ramp7.mp4

The ramp is 7°, up and down with yaw correction. At 10° it stalls halfway: it never saw a slope in training. Honest limits are in every robot page next to the videos.

## The robot catalog

Topology is a parameter (`NNM_ROBOT`), policies are files. All robots share the MicroDuck network; only input and output sizes change. [`tools/robots/catalog.py`](https://github.com/virtualrobotix/Ardupilot-Neural-Mixer/blob/main/tools/robots/catalog.py) is the single source and generates `robot.bin`, `ppo.yaml` and the [robot pages](https://github.com/virtualrobotix/Ardupilot-Neural-Mixer/blob/main/docs/robots/README.md).

| `NNM_ROBOT` | Robot | Type | Joints | Policies today |
|---:|---|---|---:|---|
| 0 | MicroDuck (Pollen Robotics) | biped | 14 | `walk` — SITL + Pixhawk 6C HIL |
| 1 | Microban | humanoid | 18 | `walk`, `walk_md`, `wave_right`, `dance_arms`, `dance`, `pose_cmd`, `getup` |
| 10 | Freenove Robot Dog | quadruped, PWM | 12 | `walk_v7` … `walk_v23` |
| 12 | Booster T1 | humanoid | 12 | upstream policy converted to `.nnm`, verified on the contract |
| 2–9, 11, 13–17 | Zeroth-01, Bimo, Legolas, Upkie, Rex/SpotMicro, Yertle, AlbertPro, Open Duck Mini v2, Petoi Bittle, flybody, Yanshee, TienKung, ToddlerBot, Jumper | bipeds, quadrupeds, wheeled, hexapod | 6–20 | profiles and PPO configs; scenes ready for some, policies to train |

Each page links the original repository, model file, CAD and BOM, lists joints, standing pose, servo outputs and every checkpoint video against the environment version and iteration.

## Firmware in practice

Tested board: Pixhawk 6C Mini. Any H7 with a microSD and ~500 KB of free heap should work with a one-line hwdef (`include ../<Board>/hwdef.dat` + `define AP_NNMIXER_ENABLED 1`).

```
/APM/nnm/microduck/robot.bin
/APM/nnm/microduck/policies/walk.nnm
```

| Parameter | Value | Why |
|---|---|---|
| `NNM_ENABLE` | 1 | run the task |
| `NNM_ROBOT` | catalog index | topology, read at boot |
| `NNM_POLICY` | 0 | policy file index, alphabetical; changes at runtime (deferred while walking, done in HOLD) |
| `NNM_CLOCK_HZ` / `NNM_CLOCK_AUTO` | 0 | gesture or gait clock in the two channels after the twist |
| `NNM_GETUP_*` | | fall detection and automatic get-up policy |
| `SERVO1..n_FUNCTION` | 94.. | joint outputs (Scripting1..16) |
| `INS_GYRO_FILTER` | 0 | the policy needs the raw gyro |
| `SCHED_LOOP_RATE` | 200 | gravity filter at 200 Hz, as in training |

At boot the GCS shows `NNMixer: robot microduck joints=14 obs=61` and `NNMixer: loaded .../walk.nnm into slot 0`. Telemetry: `PPO_FAIL` (0 ok, 1 disarmed, 2 no joint feedback, 3 stale), `PPO_PGZ` ≈ −1 when upright, `PPO_SLOT`. A policy for another robot is rejected by id and sizes.

## What is missing, and where I would like your input

1. **Servo-bus backend.** Today joint feedback comes from the MuJoCo bridge. Real MicroDuck and Microban use Feetech / Dynamixel serial servos, so the firmware needs a bus backend that writes targets and reads positions at 50 Hz for 14–18 servos. PWM robots (Freenove, Rex, Yertle, Bittle) need a per-joint calibration (direction, center, rad/µs) in `robot.bin`. If anyone has done half-duplex Feetech/Dynamixel on ChibiOS UARTs, I would love to compare notes.
2. **More boards.** Only the 6C Mini has been exercised. Flash is tight with the float32 fallback MLP in flash; the SD-only target leaves 811 KB.
3. **Rover integration.** NNMixer currently rides on ArduRover's modes and `SRV_Channels`. Whether a legged vehicle deserves its own frame class, how GUIDED velocity should map to gait commands, and how this could ever be upstreamable are open questions I would rather discuss here than decide alone.
4. **Your robot.** Adding a robot is one entry in `catalog.py` plus a MuJoCo scene. If you have an open biped or quadruped, the pipeline will produce `robot.bin`, a training config and a doc page for it.

## Links

- Repo: https://github.com/virtualrobotix/Ardupilot-Neural-Mixer
- Firmware: https://github.com/virtualrobotix/ardupilot/tree/microduck-ppo (`libraries/AP_NNMixer/`, `hwdef/Pixhawk6C-NNMixer*/`)
- Reference document (MicroDuck integration in depth, English): [docs/reference/microduck-ppo.md](https://github.com/virtualrobotix/Ardupilot-Neural-Mixer/blob/main/docs/reference/microduck-ppo.md)
- Robot catalog: [docs/robots/README.md](https://github.com/virtualrobotix/Ardupilot-Neural-Mixer/blob/main/docs/robots/README.md)
- All videos: [docs/media/](https://github.com/virtualrobotix/Ardupilot-Neural-Mixer/tree/main/docs/media)

Quickstart in SITL, no hardware needed:

```bash
git clone --recurse-submodules https://github.com/virtualrobotix/Ardupilot-Neural-Mixer.git ~/nnmixer
cd ~/nnmixer && uv venv --python 3.12 && uv pip install -r requirements.txt
cd ardupilot && ../.venv/bin/python ./waf configure --board sitl && ../.venv/bin/python ./waf rover && cd ..
.venv/bin/python scripts/demo_mavproxy.py
```

Comments, criticism and pull requests welcome.

Roberto Navoni — DelphyAI LAB
