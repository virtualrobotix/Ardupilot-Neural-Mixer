# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""Single source of truth for the robot catalog.

tools/robots/build_catalog.py writes, for every robot here:
  robots/<id>/robot/profile.json   topology + upstream + simulation block
  robots/<id>/robot/ppo.yaml       PPO architecture and ArduPilot-compatible env
  docs/robots/<id>.md              robot page
  docs/robots/README.md            configuration table

Upstream facts (paths, joints, servos, rates, licenses) were read from the
upstream repositories in September 2026; the "source" notes say where.
"""

from __future__ import annotations

import math

DEG = math.pi / 180.0

# Hidden sizes shared by every robot: the MicroDuck network already validated on a Pixhawk 6C.
MLP_HIDDEN = [512, 256, 128]

ROBOTS: dict[str, dict] = {
    "microduck": {
        "index": 0,
        "class": "biped",
        "display_name": "MicroDuck",
        "maker": "Pollen Robotics / Hugging Face",
        "status": "policy",
        "joint_names": [
            "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
            "neck_pitch", "head_pitch", "head_yaw", "head_roll",
            "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
        ],
        "q0": [0.0, -0.0873, -0.4579, -0.0049, 0.4530, 0.3491, 0.3491, 0.0, 0.0,
               0.0, 0.0873, 0.4579, 0.0049, -0.4530],
        "extra_cmd_dim": 10,           # head pose 4 + body pose 6, zero on the autopilot
        "rate_hz": 50,
        "command_ranges": {"vx": [-0.4, 0.4], "vy": [-0.3, 0.3], "wz": [-1.0, 1.0]},
        "upstream": {
            "repo": "https://github.com/pollen-robotics/microduck_rl",
            "license": "vedi upstream",
            "sim_model": "src/mjlab_microduck/robot/microduck/scene.xml",
            "cad": "https://huggingface.co/spaces/pollen-robotics/microduck-simulator",
            "bom": "https://huggingface.co/spaces/pollen-robotics/microduck-simulator",
            "training": "mjlab 1.3.0 + rsl_rl PPO (MuJoCo Warp)",
            "published_policy": "addestrata nel laboratorio NOESIS EXPERIMENT; ONNX in policies/ di questo repo",
        },
        "sim": {"actuator": "bam_xl330", "trunk_body": "trunk_base", "freejoint": "trunk_base_freejoint",
                "gyro_sensor": "imu_ang_vel", "accel_sensor": "imu_accel", "home_z": 0.125},
        "servos": "14× Dynamixel XL330 (bus)",
        "link": "bus",
        "policies": {"walk.nnm": "MLP 61-512-256-128-14 (2048 env x 2000 iterazioni), int8 per riga "
                                 "ricavato dall'ONNX float32 validato. Nell'ambiente a contratto: in piedi 10 s, "
                                 "avanti 10 s (~0,10 m/s a comando 0,3), rotazione 10 s, nessuna caduta."},
        "notes": [
            "Integrazione di riferimento: SITL, batteria HIL 8/8 e HIL hardware su Pixhawk 6C Mini con la build float32.",
            "Il firmware comanda 14 funzioni servo (Scripting1..14). Il robot reale richiede un backend bus "
            "Dynamixel nel firmware, non ancora scritto; SITL e HIL usano il trasporto SIM_JSON / MAVLink.",
        ],
    },
    "microban": {
        "index": 1,
        "class": "biped",
        "display_name": "Microban",
        "maker": "Rhoban",
        "status": "policy-upstream",
        "joint_names": [
            "right_shoulder_pitch", "right_shoulder_roll", "right_elbow",
            "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle_pitch",
            "right_ankle_roll",
            "left_shoulder_pitch", "left_shoulder_roll", "left_elbow",
            "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle_pitch",
            "left_ankle_roll",
        ],
        "q0": [0.0, -10 * DEG, -20 * DEG, 0.0, -5 * DEG, -10 * DEG, 0.0, 0.0, 5 * DEG,
               0.0, 10 * DEG, -20 * DEG, 0.0, 5 * DEG, -10 * DEG, 0.0, 0.0, -5 * DEG],
        # gesture clock: sin and cos of a phase the firmware advances at NNM_CLOCK_HZ (0 = channels at
        # zero, the walk). A timed gesture (the wave) is a motion-imitation task: the policy tracks the
        # reference pose of the current phase, so it must see the phase.
        # channels: sin, cos of the gesture clock, then 8 pose-command offsets from q0
        # (right/left shoulder pitch, roll, elbow; knee_bend; lateral sway). Zero = stand still.
        "extra_cmd_dim": 10,
        "extra_cmd_desc": "orologio sin, cos 2 (`NNM_CLOCK_HZ`) + posa comandata 8 "
                          "(braccia 6, piegamento, ondeggiamento; via MAVLink `NNM_POSE`)",
        "rate_hz": 50,
        "command_ranges": {"vx": [-0.3, 0.3], "vy": [-0.2, 0.2], "wz": [-1.0, 1.0]},
        "upstream": {
            "repo": "https://github.com/Rhoban/mjlab_microban",
            "license": "Apache-2.0 (software); repo hardware GPL-3.0 / CC BY-NC-SA 4.0",
            "sim_model": "src/mjlab_microban/robot/microban/scene.xml",
            "hardware_repo": "https://github.com/Rhoban/microban",
            "cad": "https://github.com/Rhoban/microban/tree/main/cad (stl/, step/) e Onshape "
                   "https://cad.onshape.com/documents/d424992a192a8ce34ffce163",
            "bom": "https://github.com/Rhoban/microban/tree/main/docs (bom.md, printing.md, assembly.md)",
            "training": "mjlab 1.3.0 + rsl_rl PPO, modello attuatore BAM XL330 (kp_fw 125)",
            "published_policy": "https://github.com/Rhoban/microban/blob/main/src/agents/walk.onnx",
            "policy_url": "https://raw.githubusercontent.com/Rhoban/microban/main/src/agents/walk.onnx",
            "policy_name": "walk",
        },
        # the MJCF imu site is rotated w.r.t. the trunk; the env adds an IMU aligned with the trunk,
        # which is what the autopilot reports once AHRS_ORIENTATION matches its mounting
        "sim": {"actuator": "bam_xl330", "bam_kp_fw": 125.0, "trunk_body": "trunk",
                "freejoint": "trunk_freejoint", "home_z": 0.168,
                "feet": [{"site": "left_foot", "body": "foot_2"}, {"site": "right_foot", "body": "foot"}]},
        # gait terms as in the MicroDuck mjlab task; upright/pose lowered after a run that learned to stand still
        # alive keeps the per-step reward positive from iteration 0: without it a random policy learns to
        # end episodes early by falling (action_rate outweighs the other terms)
        # MicroDuck mjlab velocity task (params/env.yaml of W&B run mjlab_microduck/vfaof1ds), same terms,
        # shapes and weights; swing target raised from 0.02 m to 0.03 m for the taller robot, arm posture
        # std added (MicroDuck has no arms)
        "reward": {
            "track_lin_vel": 2.0, "tracking_sigma": 0.1,            # std 0.316
            "track_ang_vel": 2.0, "tracking_sigma_ang": 0.5,        # std 0.707
            "upright": 2.0, "upright_std": 0.2236,
            "pose": 1.0, "walking_threshold": 0.01,
            "pose_std_standing": {".*hip_yaw.*": 0.1, ".*hip_roll.*": 0.05, ".*hip_pitch.*": 0.15,
                                  ".*knee.*": 0.15, ".*ankle.*": 0.1, ".*shoulder.*": 0.2, ".*elbow.*": 0.2},
            "pose_std_walking": {".*hip_yaw.*": 0.3, ".*hip_roll.*": 0.05, ".*hip_pitch.*": 0.4,
                                 ".*knee.*": 0.4, ".*ankle.*": 0.25, ".*shoulder.*": 0.5, ".*elbow.*": 0.5},
            "action_rate": -0.1, "alive": 0.0,
            "body_ang_vel": -0.05, "angular_momentum": -0.02, "dof_pos_limits": -1.0, "self_collisions": -1.0,
            "air_time": 3.0, "air_time_mode": "in_range", "air_time_min_s": 0.125, "air_time_max_s": 0.3,
            "foot_clearance": -2.0, "foot_swing_height": -0.25, "swing_height_m": 0.03, "foot_slip": -0.1,
            # added after the aligned run stood still with feet on the ground at iteration 300 (MicroDuck
            # was already stepping there): reward single stance while moving
            "single_stance": 1.0,
            "foot_lift": 1.0,       # lifting the swing foot during a step, up to swing_height_m
            # iteration 400 stepped with one foot only: reward left/right alternation per touchdown
            "foot_alternation": 0.3,
            # iteration 1000 alternated but the right foot only shuffled (1 cm, 0.13 s vs 5 cm, 0.29 s):
            # a step needs >= 0.1 s airborne and half the swing height, and both feet must match
            "alternation_min_air_s": 0.1, "alternation_min_height_frac": 0.5,
            "foot_symmetry": -0.1,

        },
        "env": {
            "fall_tilt_deg": 70.0, "resample_s": [3.0, 8.0], "p_zero_command": 0.02,
            "push": {"interval_s": [3.0, 6.0], "vel_xy": 0.15},
            "curriculum": {
                "action_rate_stages": [[0, -0.1], [12000, -0.2], [18000, -0.4], [24000, -0.6],
                                       [30000, -0.8], [36000, -1.0]],
                "standing_stages": [[0, 0.02], [12000, 0.05], [18000, 0.1], [24000, 0.15],
                                    [36000, 0.2], [48000, 0.25]],
            },
        },
        "init_noise_std": 1.0,
        "servos": "19× Dynamixel XL330-M288-T (bus); la testa non è comandata dalla policy",
        "link": "bus",
        "policies": {"walk.nnm": "walk.onnx pubblicato da Rhoban (MLP 63-512-256-128-18, stesso contratto "
                                 "NNMixer) convertito in int8 per riga. Nell'ambiente a contratto: in piedi 10 s, "
                                 "avanti cade a 6,8 s, rotazione cade a 1 s. Con la gravità esatta del "
                                 "simulatore regge 10 s in avanti: la policy è stata addestrata senza il filtro "
                                 "IMU dell'autopilota. Da rifinire con --init-onnx prima dell'uso.",
                     "walk_md.nnm": "addestrata da zero sul contratto ArduPilot in int8 (QAT): reward e curriculum "
                                    "del task MicroDuck più premi sul passo (appoggio singolo, piede sollevato, "
                                    "alternanza, simmetria), 512 env × 3000 iterazioni, checkpoint 2700 (punteggio "
                                    "0,64). Sopravvivenza 100% in tutte le modalità; avanti/indietro ~0,16-0,19 m/s "
                                    "a comando 0,3; rotazione 0,68-0,87 rad/s a comando 0,8; laterale 0,04 m/s a "
                                    "comando 0,2; passo alternato e simmetrico (3-4 cm, 0,27-0,29 s per piede). "
                                    "W&B mjlab_microban/h5e7djou. Allargata a 65 ingressi con due colonne a zero "
                                    "(uscita identica) quando Microban ha ricevuto i canali dell'orologio.",
                     "wave_right.nnm": "saluto con la destra per imitazione di una clip (vedi gesture_imitation.md): "
                                       "mano alta, la spalla porta la mano in fuori e la riporta, un'onda ogni 2 s "
                                       "(`NNM_CLOCK_HZ 0.5`). Rifinita da walk_md, checkpoint 300: errore medio sui "
                                       "tre giunti 0,01 rad, in piedi senza spostarsi. W&B mjlab_microban/fqp2z48h.",
                     "dance_arms.nnm": "balletto delle sole braccia: le due braccia pompano in alternanza, ciclo di "
                                       "1,6 s (`NNM_CLOCK_HZ 0.625`). Da wave_right, checkpoint 200: errore "
                                       "0,01-0,05 rad su sei giunti. W&B mjlab_microban/pa1otmey.",
                     "dance.nnm": "balletto a corpo intero, 16 giunti su 18: braccia di dance_arms, molleggio sulle "
                                  "ginocchia due volte per ciclo (anca e caviglia coordinate, piede piatto) e "
                                  "ondeggiamento laterale del bacino di 2,4 cm (`NNM_CLOCK_HZ 0.625`). Da dance_arms, "
                                  "checkpoint 400: errore 0,00-0,05 rad su tutti i giunti della coreografia, nessuna "
                                  "caduta, resta sul posto. W&B mjlab_microban/o57v0gih.",
                     "pose_cmd.nnm": "teleoperazione: esegue una posa comandata in tempo reale via MAVLink "
                                     "(`DEBUG_FLOAT_ARRAY` nome `NNM_POSE`, 8 canali dopo l'orologio: 6 offset delle "
                                     "braccia, piegamento delle ginocchia 0-0,55 rad, ondeggiamento ±0,15 rad) tenendo "
                                     "l'equilibrio (vedi gesture_imitation.md). Addestrata da dance it0400 su pose "
                                     "casuali, comando zero e stream delle clip, con ritardo 0-3 tick e passa-basso "
                                     "0,15 s come il firmware (`NNM_POSE_WD 500`, `NNM_POSE_TAU 0.15`, "
                                     "`NNM_CLOCK_HZ 0`). Checkpoint 1800 di 2000 (800 + 1200 dopo la correzione di "
                                     "`base_height`): errore medio sui 16 giunti su pose tenute 0,03 rad, saluto in "
                                     "streaming a 40 Hz 0,04 rad sulle braccia, nessuna caduta. Il selettore "
                                     "stand/walk del trainer non misura la posa: il file è scelto sul tracking."},
        # Each clip is the int8 policy on the ArduPilot contract. "env" is the reward/config
        # version of that run; one training iteration is 5 PPO epochs.
        "videos": [{
            "env": "pose_cmd (MAVLink)",
            "iteration": "1800",
            "epochs": "9000",
            "latest": True,
            "mp4": "microban_pose_cmd_mavlink_wave.mp4",
            "caption": "Teleoperazione: `tools/mocap/mocap_gcs.py --source clip` manda il saluto come stream "
                       "`NNM_POSE` a 40 Hz via UDP; `play_policy.py --pose-mavlink` lo riceve e la policy "
                       "`pose_cmd.nnm` lo esegue in tempo reale (errore braccia 0,04 rad). In verde la posa comandata.",
        }, {
            "env": "pose_cmd (generalizzazione)",
            "iteration": "1800",
            "epochs": "9000",
            "mp4": "microban_pose_cmd_generalization.mp4",
            "caption": "Movimenti mai visti in training, da landmark MediaPipe sintetici attraverso il retarget "
                       "reale: jumping jack (err 0,07 rad), pugni alternati a 1 Hz (0,15: ritardo del passa-basso "
                       "0,15 s), squat con braccia avanti (0,05), inclinazioni laterali (0,06), combo (0,06). "
                       "22 s senza cadere.",
        }, {
            "env": "pose_cmd",
            "iteration": "1800",
            "epochs": "9000",
            "mp4": "microban_pose_cmd_it1800.mp4",
            "caption": "Pose comandate da script (`pose_cmd.nnm`): braccia alzate, piegamento 0,45 rad, "
                       "ondeggiamento ±0,12, posa completa. Errore medio 0,06 rad lungo la sequenza, transizioni "
                       "incluse; 0,03 rad a regime.",
        }, {
            "env": "pose_cmd",
            "iteration": "400",
            "epochs": "2000",
            "mp4": "microban_pose_cmd_it0400.mp4",
            "caption": "Stesso run alla 400 (prima della correzione di `base_height`): braccia seguite, ginocchia "
                       "ancora piegate a comando zero, errore 0,13 rad.",
        }, {
            "env": "pose_cmd",
            "iteration": "100",
            "epochs": "500",
            "mp4": "microban_pose_cmd_demo.mp4",
            "caption": "Stessa sequenza alla 100: le braccia seguono grosso modo, le ginocchia restano piegate "
                       "(prior del balletto), errore 0,27 rad.",
        }, {
            "env": "dance",
            "iteration": "400",
            "epochs": "2000",
            "mp4": "microban_dance_it0400.mp4",
            "caption": "Balletto a corpo intero per imitazione di clip (`dance.nnm`). In verde il corpo di "
                       "riferimento della clip in quell'istante. Braccia, molleggio e ondeggiamento seguono la "
                       "clip; 8 s sul posto senza cadere.",
        }, {
            "env": "dance",
            "iteration": "100",
            "epochs": "500",
            "mp4": "microban_dance_it0100.mp4",
            "caption": "Stesso run alla 100: le braccia seguono, le gambe fanno metà del molleggio e le "
                       "caviglie restano ferme. Il tronco non oscilla ancora.",
        }, {
            "env": "dance_arms",
            "iteration": "200",
            "epochs": "1000",
            "mp4": "microban_dance_arms_it0200.mp4",
            "caption": "Balletto delle sole braccia (`dance_arms.nnm`), partito dal saluto: la clip è seguita "
                       "entro 0,05 rad già alla 200.",
        }, {
            "env": "wave_right_clip",
            "iteration": "300",
            "epochs": "1500",
            "mp4": "microban_wave_right_clip_it0300.mp4",
            "caption": "Saluto per imitazione di clip (`wave_right.nnm`): mano alta, arco della spalla ogni "
                       "2 s, quattro onde in 8 s, errore 0,01 rad.",
        }, {
            "env": "wave_right_clip",
            "iteration": "100",
            "epochs": "500",
            "mp4": "microban_wave_right_clip_it0100.mp4",
            "caption": "Stesso run alla 100: braccio alzato, l'arco non è ancora seguito.",
        }, {
            "env": "wave_right_v5 (senza orologio)",
            "iteration": "100",
            "epochs": "500",
            "mp4": "microban_wave_right_v5_it0100.mp4",
            "caption": "Ultimo tentativo con reward scritto a mano (fascia + velocità + arco): un solo arco, "
                       "poi fermo. La rete senza fase non trova l'oscillazione.",
        }, {
            "env": "wave_right_v4 (senza orologio)",
            "iteration": "100",
            "epochs": "500",
            "mp4": "microban_wave_right_v4_it0100.mp4",
            "caption": "Reward su velocità del gomito: 38 inversioni in 8 s, uno scuotimento, non un saluto.",
        }, {
            "env": "wave_right (senza orologio)",
            "iteration": "400",
            "epochs": "2000",
            "mp4": "microban_wave_right_it0400.mp4",
            "caption": "Primo tentativo, bersaglio alternato per il gomito: sta in piedi e non muove il "
                       "braccio. Il premio del gesto era già a zero a braccio abbassato.",
        }, {
            "env": "walk_md",
            "iteration": "3000",
            "epochs": "15000",
            "title": "Risultato dopo 3000 iterazioni (`walk_md.nnm`)",
            "gif": "microban_walk_md.gif",
            "mp4": "microban_walk_md_it3000.mp4",
            "caption": "Ambiente allineato a MicroDuck più premi sul passo. Avanti 5 s a 0,3 m/s, "
                       "destra 3 s, 180° a sinistra, avanti 5 s: nessuna caduta in 16,7 s, 2,4 m percorsi.",
        }, {
            "env": "walk_md",
            "iteration": "0–3000",
            "epochs": "0–15000",
            "mp4": "microban_md_evolution.mp4",
            "caption": "Evoluzione dello stesso ambiente: stessi comandi sui checkpoint successivi.",
        }, {
            "env": "walk_gait_v1",
            "iteration": "900",
            "epochs": "4500",
            "mp4": "microban_gait_best_it900.mp4",
            "caption": "Miglior checkpoint del run sul passo. Sta in piedi e ruota sul posto; "
                       "l'avanzamento resta sotto 0,1 m.",
        }, {
            "env": "walk_gait_v1",
            "iteration": "0–1400",
            "epochs": "0–7000",
            "mp4": "microban_evolution.mp4",
            "caption": "14 clip, avanti 4 s poi rotazione. Entro 100 iterazioni non cade; "
                       "dalla 500 ruota; in avanti si sposta di pochi centimetri.",
        }, {
            "env": "walk_ap",
            "iteration": "200",
            "epochs": "1000",
            "mp4": "microban_sequence_it200.mp4",
            "caption": "Rifinitura della policy upstream. Resta in piedi; la velocità comandata "
                       "non è ancora seguita.",
        }],
        "notes": [
            "L'osservazione dell'ONNX è gyro, gravità proiettata, q−q0, q̇, azione precedente, twist: il formato "
            "NNMixer con 18 giunti, quindi la policy pubblicata si converte senza riaddestrarla.",
            "18 giunti superano le 16 funzioni servo Scripting consecutive: il firmware carica la topologia "
            "(`NNM_MAX_JOINTS` 20) e gira in SITL, ma sul robot servono le uscite del backend bus Dynamixel. "
            "Simulazione e training funzionano già.",
            "L'osservazione è 73: dopo il twist ci sono seno e coseno dell'orologio dei gesti "
            "(`NNM_CLOCK_HZ`, 0 per la camminata) e otto canali di posa comandata via MAVLink (`NNM_POSE`; "
            "a zero = riposo, quindi walk e clip non ne risentono). Saluto, balletto e teleoperazione sono "
            "selezionabili con `NNM_POLICY`: vedi [gesture_imitation.md](gesture_imitation.md).",
        ],
    },
    "zeroth": {
        "index": 2,
        "class": "biped",
        "display_name": "Zeroth-01",
        "maker": "Zeroth Robotics / K-Scale Labs",
        "status": "needs-model",
        "joint_names": [
            "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee_pitch", "right_ankle_pitch",
            "right_ankle_roll",
            "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee_pitch", "left_ankle_pitch",
            "left_ankle_roll",
            "left_shoulder_pitch", "left_shoulder_roll", "left_elbow_roll", "left_gripper_roll",
            "right_shoulder_pitch", "right_shoulder_roll", "right_elbow_roll", "right_gripper_roll",
        ],
        "q0": [0.0, -0.1, -0.4, -0.8, -0.4, -0.1,
               0.0, 0.1, -0.4, -0.8, -0.4, 0.1,
               0.0, 0.2, -0.2, 0.0,
               0.0, -0.2, 0.2, 0.0],
        "extra_cmd_dim": 0,
        "rate_hz": 50,
        "command_ranges": {"vx": [-0.3, 0.3], "vy": [-0.2, 0.2], "wz": [-1.0, 1.0]},
        "upstream": {
            "repo": "https://github.com/zeroth-robotics/zeroth-bot",
            "license": "MIT",
            "sim_model_note": "nessun MJCF nel repo: il task ksim lo scarica con "
                              "ksim.get_mujoco_model_path('zbot', name='robot') (pip install ksim); "
                              "salvarlo come robots/zeroth/robot/scene.xml",
            "cad": "Onshape https://cad.onshape.com/documents/cacc96f8a7850b951e7aa69a",
            "bom": "https://docs.kscale.dev/robots/zeroth-01/bom/",
            "training": "ksim (MuJoCo / JAX), policy ricorrente esportata in .kinfer",
            "published_policy": "release V0.2.1 ppo_standing.pt / ppo_walking.pt (pipeline più vecchia, "
                                "osservazione diversa); non convertibile",
        },
        "sim": {"actuator": "position", "home_z": 0.40},
        "servos": "servo Feetech STS3250 su bus seriale (la documentazione ne indica 16; il task ksim comanda 20 giunti)",
        "link": "bus",
        "policies": {},
        "notes": [
            "q0 è JOINT_BIASES di ksim-gym-zbot/train.py.",
            "L'osservazione ksim (quaternione, yaw assoluto, altezza della base) non è il contratto NNMixer: "
            "la policy va addestrata con tools/robots/train_velocity.py o mjlab.",
            "20 giunti superano le 16 funzioni servo Scripting: sull'autopilota serve il backend bus.",
        ],
    },
    "bimo": {
        "index": 3,
        "class": "biped",
        "display_name": "Bimo",
        "maker": "Mekion",
        "status": "needs-model",
        "joint_names": ["RHip", "LHip", "RShoulder", "LShoulder", "RKnee", "LKnee", "RAnkle", "LAnkle"],
        "q0": [-30 * DEG, -30 * DEG, 0.0, 0.0, 60 * DEG, 60 * DEG, 30 * DEG, 30 * DEG],
        "extra_cmd_dim": 0,
        "rate_hz": 25,
        "command_ranges": {"vx": [-0.2, 0.2], "vy": [0.0, 0.0], "wz": [-0.8, 0.8]},
        "upstream": {
            "repo": "https://github.com/mekion/the-bimo-project",
            "license": "Apache-2.0",
            "sim_model_note": "solo USD Isaac Lab (IsaacLab/bimo/assets/Bimo.usd); convertire USD -> MJCF "
                              "e salvarlo come robots/bimo/robot/scene.xml",
            "cad": "non ancora pubblicato (upstream: coming soon)",
            "bom": "non ancora pubblicata; per il setup MCU/README.md e BimoAPI/README.md",
            "training": "Isaac Lab + rsl_rl PPO, distillata in uno studente [64, 32] su RP2040",
            "published_policy": "nessuna",
        },
        "sim": {"actuator": "position", "home_z": 0.38},
        "servos": "8× Feetech STS3215 12 V (bus)",
        "link": "bus",
        "policies": {},
        "notes": [
            "L'upstream gira a 20 Hz, che non divide i 50 Hz del task AP_NNMixer: si addestra a 25 Hz.",
            "Le azioni upstream sono incrementi del comando giunto; il contratto NNMixer usa offset da q0, "
            "quindi la policy va riaddestrata.",
        ],
    },
    "legolas": {
        "index": 4,
        "class": "biped",
        "display_name": "Legolas",
        "maker": "daviddoo02",
        "status": "needs-model",
        "joint_names": ["R_Hip_1", "R_Hip_2", "R_Thigh", "R_Foreleg", "R_Servo",
                        "L_Hip_1", "L_Hip_2", "L_Thigh", "L_Foreleg", "L_Servo"],
        "q0": [0.0] * 10,
        "extra_cmd_dim": 0,
        "rate_hz": 50,
        "command_ranges": {"vx": [-0.3, 0.3], "vy": [-0.1, 0.1], "wz": [-0.8, 0.8]},
        "upstream": {
            "repo": "https://github.com/daviddoo02/Legolas-an-open-source-biped",
            "license": "MIT",
            "sim_model": "Mujoco xml/Working Mk 5 - Old model - Demo only/CMU Mk 5.xml",
            "sim_model_note": "MJCF dimostrativo: giunto libero commentato, gravità 0, ctrlrange dei motori ±1e-5, "
                              "gambe a catena chiusa (4 vincoli connect); va sistemato prima del training e "
                              "salvato come robots/legolas/robot/scene.xml",
            "cad": "https://github.com/daviddoo02/Legolas-an-open-source-biped/tree/main/CAD "
                   "(SolidWorks V1-V3, STL in CAD/Legolas/V2/STLs)",
            "bom": "lista materiali nel README upstream; guida di montaggio non ancora pubblicata",
            "training": "nessuno upstream (controllore IK del passo su ROS)",
            "published_policy": "nessuna",
        },
        "sim": {"actuator": "position", "home_z": 0.6},
        "servos": "10 servo hobby PWM (8× 40 kg, 2× 80 kg), upstream tramite PCA9685",
        "link": "pwm",
        "policies": {},
        "notes": [
            "q0 non è pubblicata: va definita la posa di stand sul MJCF sistemato e scritta in catalog.py.",
            "I servo PWM si collegano direttamente alle uscite dell'autopilota (10 ≤ 16 funzioni servo); manca "
            "ancora in robot.bin una calibrazione per giunto (verso, centro, rad/µs).",
        ],
    },
    "upkie": {
        "index": 5,
        "class": "biped",
        "display_name": "Upkie (wheeled biped)",
        "maker": "upkie / Stéphane Caron",
        "status": "needs-firmware",
        "joint_names": ["left_hip", "left_knee", "left_wheel", "right_hip", "right_knee", "right_wheel"],
        "q0": [0.0] * 6,
        "extra_cmd_dim": 0,
        "rate_hz": 50,
        "command_ranges": {"vx": [-0.8, 0.8], "vy": [0.0, 0.0], "wz": [-1.5, 1.5]},
        "upstream": {
            "repo": "https://github.com/upkie/upkie",
            "license": "Apache-2.0",
            "model_repo": "https://github.com/MarcDcls/mjlab_upkie",
            "sim_model": "src/mjlab_upkie/robot/upkie/scene.xml",
            "cad": "https://github.com/upkie/upkie_parts (FreeCAD, STL, 3MF)",
            "bom": "https://upkie.github.io/upkie/build-your-own.html",
            "training": "mjlab 1.3.0 + rsl_rl (MarcDcls/mjlab_upkie)",
            "published_policy": "MarcDcls/mjlab_upkie logs/rsl_rl/upkie_velocity/bests/default.onnx "
                                "(osservazione con quaternione del tronco, ruote in velocità); non convertibile",
        },
        "sim": {"actuator": "position", "home_z": 0.343},
        "servos": "4× mjbots qdd100 (anche, ginocchia), 2× moteus + mj5208 (ruote), CAN-FD",
        "link": "can",
        "policies": {},
        "notes": [
            "Le ruote ricevono comandi di velocità. AP_NNMixer tratta ogni azione come offset di posizione, "
            "quindi Upkie richiede nel firmware un tipo di azione per giunto.",
        ],
    },
    "rex": {
        "index": 6,
        "class": "quadruped",
        "display_name": "Rex / SpotMicro",
        "maker": "nicrusso7 (SpotMicroAI community design)",
        "status": "needs-training",
        "joint_names": [
            "motor_front_left_shoulder", "motor_front_left_leg", "foot_motor_front_left",
            "motor_front_right_shoulder", "motor_front_right_leg", "foot_motor_front_right",
            "motor_rear_left_shoulder", "motor_rear_left_leg", "foot_motor_rear_left",
            "motor_rear_right_shoulder", "motor_rear_right_leg", "foot_motor_rear_right",
        ],
        "q0": [0.0, -0.88643435, 1.30197369] * 4,
        "extra_cmd_dim": 0,
        "rate_hz": 50,
        "command_ranges": {"vx": [-0.3, 0.3], "vy": [-0.2, 0.2], "wz": [-1.0, 1.0]},
        "upstream": {
            "repo": "https://github.com/nicrusso7/rex-gym",
            "branch": "master",
            "license": "Apache-2.0",
            "urdf": "rex_gym/util/pybullet_data/assets/urdf/rex.urdf",
            "cad": "https://www.thingiverse.com/thing:3445283 e https://github.com/FlorianWilk/SpotMicroAI",
            "bom": "https://github.com/nicrusso7/rexctl/wiki/Mark-I",
            "training": "PyBullet + PPO TensorFlow 1 (ibrido open-loop + residuo)",
            "published_policy": "checkpoint TensorFlow rex_gym/policies/*/model.ckpt-* (osservazione di 4 "
                                "valori, azioni residue); non convertibili",
        },
        "sim": {"actuator": "position", "trunk_body": "base_link", "freejoint": "root",
                "gyro_sensor": "nnm_gyro", "accel_sensor": "nnm_accel", "home_z": 0.22,
                "position_kp": 8.0, "position_kv": 0.3, "force_limit": 1.1},
        "servos": "12× MG996R PWM",
        "link": "pwm",
        "policies": {},
        "notes": [
            "Solo URDF: fetch_upstream.py genera robots/rex/robot/scene.xml (giunto libero, pavimento, IMU, "
            "attuatori di posizione). Guadagni e limiti di coppia degli attuatori sono stime da verificare.",
            "I servo PWM si collegano direttamente alle uscite dell'autopilota (12 ≤ 16); manca ancora in "
            "robot.bin la calibrazione per giunto.",
        ],
    },
    "yertle": {
        "index": 7,
        "class": "quadruped",
        "display_name": "Yertle",
        "maker": "Jerome Graves",
        "status": "needs-training",
        "joint_names": [
            "lf_shoulder", "lf_thigh", "lf_shin",
            "rf_shoulder", "rf_thigh", "rf_shin",
            "lb_shoulder", "lb_thigh", "lb_shin",
            "rb_shoulder", "rb_thigh", "rb_shin",
        ],
        "q0": [0.0, -0.6, 0.9] * 4,
        "extra_cmd_dim": 0,
        "rate_hz": 50,
        "command_ranges": {"vx": [-0.4, 0.4], "vy": [-0.2, 0.2], "wz": [-1.0, 1.0]},
        "upstream": {
            "repo": "https://github.com/Jerome-Graves/yertle",
            "license": "MIT",
            "urdf": "simulation/yertle.urdf",
            "cad": "https://github.com/Jerome-Graves/yertle/tree/main/design (CAD/Yertle_Single_v2.step, STL/)",
            "bom": "https://github.com/Jerome-Graves/yertle/blob/main/design/README.md",
            "training": "SB3 PPO su PyBullet; Isaac Lab + rsl_rl",
            "published_policy": "nessuna",
        },
        "sim": {"actuator": "position", "trunk_body": "base_link", "freejoint": "root",
                "gyro_sensor": "nnm_gyro", "accel_sensor": "nnm_accel", "home_z": 0.25,
                "position_kp": 15.0, "position_kv": 0.5, "force_limit": 3.4},
        "servos": "12× SPT5435LV-180W 35 kg PWM, upstream tramite PCA9685 su ESP32",
        "link": "pwm",
        "policies": {},
        "notes": [
            "L'osservazione upstream (48) è il formato NNMixer più la velocità lineare della base, che "
            "l'autopilota non misura; il contratto NNMixer la toglie (45).",
            "I servo PWM si collegano direttamente alle uscite dell'autopilota (12 ≤ 16); manca ancora in "
            "robot.bin la calibrazione per giunto.",
        ],
    },
}

ROBOTS["albert"] = {
    "index": 8,
    "class": "quadruped",
    "display_name": "AlbertPro",
    "maker": "thinking0things",
    "status": "needs-training-mjcf",
    "joint_names": ["FL_hip", "FL_knee", "FR_hip", "FR_knee", "RL_hip", "RL_knee", "RR_hip", "RR_knee"],
    "q0": [0.90, -1.40] * 4,
    "extra_cmd_dim": 0,
    "rate_hz": 50,
    "command_ranges": {"vx": [-0.25, 0.25], "vy": [0.0, 0.0], "wz": [-0.8, 0.8]},
    "upstream": {
        "repo": "https://github.com/thinking0things/AlbertPro",
        "license": "MIT",
        "sim_model": "RL/dog.xml",
        "cad": "https://github.com/thinking0things/AlbertPro/tree/main/hardware "
               "(mesh STL per la simulazione in RL/meshes)",
        "bom": "https://github.com/thinking0things/AlbertPro#robot (corpo 14 × 11 × 2 cm, 8 servo, "
               "PCA9685, ESP32)",
        "training": "PPO + GAE in MuJoCo (notebook in RL/), 100 Hz, azioni ΔΔθ su un buffer di delta",
        "published_policy": "RL/models/ policy.h (MLP 24-64-8 ReLU/tanh, osservazione senza IMU, azioni "
                            "ΔΔθ); non convertibile",
    },
    "sim": {"actuator": "position", "trunk_body": "trunk", "freejoint": "floating_base",
            "gyro_sensor": "imu_gyro", "accel_sensor": "imu_acc", "home_z": 0.10},
    "servos": "8 servo PWM tramite PCA9685 (I²C) su ESP32",
    "link": "pwm",
    "policies": {},
    "notes": [
        "La scena upstream è già MuJoCo nativa (attuatori di posizione kp 60, IMU sul tronco): "
        "fetch_upstream.py la usa così com'è.",
        "L'upstream gira a 100 Hz; il task AP_NNMixer arriva a 50 Hz, quindi si addestra a 50 Hz.",
        "La policy upstream non vede l'IMU e comanda accelerazioni dei giunti (ΔΔθ): per l'autopilota "
        "va riaddestrata sul contratto NNMixer (offset da q0).",
        "I servo PWM si collegano direttamente alle uscite dell'autopilota (8 ≤ 16); manca ancora in "
        "robot.bin la calibrazione per giunto.",
    ],
}

ROBOTS["openduck"] = {
    "index": 9,
    "class": "biped",
    "display_name": "Open Duck Mini v2",
    "maker": "Antoine Pirrone (apirrone) e comunità, supporto Hugging Face / Pollen Robotics",
    "status": "needs-training-mjcf",
    "joint_names": [
        "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
        "neck_pitch", "head_pitch", "head_yaw", "head_roll",
        "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
    ],
    "q0": [0.002, 0.053, -0.63, 1.368, -0.784, 0.0, 0.0, 0.0, 0.0,
           -0.003, -0.065, 0.635, 1.379, -0.796],
    "extra_cmd_dim": 0,
    "rate_hz": 50,
    "command_ranges": {"vx": [-0.15, 0.15], "vy": [-0.2, 0.2], "wz": [-1.0, 1.0]},
    "upstream": {
        "repo": "https://github.com/apirrone/Open_Duck_Mini",
        "branch": "v2",
        "license": "Apache-2.0 (Open_Duck_Mini); Open_Duck_Playground senza licenza dichiarata",
        "model_repo": "https://github.com/apirrone/Open_Duck_Playground",
        "sim_model": "playground/open_duck_mini_v2/xmls/scene_flat_terrain.xml",
        "cad": "Onshape https://cad.onshape.com/documents/64074dfcfa379b37d8a47762 e "
               "https://github.com/apirrone/Open_Duck_Mini/tree/v2/print (guida di stampa)",
        "bom": "https://tnkr.ai/explore/docs/open-duck-mini/open-duck-mini-v2 (guida di montaggio) e BOM "
               "Google Sheets linkata nel README upstream (sotto 400 $)",
        "training": "MuJoCo Playground (JAX / Brax PPO) con reference motion per imitazione, 50 Hz; "
                    "modelli attuatore identificati con BAM",
        "published_policy": "BEST_WALK_ONNX.onnx e BEST_WALK_ONNX_2.onnx nella radice del repo (MLP "
                            "101-512-256-128-28, attivazione swish, uscita tanh, osservazione con fase e "
                            "riferimenti di imitazione); non convertibili",
    },
    "sim": {"actuator": "position", "trunk_body": "base", "freejoint": "floating_base",
            "gyro_sensor": "gyro", "accel_sensor": "accelerometer", "home_z": 0.15},
    "servos": "14× Feetech STS3215 (bus seriale); runtime upstream su Raspberry Pi Zero 2W",
    "link": "bus",
    "policies": {},
    "notes": [
        "Stessi 14 giunti di MicroDuck, nello stesso ordine: stesso contratto di osservazione e di "
        "azione, con q0 e dimensioni proprie (robot alto 42 cm).",
        "La scena del Playground è MuJoCo nativa (attuatori di posizione STS3215, IMU sulla base): "
        "fetch_upstream.py la usa così com'è.",
        "Le policy pubblicate usano swish e tanh e un'osservazione di 101 valori con fase del passo: "
        "AP_NNMixer esegue solo MLP ELU sul contratto NNMixer, quindi la policy va riaddestrata.",
        "Il robot reale richiede il backend bus Feetech nel firmware (14 ≤ 16 funzioni servo, quindi "
        "SITL e HIL funzionano come per MicroDuck).",
    ],
}

# Kinematics, wiring and signs from Code/Server/Control.py, Servo.py and Tutorial.pdf (Step 13 wiring, Step 15
# assembly pose) of the upstream repo. q0 is the stand of Control.stop(): foot at x=10, y=99, z=+/-10 mm from
# each abduction axis, converted by coordinateToAngle() and the joint convention of robots/freenove/robot/freenove.xml.
ROBOTS["freenove"] = {
    "index": 10,
    "class": "quadruped",
    "display_name": "Freenove Robot Dog",
    "maker": "Freenove (kit FNK0050)",
    "status": "policy-sim",
    "joint_names": [
        "FL_hip_roll", "FL_hip_pitch", "FL_knee",
        "FR_hip_roll", "FR_hip_pitch", "FR_knee",
        "RL_hip_roll", "RL_hip_pitch", "RL_knee",
        "RR_hip_roll", "RR_hip_pitch", "RR_knee",
    ],
    "q0": [0.1007, 0.6635, -0.0161, -0.1007, 0.6635, -0.0161,
           0.1007, 0.6635, -0.0161, -0.1007, 0.6635, -0.0161],
    "extra_cmd_dim": 0,
    "rate_hz": 50,
    "command_ranges": {"vx": [-0.2, 0.2], "vy": [-0.1, 0.1], "wz": [-0.8, 0.8]},
    "upstream": {
        "repo": "https://github.com/Freenove/Freenove_Robot_Dog_Kit_for_Raspberry_Pi",
        "branch": "master",
        "license": "CC BY-NC-SA 3.0 (uso non commerciale)",
        "sim_model_note": "nessun URDF, MJCF o CAD 3D upstream (solo Head_Part_2D.dwg): il modello "
                          "robots/freenove/robot/freenove.xml è ricostruito dalla cinematica di "
                          "Code/Server/Control.py, generato da tools/robots/freenove_mjcf.py e versionato in "
                          "questo repo. Per cambiare masse, servo o misure si modifica lo script e si rigenera: "
                          "`.venv/bin/python tools/robots/freenove_mjcf.py --gait`",
        "cad": "non pubblicato (parti in acrilico tagliate al laser; solo Head_Part_2D.dwg nel repo)",
        "bom": "Tutorial.pdf nel repo upstream (elenco parti, montaggio, cablaggio Step 13); servono 2× 18650 "
               "non protette e un Raspberry Pi 5 / 4B / 3B+",
        "training": "nessuno upstream (passo open-loop in Control.py: traiettorie ellittiche dei piedi + IK, "
                    "bilanciamento PID sull'IMU)",
        "published_policy": "nessuna",
    },
    "sim": {"actuator": "position", "mjcf": "freenove.xml", "trunk_body": "trunk",
            "freejoint": "floating_base", "gyro_sensor": "imu_gyro", "accel_sensor": "imu_acc",
            "home_z": 0.1014,
            # the foot sphere, not the whole shank: walk_v4 leaned on the shanks with the feet up and it counted
            # as stance
            "feet": [{"site": f"{leg}_foot", "body": f"{leg}_shank", "geom": f"{leg}_foot"}
                     for leg in ("FL", "FR", "RL", "RR")],
            "trot_pairs": [[0, 3], [1, 2]],                   # FL+RR, FR+RL
            "footfall_cycle": [2, 0, 3, 1]},                  # dog walk: RL, FL, RR, FR
    # Quadruped version of the MicroDuck / Microban velocity task (mjlab Go1 terms), scaled to a 0.55 kg
    # robot with 10 cm legs and 0.17 N.m servos:
    # - tracking std 0.12 m/s and 0.4 rad/s: half the commanded range, as Go1 (std 0.5 on +/-1 m/s); the
    #   Microban 0.32 m/s would pay 75% of the reward for standing still at 0.15 m/s
    # - trot on the diagonal pairs instead of the biped single stance / alternation terms
    # - air time 0.05-0.3 s per foot and a 15 mm swing height: the upstream gait lifts 6 mm on a 0.7 s cycle
    # - posture std per joint type; abduction kept tight, it only steers lateral steps
    # - servo torque penalty: two legs in stance already load the knees to 2/3 of stall
    # - upright weight lowered: four feet keep the trunk level by themselves
    # Run walk (iterations 0-370) with in-range air time 0.05-0.3 s (weight 1.5), tracking std 0.12 m/s and
    # body_ang_vel -0.05 stood still and vibrated its feet: air time reached 3.1 per second, the largest term,
    # tracking stayed at 0.2 and roll/pitch rates near 2 rad/s. Air time is now paid at touchdown
    # (legged_gym: min(air, max) - min, negative for hops under 0.05 s). body_ang_vel -0.2 made every early
    # episode negative (-0.13 per second while flailing) and the policy learned to fall sooner: kept at -0.05.
    # Tracking uses the base twist low-passed over 0.25 s: on the instantaneous twist the upstream gait at the
    # right mean speed scored less than standing still. Checked on the model at 0.05 and 0.15 m/s: upstream
    # gait > standing > the vibrating policy of iteration 300.
    # Go2 / Go1 recipe (unitree_rl_lab Go2 velocity_env_cfg, MuJoCo Playground Go1 joystick, mjlab go2_velocity),
    # scaled to a 0.55 kg robot with 0.15 m/s commands. Replaces walk_v1..v11, where a growing list of gait
    # shaping terms (three/all/under stance, no-progress, alive, fall penalty, penalty curriculum) fought
    # velocity tracking and the best policy moved at a third of the command. Differences taken from Go2/Go1:
    # - step reward clipped at 0 (legged_gym only_positive_rewards, Playground clip): ending an episode never
    #   pays, so alive, the fall penalty and the penalty curriculum are no longer needed
    # - illegal contact ends the episode (trunk, head, hips, thighs on the floor), as mjlab / unitree_rl_lab
    # - privileged critic (Playground privileged_state): true base twist, gravity, height, foot contacts
    # - tracking weights raised after walk_v12: at the errors the policy actually had (0.15 m/s, 1.5 rad/s)
    #   exp(-err²/0.005) was ~1% and the yaw kernel was ~0 because roll/pitch sat inside it. The step reward
    #   was clipped to 0, so velocity had no advantage. std is now half the command (0.14 m/s, 0.5 rad/s)
    #   and the kernel is the command error only.
    # - walk_v13 added tracking after the only_positive clip: every penalty was clipped away, it walked at
    #   the command but with the rear hips splayed to +/-1.13 rad and the trunk at 83 mm. The clip is on the
    #   whole step again, and the posture is a hard limit that ends the episode (trunk under 85 mm, hip
    #   abduction over 0.5 rad, hip pitch over 0.7 rad from the stand pose). The walk_v12 gait, upright
    #   and on its feet, reached 0.36 rad of abduction and 0.46 rad of hip pitch in its first steps.
    # - regular gait from air_time_variance (Go2 -1.0) instead of stance-count terms; air time at touchdown
    # - flat orientation penalty (Go2 -2.5, Go1 -5) instead of the upright reward; stand_still (Go1 -1)
    # Kept from the Freenove runs: filtered twist for tracking, foot-only contacts, posture penalty, joint
    # limit penalty, joint speed and action rate for a smooth gait, single-axis commands, 0.15 s steps.
    "reward": {
        "only_positive": True,
        "track_lin_vel": 6.0, "tracking_sigma": 0.01, "tracking_filter_s": 0.25,
        # walk_v19 iteration 3000 turned at 0.25 rad/s by pivoting on two diagonal feet, one rear foot held
        # in the air, the other front foot shuffling at 6 mm; backward it stood still. In a pure turn the
        # linear term paid its full 6/s for standing in place while the yaw term was worth 3 at most, and
        # holding a foot up cost nothing: yaw weight 5, foot_hold penalty, more backward commands.
        "track_ang_vel": 5.0, "tracking_sigma_ang": 0.15,
        "tracking_lin_z": 0.0, "tracking_ang_rp": 0.0,
        # walk_v14 stood still on every command (vx 0.000, survival 100%): standing paid 32% of the linear
        # tracking, height and posture, at no risk. Kernel std now 0.1 m/s / 0.39 rad/s (standing at 0.15
        # m/s pays 10%), the positive posture terms are small, and no_progress takes the standing reward
        # away while a move is commanded (vanishes once the command is followed).
        "no_progress": -2.5,
        "upright": 0.0, "orientation": -5.0,
        "pose": 0.3, "walking_threshold": 0.01,
        "pose_std_standing": {".*hip_roll": 0.05, ".*hip_pitch": 0.1, ".*knee": 0.1},
        "pose_std_walking": {".*hip_roll": 0.08, ".*hip_pitch": 0.35, ".*knee": 0.35},
        "pose_l2": -0.3, "stand_still": -1.0,
        "action_rate": -0.1, "alive": 0.0,
        "body_ang_vel": -0.05, "dof_pos_limits": -10.0, "self_collisions": -1.0,
        "joint_torque": -0.05, "joint_vel": -0.001,
        # walk_v15 / v16 (iterations 500-1600) turned at the command but crept forward at 0.03 m/s with
        # 5-8 foot lifts per second per foot: a tremble, not steps. air_time at weight 2 cost it 0.02/s
        # against 2.5/s of tracking. Weight 5 with a 0.10 s minimum: a 0.04 s hop costs 0.3, a 0.25 s
        # step pays 0.75 (Go2: 0.25 * (air - 0.5) per touchdown against 1.5/s of tracking).
        # walk_v17 iteration 2000 (score 0.73, forward at the command): 263 touchdowns in 10 s, 6.6 per
        # foot per second, flights of 0.08 s, 10 mm lift, in the right footfall order. A scurry: the
        # per-touchdown footfall_sequence reward (2.1/s) paid for cadence. 0.15 m/s with 5.5 cm leg
        # segments is about 3 steps per second per foot (5 cm stride): flights of 0.12-0.25 s. Only such
        # steps count for air_time and footfall_sequence, and the sequence reward is halved.
        # Contact timelines at iteration 2000/2100: a 4 Hz trot with 22-33 mm lifts (v17), then a walk
        # with the rear feet dragged at 5-9 mm (v18); isolated contact bounces inside a phase counted as
        # 0.02 s steps. Bounces under 0.04 s are ignored; landing with a low peak costs more.
        "air_time": 5.0, "air_time_mode": "touchdown", "air_time_min_s": 0.12, "air_time_max_s": 0.35,
        "air_time_debounce_s": 0.04, "foot_hold": -2.0,
        "air_time_variance": -1.0,
        "foot_clearance": -2.0, "foot_swing_height": -1.0, "swing_height_m": 0.015, "foot_slip": -0.1,
        "footfall_sequence": 0.1, "alternation_min_air_s": 0.12, "alternation_min_height_frac": 0.5,
        "undesired_contacts": -1.0,
        "base_height": 0.3, "base_height_target_m": 0.10, "base_height_std_m": 0.015,
    },
    "env": {
        "resample_s": [3.0, 8.0], "p_zero_command": 0.02, "p_no_lateral": 0.1, "p_single_axis": 0.8,
        # [+vx, -vx, +vy, -vy, +wz, -wz]: backward and the turns, still untracked at walk_v19, are drawn more
        "single_axis_weights": [0.15, 0.30, 0.10, 0.10, 0.175, 0.175],
        "push": {"interval_s": [4.0, 8.0], "vel_xy": 0.05},
        "terminate_on_illegal_contact": True,
        "posture_limits": {"min_height_m": 0.085, "max_dev": {".*hip_roll": 0.5, ".*hip_pitch": 0.7}},
        "curriculum": {
            "action_rate_stages": [[0, -0.1], [24000, -0.2], [36000, -0.4]],
            "standing_stages": [[0, 0.02], [12000, 0.05], [18000, 0.1], [24000, 0.15],
                                [36000, 0.2], [48000, 0.25]],
        },
        "eval_modes": {"stand": [0.0, 0.0, 0.0], "forward_0.15": [0.15, 0.0, 0.0],
                       "backward_0.15": [-0.15, 0.0, 0.0], "lateral_0.1": [0.0, 0.1, 0.0],
                       "turn_0.6": [0.0, 0.0, 0.6]},
    },
    "privileged_critic": True,
    # Go2 action scale: the network works in units of 0.25 rad (folded into the exported .nnm), the
    # exploration std is 0.8 units = 0.2 rad. With raw radians the adaptive learning rate sat at its
    # 1e-5 floor from the first iterations of every run (KL overshoot on each Adam step). 0.4 rad of
    # noise broke the 0.5 rad hip posture limit within 8 steps.
    "action_scale": 0.25,
    "init_noise_std": 0.8,
    "servos": "12× EMAX ES08MA II (12 g, analogici, 1,6 kgf·cm a 4,8 V) su PCA9685 0x40 a 50 Hz; "
              "Raspberry Pi, IMU MPU6050",
    "link": "pwm",
    "policies": {
        "walk_v20_it3400.nnm": "policy di riferimento: ricetta Go2 (only_positive, contatti illegali e limiti "
                               "di postura che chiudono l'episodio, critico privilegiato, action_scale 0,25) "
                               "ripresa da walk_v15 → v19 con air time a touchdown, debounce dei contatti, "
                               "foot_hold e no_progress. Iterazione 3400 (17000 epoche PPO, punteggio 0,86). "
                               "Sopravvive sempre, tronco a 10 cm, zampe chiuse: avanti 0,14 m/s a comando "
                               "0,15, rotazione 0,80 rad/s a comando 0,6, indietro 0,08 m/s, laterale 0,07 "
                               "m/s. Passo a quattro zampe in ogni direzione (2–3 Hz per zampa).",
        "walk_v20.nnm": "stesso run, iterazione 5000 (punteggio 0,865, il massimo): avanti 0,15 m/s esatto, "
                        "rotazione 0,56 rad/s, indietro 0,07, laterale 0,06. Con l'esplorazione ormai a 0,06 "
                        "rad il passo è diventato asimmetrico (avanza soprattutto con FR e RL, FL e RR quasi "
                        "sempre a terra): punteggio pari a walk_v20_it3400 ma passo meno naturale.",
        "walk_v19.nnm": "iterazione 3000 di walk_v19 (punteggio 0,61), il checkpoint da cui è partito walk_v20: "
                        "avanti 0,11 m/s con passo a 2 Hz e sollevamenti di 27–29 mm davanti; ruotava "
                        "strisciando su una zampa e indietro stava fermo.",
        "walk_v7.nnm": "addestrata da zero sul contratto ArduPilot in int8 (QAT), ambiente walk_v7 "
                       "(passo da cane, un comando per asse). Checkpoint dell'iterazione 3000 "
                       "(15000 epoche PPO, punteggio 0,59). In valutazione sopravvive sempre: "
                       "avanti 0,21 m/s a comando 0,15; indietro, laterale e rotazione non ancora seguiti.",
    },
    # Each clip is the int8 policy. "env" is that run's reward configuration; one iteration is 5 PPO epochs.
    "videos": [{
        "env": "walk_v20",
        "iteration": "3400",
        "epochs": "17000",
        "latest": True,
        "mp4": "freenove_walk_v20_it3400_full.mp4",
        "caption": "Policy di riferimento, sequenza completa (play_policy.py --full): avanti 5 s (+7°), "
                   "indietro 5 s a 0,14 m/s, laterale 4 s, destra 3 s (−134°), sinistra 180° in 4,1 s, "
                   "avanti 5 s. Percorso 3,05 m, nessuna caduta, tutte e quattro le zampe in ogni direzione.",
    }, {
        "env": "walk_v20",
        "iteration": "5000",
        "epochs": "25000",
        "mp4": "freenove_walk_v20_it5000_full.mp4",
        "caption": "Ultimo checkpoint (punteggio 0,865): avanti a 0,15 m/s esatti e indietro dritto (+3°), "
                   "ma il passo usa soprattutto la coppia FR+RL.",
    }, {
        "env": "walk_v19",
        "iteration": "3000",
        "epochs": "15000",
        "mp4": "freenove_walk_v19_it3000.mp4",
        "caption": "Passo a 2 Hz con appoggi lunghi, avanti perfettamente dritto (−2°, 0°); la rotazione a "
                   "sinistra non arriva a 180° in 10 s: ruotava strisciando su una zampa.",
    }, {
        "env": "walk_v17",
        "iteration": "1800",
        "epochs": "9000",
        "mp4": "freenove_walk_v17_it1800.mp4",
        "caption": "Primo checkpoint che segue la velocità: 1,06 m netti in 17 s, ma con un trotto a 4 Hz "
                   "(air_time da 2 a 5). La ricetta Go2 con il kernel del tracking allargato.",
    }, {
        "env": "walk_v15",
        "iteration": "1100",
        "epochs": "5500",
        "mp4": "freenove_walk_v15_it1100.mp4",
        "caption": "Postura alta e zampe chiuse imposte dai limiti di postura (tronco > 85 mm, anche entro "
                   "0,5/0,7 rad). Rotazioni complete, avanti a un terzo del comando.",
    }, {
        "env": "walk_v12",
        "iteration": "1500",
        "epochs": "7500",
        "mp4": "freenove_walk_v12_it1500.mp4",
        "caption": "Prima ricetta Go2 (only_positive, contatti illegali, critico privilegiato): in piedi e "
                   "alto, ma con il kernel del tracking a 0,07 m/s il reward di velocità era ~0 e stava fermo.",
    }, {
        "env": "walk_v7",
        "iteration": "2700",
        "epochs": "13500",
        "mp4": "freenove_walk_v7_it2700.mp4",
        "caption": "Ultimo video della serie v1–v7 (shaping incrementale). "
                   "Non cade, tronco a 10 cm. Avanti dritto a circa 0,2 m/s; indietro, laterale e "
                   "rotazione restano fermi. Circa 20 atterraggi al secondo.",
    }, {
        "env": "walk_v7",
        "iteration": "2000",
        "epochs": "10000",
        "mp4": "freenove_walk_v7_it2000.mp4",
        "caption": "Cammina avanti, alto sui piedi, più calmo di walk_v5: giunti 4,5 rad/s, "
                   "due o più piedi a terra l'80% del tempo.",
    }, {
        "env": "walk_v7",
        "iteration": "1600",
        "epochs": "8000",
        "mp4": "freenove_walk_v7_it1600.mp4",
        "caption": "Ripresa da walk_v6 con la penalità per lo stare fermo. Ancora poco spostamento "
                   "nella direzione comandata.",
    }, {
        "env": "walk_v5",
        "iteration": "900",
        "epochs": "4500",
        "mp4": "freenove_walk_v5_it0900.mp4",
        "caption": "Primo checkpoint che avanza (0,17 m/s) con la sequenza dei passi del cane, "
                   "ma frenetico: 7,7 rad/s e circa 260° di rotazione nei 5 s di avanti.",
    }, {
        "env": "walk_v5",
        "iteration": "0–200",
        "epochs": "0–1000",
        "mp4": "freenove_evolution_v5.mp4",
        "caption": "Alto sui piedi e solleva una zampa alla volta, senza seguire la direzione.",
    }, {
        "env": "walk_v4",
        "iteration": "0–900",
        "epochs": "0–4500",
        "mp4": "freenove_evolution_v4.mp4",
        "caption": "Dall'iterazione 900 si sposta saltando e ruotando, non con un passo.",
    }, {
        "env": "walk_v2",
        "iteration": "0–400",
        "epochs": "0–2000",
        "mp4": "freenove_evolution_v2.mp4",
        "caption": "Impara solo a stare in piedi, accucciato e fermo, qualunque sia il comando.",
    }, {
        "env": "walk",
        "iteration": "0–300",
        "epochs": "0–1500",
        "mp4": "freenove_evolution_it0300.mp4",
        "caption": "Prima configurazione. Dalla iterazione 100 resta in piedi vibrando le zampe "
                   "sul posto, senza camminare.",
    }, {
        "env": "walk",
        "iteration": "100",
        "epochs": "500",
        "mp4": "freenove_walk_it0100.mp4",
        "caption": "Sequenza completa dei comandi. In piedi per 26 s, spostamento netto 0,21 m, "
                   "rotazione non comandata.",
    }],
    # grandezza, Freenove, AlbertPro, fonte Freenove
    "mechanics": {
        "compare_with": "AlbertPro",
        "rows": [
            ("Gradi di libertà per zampa", "3: abduzione, anca, ginocchio", "2: anca, ginocchio",
             "`Control.coordinateToAngle()`"),
            ("Giunti comandati", "12", "8", "Step 13 del tutorial (12 servo zampe + 1 testa)"),
            ("Interasse anche (x × y)", "136 × 76 mm", "110 × 110 mm", "`Control.postureBalance()`: l, b"),
            ("Asse abduzione → asse anca", "23 mm, verticale a riposo", "—", "`coordinateToAngle()`: l1"),
            ("Coscia / tibia", "55 / 55 mm", "30 / 42 mm nel MJCF (5 / 5 cm dichiarati)",
             "`coordinateToAngle()`: l2, l3"),
            ("Configurazione della zampa", "coscia indietro, tibia in avanti (ginocchio verso dietro)",
             "uguale", "`angleToCoordinate()` e foto dello stand"),
            ("Piede in stand rispetto all'anca", "+10 mm avanti, 10 mm verso l'esterno, 99 mm sotto",
             "−3 mm, 8 mm, 56 mm sotto", "`Control.stop()`"),
            ("Massa", "circa 0,55 kg (stima per componenti)", "1,38 kg nel MJCF (densità di default sulle mesh)",
             "non pubblicata; 970 g è il peso della confezione"),
            ("Servo", "12× EMAX ES08MA II, 0,16–0,20 N·m, 0,12 s/60°", "8 servo PWM non specificati",
             "elenco parti del tutorial; datasheet EMAX"),
            ("Corsa servo", "18°–162° (±72° attorno a 90°)", "limiti nel MJCF", "`Servo.angleMin/angleMax`"),
            ("Driver PWM", "PCA9685 0x40, 50 Hz, 500–2500 µs su 0–180°", "PCA9685 su ESP32", "`Servo.py`, `PCA9685.py`"),
            ("Attuatore nel MJCF", "posizione kp 2 N·m/rad, kv 0,02, coppia ±0,17 N·m",
             "posizione kp 60 senza limite di coppia", "datasheet ES08MA II"),
            ("IMU", "MPU6050 0x68 sulla shield", "nessuna nell'osservazione upstream", "`IMU.py`"),
        ],
    },
    # joint -> PCA9685 channel, servo_deg = 90 + sign * degrees(q) (+ calibration of point.txt)
    "servo_map": {
        "FL_hip_roll": (4, -1), "FL_hip_pitch": (3, 1), "FL_knee": (2, -1),
        "FR_hip_roll": (11, -1), "FR_hip_pitch": (12, -1), "FR_knee": (13, 1),
        "RL_hip_roll": (7, -1), "RL_hip_pitch": (6, 1), "RL_knee": (5, -1),
        "RR_hip_roll": (8, -1), "RR_hip_pitch": (9, -1), "RR_knee": (10, 1),
    },
    "notes": [
        "Rispetto ad AlbertPro, usato come base per la locomozione, il modello meccanico è diverso: 3 giunti "
        "per zampa invece di 2 (in più l'abduzione), segmenti di 55 mm invece di 30/42, anche su un "
        "rettangolo di 136 × 76 mm invece di 110 × 110. Restano uguali la configurazione della zampa "
        "(ginocchio verso dietro), il driver PCA9685 e l'attuatore di posizione; il resto è stato corretto. "
        "La tabella sopra elenca ogni voce.",
        "Lo zero dei giunti è la posa di montaggio del tutorial (tutti i servo a 90°): coscia verticale, tibia "
        "orizzontale in avanti. Così un giunto a 0 corrisponde a 1500 µs sia sul filo NNMixer sia sul servo.",
        "Scala da applicare in robot.bin: il filo NNMixer vale 3 mrad/µs, il servo Freenove 2000 µs su π rad "
        "(1,571 mrad/µs), quindi µs_servo = 1500 + segno × 1,910 × (µs_filo − 1500), più l'offset di "
        "calibrazione del singolo servo.",
        "Modello MuJoCo: geometrie visive (gruppo 2) sulle foto del tutorial, geometrie di collisione "
        "(gruppo 3) che portano le masse, keyframe `home` nello stand, camere `track`, `side`, `front`, "
        "sensori IMU e di contatto ai piedi. Verifica: il passo open-loop di Control.py, eseguito sul modello "
        "con la sua IK, avanza di 17 cm in 3,5 s e ruota di 50° in 3,5 s senza cadere "
        "([video](../media/freenove_model_gait.mp4)).",
        "Masse e rigidezza dei servo sono stime: pesare il robot montato e aggiornare MASS in "
        "tools/robots/freenove_mjcf.py. "
        "In stand su 4 zampe il ginocchio lavora a circa 0,05 N·m; al trotto, con 2 zampe in appoggio, circa "
        "0,11 N·m statici, due terzi dello stallo a 4,8 V: per questo vx è limitata a ±0,2 m/s.",
        "Servo analogici aggiornati a 50 Hz dal PCA9685: la policy a 50 Hz è il massimo utile.",
        "I 12 servo si collegano direttamente alle uscite dell'autopilota (12 ≤ 16) al posto del PCA9685; "
        "manca ancora in robot.bin la calibrazione per giunto.",
        "Licenza CC BY-NC-SA 3.0: modello e policy derivati non vanno usati per scopi commerciali.",
    ],
}

# docs/robots/img/<id>.jpg, resized copies of the photo each upstream README shows; credit and source
# are printed under the image on the robot page.
PHOTOS = {
    "microduck": ("robot reale (metà destra del confronto sim/reale)", "https://github.com/pollen-robotics/microduck_rl"),
    "microban": ("robot montato", "https://github.com/Rhoban/microban"),
    "zeroth": ("Zeroth-01", "https://github.com/zeroth-robotics/zeroth-bot"),
    "bimo": ("robot montato", "https://github.com/mekion/the-bimo-project"),
    "legolas": ("robot montato, vista frontale", "https://github.com/daviddoo02/Legolas-an-open-source-biped"),
    "upkie": ("robot montato", "https://github.com/upkie/upkie"),
    "rex": ("SpotMicro montato", "https://github.com/nicrusso7/rex-gym"),
    "yertle": ("fotogramma del video di camminata del robot reale", "https://github.com/Jerome-Graves/yertle"),
    "albert": ("render MuJoCo (il repo pubblica solo render)", "https://github.com/thinking0things/AlbertPro"),
    "openduck": ("robot montato", "https://github.com/apirrone/Open_Duck_Mini"),
    "freenove": ("render del client Freenove (Tutorial.pdf, capitolo 4)",
                 "https://github.com/Freenove/Freenove_Robot_Dog_Kit_for_Raspberry_Pi"),
}

STATUS_TEXT = {
    "policy": "policy int8 disponibile; la stessa rete in float32 è validata in SITL e HIL",
    "policy-upstream": "walk_md.nnm addestrata sul contratto ArduPilot; walk.nnm upstream da rifinire",
    "policy-sim": "policy int8 addestrata in simulazione; video dei checkpoint nella scheda",
    "needs-training": "scena MuJoCo generata dall'URDF; policy da addestrare",
    "needs-training-mjcf": "scena MuJoCo nativa pronta; policy da addestrare",
    "needs-training-handmade": "scena MuJoCo ricostruita dalla cinematica upstream; policy da addestrare",
    "needs-model": "manca una scena MuJoCo pronta per il training",
    "needs-firmware": "serve un tipo di azione per giunto nel firmware (ruote in velocità)",
}

LINK_TEXT = {
    "bus": "servo su bus seriale: serve il backend bus nel firmware (non ancora scritto)",
    "pwm": "servo PWM: collegabili alle uscite dell'autopilota",
    "can": "attuatori CAN-FD mjbots: serve un backend dedicato",
}
