# Sprite (Sprite0825)

[Catalogo robot](README.md) · [Training compatibile con ArduPilot](training.md) · [README](../../README.it.md)

| | |
|---|---|
| Id (cartella) | `open_sprite` |
| `NNM_ROBOT` | **20** |
| Classe | bipede |
| Progetto | Beijing Dynamics |
| Stato | manca una scena MuJoCo pronta per il training |
| Giunti comandati | 31 |
| Osservazione | 102 valori |
| Frequenza policy | 50 Hz |
| Attuatori | 31 giunti attuati. La caviglia è un differenziale di due Damiao DM-J4310P; il mapping motore/giunto non è nel MJCF. Protocollo del bus non pubblicato |
| Collegamento | servo su bus seriale: serve il backend bus nel firmware (non ancora scritto) |
| Licenza upstream | GNU AGPL-3.0 (modello e codice del repo). I dataset di motion di terze parti non sono ridistribuiti |

## Repo originale e file di base

- Repository: [github.com/BeijingDynamics/open_sprite](https://github.com/BeijingDynamics/open_sprite)
- Scena MuJoCo upstream: [`models/sprite0825/mujoco/sprite0825_external_pd.xml`](https://github.com/BeijingDynamics/open_sprite/blob/main/models/sprite0825/mujoco/sprite0825_external_pd.xml)
- CAD e parti stampabili: mesh STL del 25 agosto 2026 in models/sprite0825/meshes/ (AGPL-3.0)
- BOM e montaggio: umanoide circa 0,95 m, bacino a V; l'URDF somma 14,65 kg. La caviglia fisica accoppia due motori DM-J4310P con aste e un giunto cardanico: nel MJCF pitch e roll sono coordinate generalizzate. Gli altri motori e il bus non sono nel repo
- Training upstream: Isaac Lab sull'URDF sanitized v4, policy a 50 Hz. V38 è un traguardo di whole-body tracking (osservazione 164, azione = 31 residui di posizione, senza velocità di base né posizione globale dell'ancora) qualificato in Isaac Lab e MuJoCo; lo stage 2 è locomozione a comando (stare fermo, partire e fermarsi, avanti, rotazione). docs/locomotion_milestones.md
- Policy upstream: nessun peso nel repo. Il pacchetto V38 (SHA-256 b8d1851de07e654c367d0ad9d6d2e32a9fbc9588e4f26aabd7e5e4d0d71166ff) non è pubblicato: la traiettoria viene da AMASS/BMLrub e i termini di redistribuzione sono in revisione. Anche pubblicato, 164 → 31 non è il contratto NNMixer

Per scaricare il repo originale e preparare la scena usata da simulazione e training:

```bash
.venv/bin/python tools/robots/fetch_upstream.py --robot open_sprite
```

Nota sul modello: MJCF a base flottante (pelvis_joint) senza attuatori (nu = 0) e senza sensori IMU: la coppia si scrive in qfrc_applied. La scena del viewer (models/sprite0825/mujoco/scene.xml) ha i giunti bloccati. L'URDF di training è models/sprite0825/sprite0825_sanitized_v4.urdf. Nessuno dei tre è ancora copiato in robots/open_sprite/robot/.

## Topologia

File: [`robots/open_sprite/robot/profile.json`](../../robots/open_sprite/robot/profile.json) → `robot.bin` sulla microSD. Ordine dei giunti = ordine di osservazione, azione e uscite servo.

| # | Giunto | q0 (rad) | Uscita servo |
|---:|---|---:|---|
| 0 | `left_hip_pitch_joint` | +0.0000 | `SERVO1_FUNCTION 94` |
| 1 | `left_hip_roll_joint` | +0.0000 | `SERVO2_FUNCTION 95` |
| 2 | `left_hip_yaw_joint` | +0.0000 | `SERVO3_FUNCTION 96` |
| 3 | `left_knee_joint` | +0.0000 | `SERVO4_FUNCTION 97` |
| 4 | `left_ankle_pitch_joint` | +0.0000 | `SERVO5_FUNCTION 98` |
| 5 | `left_ankle_roll_joint` | +0.0000 | `SERVO6_FUNCTION 99` |
| 6 | `right_hip_pitch_joint` | +0.0000 | `SERVO7_FUNCTION 100` |
| 7 | `right_hip_roll_joint` | +0.0000 | `SERVO8_FUNCTION 101` |
| 8 | `right_hip_yaw_joint` | +0.0000 | `SERVO9_FUNCTION 102` |
| 9 | `right_knee_joint` | +0.0000 | `SERVO10_FUNCTION 103` |
| 10 | `right_ankle_pitch_joint` | +0.0000 | `SERVO11_FUNCTION 104` |
| 11 | `right_ankle_roll_joint` | +0.0000 | `SERVO12_FUNCTION 105` |
| 12 | `waist_roll_joint` | +0.0000 | `SERVO13_FUNCTION 106` |
| 13 | `waist_yaw_joint` | +0.0000 | `SERVO14_FUNCTION 107` |
| 14 | `left_shoulder_pitch_joint` | +0.0000 | `SERVO15_FUNCTION 108` |
| 15 | `left_shoulder_roll_joint` | +0.0000 | `SERVO16_FUNCTION 109` |
| 16 | `left_shoulder_yaw_joint` | +0.0000 | oltre Scripting16: serve il backend bus |
| 17 | `left_elbow_joint` | +0.0000 | oltre Scripting16: serve il backend bus |
| 18 | `left_wrist_yaw_joint` | +0.0000 | oltre Scripting16: serve il backend bus |
| 19 | `left_wrist_pitch_joint` | +0.0000 | oltre Scripting16: serve il backend bus |
| 20 | `left_wrist_roll_joint` | +0.0000 | oltre Scripting16: serve il backend bus |
| 21 | `right_shoulder_pitch_joint` | +0.0000 | oltre Scripting16: serve il backend bus |
| 22 | `right_shoulder_roll_joint` | +0.0000 | oltre Scripting16: serve il backend bus |
| 23 | `right_shoulder_yaw_joint` | +0.0000 | oltre Scripting16: serve il backend bus |
| 24 | `right_elbow_joint` | +0.0000 | oltre Scripting16: serve il backend bus |
| 25 | `right_wrist_yaw_joint` | +0.0000 | oltre Scripting16: serve il backend bus |
| 26 | `right_wrist_pitch_joint` | +0.0000 | oltre Scripting16: serve il backend bus |
| 27 | `right_wrist_roll_joint` | +0.0000 | oltre Scripting16: serve il backend bus |
| 28 | `head_pitch_joint` | +0.0000 | oltre Scripting16: serve il backend bus |
| 29 | `head_roll_joint` | +0.0000 | oltre Scripting16: serve il backend bus |
| 30 | `head_yaw_joint` | +0.0000 | oltre Scripting16: serve il backend bus |

Osservazione (102): gyro FLU 3, gravità FLU 3, q−q0 31, q̇ 31, azione precedente 31, twist vx vy ωz 3.
Azione (31): offset in radianti, `q_target = q0 + azione`.

## Architettura PPO

File: [`robots/open_sprite/robot/ppo.yaml`](../../robots/open_sprite/robot/ppo.yaml). Stessa rete di MicroDuck, già provata su Pixhawk 6C: **102 → 512 → 256 → 128 → 31**, ELU, normalizzazione dell'osservazione incorporata. Pesi int8 per riga addestrati sulla griglia int8 dalla prima iterazione (QAT), attivazioni float32. PPO: 2048 ambienti × 24 passi, 5 epoche, 4 minibatch, lr 1e-3 adattivo (KL 0,01), γ 0,99, λ 0,95, clip 0,2.

## Policy

Cartella: [`robots/open_sprite/policies/`](../../robots/open_sprite/policies/) → sulla microSD `/APM/nnm/open_sprite/policies/`. `NNM_POLICY` = indice del file in ordine alfabetico.

Nessuna policy ancora: va addestrata (sezione successiva).

## Training compatibile con ArduPilot

L'ambiente è già il deployment: fisica a 200 Hz come il loop dell'autopilota, gravità dal filtro IMU del firmware, azione tagliata a `NNM_ACT_MAX` e codificata in PWM, osservazione nell'ordine del profilo. Dettagli in [training.md](training.md).

```bash
.venv/bin/python tools/robots/nnm_env.py --robot open_sprite                       # carica la scena, passo a policy zero
.venv/bin/python tools/robots/train_velocity.py --robot open_sprite --envs 8 --iters 200 --name walk
.venv/bin/python tools/robots/train_velocity.py --robot open_sprite --eval robots/open_sprite/policies/walk.nnm
```

Il trainer CPU serve per verifiche e rifiniture brevi. Per una policy completa (2048 ambienti × 2000 iterazioni) si usa mjlab su GPU con gli stessi due pezzi: `deploy_contract.py` per osservazione e azione, `nnm_qat.enable_qat()` sull'actor, `export_nnm_from_actor()` per il file.

## Deploy sull'autopilota

```bash
python tools/robots/pack_robot_bin.py --robot open_sprite          # robot.bin dal profilo
# microSD: /APM/nnm/open_sprite/robot.bin  e  /APM/nnm/open_sprite/policies/*.nnm
```

Con 31 giunti il profilo supera `NNM_MAX_JOINTS` 20 e l'osservazione supera `NNM_MAX_OBS` 96: il firmware rifiuta `robot.bin` finché quei limiti non crescono.

## Note

- 31 giunti e osservazione 102 superano `NNM_MAX_JOINTS` 20 e `NNM_MAX_OBS` 96: il firmware rifiuta questo `robot.bin` finché i due limiti non crescono. L'indice `NNM_ROBOT` 20 è già assegnato, così l'ordine del catalogo non si sposta.
- I nomi e l'ordine sono quelli di `sprite0825_external_pd.xml`: gamba sinistra e destra (pitch, roll, yaw d'anca, ginocchio, pitch e roll di caviglia), vita roll e yaw, braccio sinistro e destro (pitch, roll, yaw di spalla, gomito, yaw, pitch e roll di polso), testa pitch, roll e yaw. Gli assi destri di pitch sono specchiati (ginocchio destro `0 -1 0`).
- q0 = 0 è la posa di assemblaggio dell'URDF, quella della scena bloccata del viewer. `home_z` 0,65 m è la z del bacino scritta nel MJCF, non un'altezza misurata dopo il contatto dei piedi. Il README indica un robot di circa 0,95 m.
- Prima del training la scena va copiata con le mesh e completata: 31 attuatori (oggi la coppia è esterna), un'IMU sul bacino, e il mapping del differenziale di caviglia se si comanda il motore vero. I guadagni PD non sono nel repo.
- La policy V38 non è nel repo e non è un attore a frame singolo: va riaddestrata sul contratto, dopo aver allargato il firmware oppure dopo aver scelto un sottoinsieme di giunti (per esempio le 12 gambe, come ToddlerBot) e tenuto il resto a q0.
