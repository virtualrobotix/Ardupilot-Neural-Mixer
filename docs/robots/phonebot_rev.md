# PhoneBot invertito

[Catalogo robot](README.md) · [Training compatibile con ArduPilot](training.md) · [README](../../README.it.md)

| | |
|---|---|
| Id (cartella) | `phonebot_rev` |
| `NNM_ROBOT` | **19** |
| Classe | bipede |
| Progetto | UCLA RoMeLa (Ruochen Hou, Quanyou Wang, Daniel Koh, Dennis Hong) |
| Stato | scena MuJoCo ricostruita dalla cinematica upstream; policy da addestrare |
| Giunti comandati | 13 |
| Osservazione | 50 valori |
| Frequenza policy | 50 Hz |
| Attuatori | 13× DYNAMIXEL XL430-W250-T come PhoneBot. Con le gambe ruotate la gamba fisica del disegno J7–J12 sta a destra e prende i nomi right_*, quella J1–J6 sta a sinistra (left_*): ogni servo tiene limiti e verso, cambiano solo gli ID sul bus. Sull'hardware: backend Dynamixel TTL su Pixhawk |
| Collegamento | servo su bus seriale: serve il backend bus nel firmware (non ancora scritto) |
| Licenza upstream | CAD, STEP, BOM e guide: CC BY-NC 4.0; software promesso come open source ma nessun repository pubblico è linkato al 24 ottobre 2026 |

## Repo originale e file di base

- Repository: [phonebot.dev/](https://phonebot.dev/)
- Modello MuJoCo (in questo repo): [`robots/phonebot_rev/robot/phonebot_rev.xml`](../../robots/phonebot_rev/robot/phonebot_rev.xml)
- CAD e parti stampabili: phonebot.dev/assets/models/PhoneBot-preview.glb (anteprima, 12 mesh), assets/PhoneBot_Asm.glb (assemblato completo), assets/PhoneBot_Assembly_STEP.zip e assets/Phonebot_BOM.xlsx (CC BY-NC 4.0); Onshape e STL non pubblicati al 24 ottobre 2026
- BOM e montaggio: 13× DYNAMIXEL XL430-W250-T, batteria 3S, U2D2 e SBC; 48,3 cm, 1,8 kg incluso smartphone, ≈400 USD escluso telefono. Il nostro assetto usa Pixhawk + ESP32, non l'SBC upstream
- Training upstream: PPO asimmetrico MJX/MuJoCo a 50 Hz: attore 52 (gyro 3, gravità 3, comando 3, q/qdot/azione precedente 39, fase piedi 4) → 13, critico 120; modello XL430 torque-aware, randomizzazione di guadagni, limiti, backlash e spinte; mirror sagittale del batch
- Policy upstream: policy TensorFlow Lite descritta nel paper ma pesi/repository non pubblicati; 52 → 13, ordine e fase differenti dal contratto NNMixer; da riaddestrare

Per scaricare il repo originale e preparare la scena usata da simulazione e training:

```bash
.venv/bin/python tools/robots/fetch_upstream.py --robot phonebot_rev
```

Nota sul modello: variante di PhoneBot: phonebot_rev.xml (phonebot_mjcf.py --reversed) usa le mesh del GLB ufficiale con bacino e gambe ruotati di 180° sotto il busto; telefono e IMU guardano il vecchio dietro delle gambe e le ginocchia piegano all'indietro. Masse e collisioni come phonebot.

## Topologia

File: [`robots/phonebot_rev/robot/profile.json`](../../robots/phonebot_rev/robot/profile.json) → `robot.bin` sulla microSD. Ordine dei giunti = ordine di osservazione, azione e uscite servo.

| # | Giunto | q0 (rad) | Uscita servo |
|---:|---|---:|---|
| 0 | `right_hip_yaw` | +0.0000 | `SERVO1_FUNCTION 94` |
| 1 | `right_hip_roll` | +0.0000 | `SERVO2_FUNCTION 95` |
| 2 | `right_hip_pitch` | -0.5000 | `SERVO3_FUNCTION 96` |
| 3 | `right_knee_pitch` | +1.0000 | `SERVO4_FUNCTION 97` |
| 4 | `right_ankle_pitch` | -0.5000 | `SERVO5_FUNCTION 98` |
| 5 | `right_ankle_roll` | +0.0000 | `SERVO6_FUNCTION 99` |
| 6 | `left_hip_yaw` | +0.0000 | `SERVO7_FUNCTION 100` |
| 7 | `left_hip_roll` | +0.0000 | `SERVO8_FUNCTION 101` |
| 8 | `left_hip_pitch` | -0.5000 | `SERVO9_FUNCTION 102` |
| 9 | `left_knee_pitch` | +1.0000 | `SERVO10_FUNCTION 103` |
| 10 | `left_ankle_pitch` | -0.5000 | `SERVO11_FUNCTION 104` |
| 11 | `left_ankle_roll` | +0.0000 | `SERVO12_FUNCTION 105` |
| 12 | `torso_yaw` | +0.0000 | `SERVO13_FUNCTION 106` |

Osservazione (50): gyro FLU 3, gravità FLU 3, q−q0 13, q̇ 13, azione precedente 13, twist vx vy ωz 3, orologio del passo sin, cos 2 (`NNM_CLOCK_HZ`; zero da fermi).
Azione (13): offset in radianti, `q_target = q0 + azione`.

## Architettura PPO

File: [`robots/phonebot_rev/robot/ppo.yaml`](../../robots/phonebot_rev/robot/ppo.yaml). Stessa rete di MicroDuck, già provata su Pixhawk 6C: **50 → 512 → 256 → 128 → 13**, ELU, normalizzazione dell'osservazione incorporata. Pesi int8 per riga addestrati sulla griglia int8 dalla prima iterazione (QAT), attivazioni float32. PPO: 2048 ambienti × 24 passi, 5 epoche, 4 minibatch, lr 1e-3 adattivo (KL 0,01), γ 0,99, λ 0,95, clip 0,2.

## Policy

Cartella: [`robots/phonebot_rev/policies/`](../../robots/phonebot_rev/policies/) → sulla microSD `/APM/nnm/phonebot_rev/policies/`. `NNM_POLICY` = indice del file in ordine alfabetico.

| File | Dimensione | Descrizione |
|---|---:|---|
| `walk.nnm` | 194 KB |  |
| `walk_it1800.nnm` | 194 KB |  |

## Training compatibile con ArduPilot

L'ambiente è già il deployment: fisica a 200 Hz come il loop dell'autopilota, gravità dal filtro IMU del firmware, azione tagliata a `NNM_ACT_MAX` e codificata in PWM, osservazione nell'ordine del profilo. Dettagli in [training.md](training.md).

```bash
.venv/bin/python tools/robots/nnm_env.py --robot phonebot_rev                       # carica la scena, passo a policy zero
.venv/bin/python tools/robots/train_velocity.py --robot phonebot_rev --envs 8 --iters 200 --name walk
.venv/bin/python tools/robots/train_velocity.py --robot phonebot_rev --eval robots/phonebot_rev/policies/walk.nnm
```

Il trainer CPU serve per verifiche e rifiniture brevi. Per una policy completa (2048 ambienti × 2000 iterazioni) si usa mjlab su GPU con gli stessi due pezzi: `deploy_contract.py` per osservazione e azione, `nnm_qat.enable_qat()` sull'actor, `export_nnm_from_actor()` per il file.

## Deploy sull'autopilota

```bash
python tools/robots/pack_robot_bin.py --robot phonebot_rev          # robot.bin dal profilo
# microSD: /APM/nnm/phonebot_rev/robot.bin  e  /APM/nnm/phonebot_rev/policies/*.nnm
```

Parametri: `NNM_ENABLE 1`, `NNM_ROBOT 19` (riavvio), `NNM_POLICY 0`, `SERVO1..13_FUNCTION 94..106`, `INS_GYRO_FILTER 0`, `SCHED_LOOP_RATE 200`.

## Note

- Variante fisica di PhoneBot per l'avanzamento: bacino e gambe girati di 180° sotto il busto, telefono rivolto nel nuovo verso di marcia. Baricentro circa 2,7 cm davanti al centro dei piedi invece di 1,4 cm dietro. Contratto, premi e firmware identici a PhoneBot.
- Geometria e assi dei giunti vengono dal GLB ufficiale; masse, inerzie e q0 sono stimati. Non usare una policy addestrata su questa scena sul robot prima di pesare i link e calibrare q0.
