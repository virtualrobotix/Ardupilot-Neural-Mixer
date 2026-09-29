# Petoi Bittle (OpenCat)

[Catalogo robot](README.md) · [Training compatibile con ArduPilot](training.md) · [README](../../README.it.md)

<img src="img/bittle.jpg" alt="Petoi Bittle (OpenCat)" width="360">

*Petoi Bittle (OpenCat), fotogramma del trotto del modello MuJoCo (demo.gif di bittle-mujoco, Apache-2.0). Foto: [github.com/MarcHesse/bittle-mujoco](https://github.com/MarcHesse/bittle-mujoco).*

| | |
|---|---|
| Id (cartella) | `bittle` |
| `NNM_ROBOT` | **11** |
| Classe | quadrupede |
| Progetto | Petoi (kit Bittle / Bittle X) |
| Stato | scena MuJoCo nativa pronta; policy da addestrare |
| Giunti comandati | 8 |
| Osservazione | 33 valori |
| Frequenza policy | 50 Hz |
| Attuatori | 8× Petoi P1S (alloy) o P1L (plastic) sulle zampe, 270° di corsa, stallo 3,15 kg·cm a 8,4 V, coreless; 1× sul collo (non comandato dalla policy). Upstream: NyBoard (ATmega328P) o BiBoard (ESP32), IMU MPU6050 |
| Collegamento | servo PWM: collegabili alle uscite dell'autopilota |
| Licenza upstream | MIT (OpenCat, meshes ros_opencat); Apache-2.0 (MJCF bittle-mujoco) |

## Repo originale e file di base

- Repository: [github.com/PetoiCamp/OpenCat-Quadruped-Robot](https://github.com/PetoiCamp/OpenCat-Quadruped-Robot)
- Modello MuJoCo e training: [github.com/MarcHesse/bittle-mujoco](https://github.com/MarcHesse/bittle-mujoco)
- Scena MuJoCo upstream: [`bittle.xml`](https://github.com/MarcHesse/bittle-mujoco/blob/main/bittle.xml)
- CAD e parti stampabili: STL/OBJ delle parti in PetoiCamp/ros_opencat (mesh del modello); guscio stampabile del successore Quaddle annunciato
- BOM e montaggio: kit Petoi (Bittle STEM/Robotics, Bittle X): 9 servo P1S/P1L, NyBoard o BiBoard ESP32, batteria Li-ion 7,4 V 1000 mAh, IMU MPU6050/ICM42670
- Training upstream: nessuno upstream: OpenCat esegue tabelle di passo (src/InstinctBittle.h) con bilanciamento PID sull'IMU. RL della comunità: OpenCat Gym (PyBullet+SB3), Bittle_Symmetry_RL (Isaac Gym, simmetrie nel reward), BittleHRL (Isaac Lab, CPG+RL), MH-FLOCKE (MuJoCo, rete spiking con CPG, sim-to-real su Bittle X)
- Policy upstream: nessuna nel formato NNMixer

Per scaricare il repo originale e preparare la scena usata da simulazione e training:

```bash
.venv/bin/python tools/robots/fetch_upstream.py --robot bittle
```

## Topologia

File: [`robots/bittle/robot/profile.json`](../../robots/bittle/robot/profile.json) → `robot.bin` sulla microSD. Ordine dei giunti = ordine di osservazione, azione e uscite servo.

| # | Giunto | q0 (rad) | Uscita servo |
|---:|---|---:|---|
| 0 | `FR_shoulder` | -0.7854 | `SERVO1_FUNCTION 94` |
| 1 | `FR_knee` | +1.4835 | `SERVO2_FUNCTION 95` |
| 2 | `FL_shoulder` | -0.7854 | `SERVO3_FUNCTION 96` |
| 3 | `FL_knee` | +1.4835 | `SERVO4_FUNCTION 97` |
| 4 | `RR_shoulder` | +0.7854 | `SERVO5_FUNCTION 98` |
| 5 | `RR_knee` | +1.4835 | `SERVO6_FUNCTION 99` |
| 6 | `RL_shoulder` | +0.7854 | `SERVO7_FUNCTION 100` |
| 7 | `RL_knee` | +1.4835 | `SERVO8_FUNCTION 101` |

Osservazione (33): gyro FLU 3, gravità FLU 3, q−q0 8, q̇ 8, azione precedente 8, twist vx vy ωz 3.
Azione (8): offset in radianti, `q_target = q0 + azione`.

## Modello meccanico e confronto con Freenove Robot Dog

| Grandezza | Petoi Bittle (OpenCat) | Freenove Robot Dog | Fonte |
|---|---|---|---|
| Gradi di libertà per zampa | 2: spalla (pitch), ginocchio | 3: abduzione, anca, ginocchio | MJCF bittle-mujoco |
| Giunti comandati | 8 (+1 collo) | 12 | OpenCat: PWM 8-15 zampe, 0 collo |
| Interasse spalle (x × y) | 119 × 72 mm | 136 × 76 mm | posizioni `servo_*s` nel MJCF |
| Coscia / tibia | 46 / 29 mm (al punto di contatto) | 55 / 55 mm | mj_kinematics in stand |
| Altezza del tronco in stand | 47 mm (spalle a 69 mm) | 101 mm | settle sul modello |
| Massa | 273,5 g misurati (batteria 55 g) | ≈ 550 g stimati | README bittle-mujoco |
| Servo | P1S 270°, 3,15 kg·cm (≈ 0,27 N·m a 7,4 V) | ES08MA II 0,16–0,20 N·m | specifiche Petoi |
| Attuatore nel MJCF | posizione kp 40, coppia ±0,27 N·m (scene.xml) | kp 2, ±0,17 N·m | bittle-mujoco; limite aggiunto da fetch_upstream |
| Contatti | Newton, coni ellittici, impratio 100 (meno slittamento dei piedi) | default MuJoCo | CHANGELOG bittle-mujoco 2026-09-19 |

## Architettura PPO

File: [`robots/bittle/robot/ppo.yaml`](../../robots/bittle/robot/ppo.yaml). Stessa rete di MicroDuck, già provata su Pixhawk 6C: **33 → 512 → 256 → 128 → 8**, ELU, normalizzazione dell'osservazione incorporata. Pesi int8 per riga addestrati sulla griglia int8 dalla prima iterazione (QAT), attivazioni float32. PPO: 2048 ambienti × 24 passi, 5 epoche, 4 minibatch, lr 1e-3 adattivo (KL 0,01), γ 0,99, λ 0,95, clip 0,2.

## Policy

Cartella: [`robots/bittle/policies/`](../../robots/bittle/policies/) → sulla microSD `/APM/nnm/bittle/policies/`. `NNM_POLICY` = indice del file in ordine alfabetico.

Nessuna policy ancora: va addestrata (sezione successiva).

### Risultati per versione di ambiente ed epoca

Ogni link è la policy int8, quella che gira sull'autopilota, a quel checkpoint. La versione è la configurazione di reward e comandi di quel run (`robots/bittle/robot/ppo.yaml` ne tiene l'ultima). Un'iterazione di training esegue 5 epoche PPO.

| Versione ambiente | Iterazione | Epoche PPO | Video | Risultato |
|---|---:|---:|---|---|
| `modello (passo OpenCat open-loop)` | — | — | [`bittle_model_trot.mp4`](../media/bittle_model_trot.mp4) | Verifica del modello: la tabella `trF` di OpenCat (48 frame a 50 Hz) eseguita open-loop attraverso lo stesso percorso servo dell'ambiente (kp 40, coppia 0,27 N·m, damping 0,03): trotto a 0,086 m/s, tronco a 37 mm, errore di inseguimento 0,02 rad, nessuna caduta in 8 s. Poi `vtL` (pivot a sinistra, 0,28 rad/s) e `bkF` (indietro, 0,05 m/s). |

## Training compatibile con ArduPilot

L'ambiente è già il deployment: fisica a 200 Hz come il loop dell'autopilota, gravità dal filtro IMU del firmware, azione tagliata a `NNM_ACT_MAX` e codificata in PWM, osservazione nell'ordine del profilo. Dettagli in [training.md](training.md).

```bash
.venv/bin/python tools/robots/nnm_env.py --robot bittle                       # carica la scena, passo a policy zero
.venv/bin/python tools/robots/train_velocity.py --robot bittle --envs 8 --iters 200 --name walk
.venv/bin/python tools/robots/train_velocity.py --robot bittle --eval robots/bittle/policies/walk.nnm
```

Il trainer CPU serve per verifiche e rifiniture brevi. Per una policy completa (2048 ambienti × 2000 iterazioni) si usa mjlab su GPU con gli stessi due pezzi: `deploy_contract.py` per osservazione e azione, `nnm_qat.enable_qat()` sull'actor, `export_nnm_from_actor()` per il file.

## Deploy sull'autopilota

```bash
python tools/robots/pack_robot_bin.py --robot bittle          # robot.bin dal profilo
# microSD: /APM/nnm/bittle/robot.bin  e  /APM/nnm/bittle/policies/*.nnm
```

Parametri: `NNM_ENABLE 1`, `NNM_ROBOT 11` (riavvio), `NNM_POLICY 0`, `SERVO1..8_FUNCTION 94..101`, `INS_GYRO_FILTER 0`, `SCHED_LOOP_RATE 200`.

## Note

- Lo zero dei giunti è la posa `balance` di OpenCat (30° su tutti i servo delle zampe). Conversione dalle tabelle OpenCat: rad = (gradi − riposo) × segno × π/180 con riposo 75° (coscia) e −55° (tibia), segno −1 sulle spalle posteriori (gaits.py di bittle-mujoco).
- Passi di riferimento: le tabelle di OpenCat (trotto, camminata, crawl, bound, galoppo, salto) sono già in angoli MuJoCo in third_party/bittle_model/gaits.py: utilizzabili come clip di imitazione con l'orologio `NNM_CLOCK_HZ`, come per i gesti di Microban. Riprodotte open-loop sul modello dell'ambiente (rampa 2 s, poi 8 s): trotto `trF` 0,086 m/s, camminata `wkF` 0,031 m/s, indietro `bkF` 0,050 m/s, trotto a sinistra `trL` 0,055 m/s e 3°/s, pivot `vtL` 16°/s; galoppo e bound cadono, il crawl si sdraia. I limiti di postura dell'ambiente (tronco > 28 mm, spalle entro 1,0 rad da q0) sono tarati perché questi passi non chiudano l'episodio.
- Zampe planari: nessuna velocità laterale comandabile; il compito è avanti/indietro e rotazione.
- Gli 8 servo P1S sono PWM e si collegano alle uscite dell'autopilota al posto della NyBoard/BiBoard (8 ≤ 16). Scala da definire in robot.bin: 270° su 500–2500 µs ≈ 2,36 mrad/µs contro i 3 mrad/µs del filo NNMixer, più l'offset di calibrazione OpenCat per servo.
- Il modello ha coppia limitata a 0,27 N·m e kp 40 dell'upstream; i giunti hanno damping 1,5 e frictionloss 0,15 tarati sul trotto open-loop. Le masse sono misurate, non stimate.
