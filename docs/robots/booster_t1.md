# Booster T1

[Catalogo robot](README.md) · [Training compatibile con ArduPilot](training.md) · [README](../../README.it.md)

<img src="img/booster_t1.jpg" alt="Booster T1" width="360">

*Booster T1, render del modello MuJoCo T1_locomotion in posa di stand. Foto: [github.com/BoosterRobotics/booster_gym](https://github.com/BoosterRobotics/booster_gym).*

| | |
|---|---|
| Id (cartella) | `booster_t1` |
| `NNM_ROBOT` | **12** |
| Classe | bipede |
| Progetto | Booster Robotics |
| Stato | policy upstream convertita in .nnm e verificata nell'ambiente a contratto; da rifinire o riaddestrare sul contratto |
| Giunti comandati | 12 |
| Osservazione | 47 valori |
| Frequenza policy | 50 Hz |
| Attuatori | attuatori Booster proprietari con doppio encoder (picco 130 N·m al ginocchio; limiti nel MJCF: anca 45/45/30, ginocchio 65, caviglia 24/15 N·m); controllo in coppia/velocità/posizione via SDK DDS/ROS 2 (`/low_state`, `/joint_ctrl`) dalla scheda motion |
| Collegamento | servo su bus seriale: serve il backend bus nel firmware (non ancora scritto) |
| Licenza upstream | Apache-2.0 (booster_gym, booster_assets, MJCF Menagerie) |

## Repo originale e file di base

- Repository: [github.com/BoosterRobotics/booster_gym](https://github.com/BoosterRobotics/booster_gym)
- Scena MuJoCo upstream: [`resources/T1/T1_locomotion.xml`](https://github.com/BoosterRobotics/booster_gym/blob/main/resources/T1/T1_locomotion.xml)
- CAD e parti stampabili: mesh STL in booster_gym/resources/T1/meshes e booster_assets (URDF 23/29 DoF, XML); modello Menagerie google-deepmind/mujoco_menagerie/booster_t1 (23 giunti, attuatori di posizione)
- BOM e montaggio: robot commerciale (prezzo su richiesta): attuatori proprietari con doppio encoder, picco 130 N·m al ginocchio, Jetson AGX Orin 32 GB + i7, IMU 9 assi, RGB-D, batteria 10,5 Ah (2 h di cammino)
- Training upstream: Booster Gym: Isaac Gym, PPO actor-critic asimmetrico (attore 47 → 256-128-128 → 12, ELU; critico +14 privilegiati → 256-256-128), 4096 env, orizzonte 24, 20 mini-epoche, lr 1e-5, γ 0,995; orologio del passo 1-2 Hz nell'osservazione; PD sul motore kp 200/50, kd 5/1; randomizzazione di massa/CoM, rigidezza/attrito giunti, ritardo comando 0-10 tick a 500 Hz, attrito/compliance terreno, calci e spinte, terreni trimesh; sim2sim MuJoCo e Webots, sim2real zero-shot con conversione serie-parallelo della caviglia (Jacobiano trasposto)
- Policy upstream: deploy/models/T1.pt (TorchScript): MLP 47 → 256-128-128 → 12 ELU, convertibile in .nnm a meno dell'ordine dell'osservazione e della fase del passo

Per scaricare il repo originale e preparare la scena usata da simulazione e training:

```bash
.venv/bin/python tools/robots/fetch_upstream.py --robot booster_t1
```

## Topologia

File: [`robots/booster_t1/robot/profile.json`](../../robots/booster_t1/robot/profile.json) → `robot.bin` sulla microSD. Ordine dei giunti = ordine di osservazione, azione e uscite servo.

| # | Giunto | q0 (rad) | Uscita servo |
|---:|---|---:|---|
| 0 | `Left_Hip_Pitch` | -0.2000 | `SERVO1_FUNCTION 94` |
| 1 | `Left_Hip_Roll` | +0.0000 | `SERVO2_FUNCTION 95` |
| 2 | `Left_Hip_Yaw` | +0.0000 | `SERVO3_FUNCTION 96` |
| 3 | `Left_Knee_Pitch` | +0.4000 | `SERVO4_FUNCTION 97` |
| 4 | `Left_Ankle_Pitch` | -0.2500 | `SERVO5_FUNCTION 98` |
| 5 | `Left_Ankle_Roll` | +0.0000 | `SERVO6_FUNCTION 99` |
| 6 | `Right_Hip_Pitch` | -0.2000 | `SERVO7_FUNCTION 100` |
| 7 | `Right_Hip_Roll` | +0.0000 | `SERVO8_FUNCTION 101` |
| 8 | `Right_Hip_Yaw` | +0.0000 | `SERVO9_FUNCTION 102` |
| 9 | `Right_Knee_Pitch` | +0.4000 | `SERVO10_FUNCTION 103` |
| 10 | `Right_Ankle_Pitch` | -0.2500 | `SERVO11_FUNCTION 104` |
| 11 | `Right_Ankle_Roll` | +0.0000 | `SERVO12_FUNCTION 105` |

Osservazione (47): gyro FLU 3, gravità FLU 3, q−q0 12, q̇ 12, azione precedente 12, twist vx vy ωz 3, orologio del passo sin, cos 2 (`NNM_CLOCK_HZ`, Booster Gym 1-2 Hz; 0 = stare fermi).
Azione (12): offset in radianti, `q_target = q0 + azione`.

## Architettura PPO

File: [`robots/booster_t1/robot/ppo.yaml`](../../robots/booster_t1/robot/ppo.yaml). Stessa rete di MicroDuck, già provata su Pixhawk 6C: **47 → 512 → 256 → 128 → 12**, ELU, normalizzazione dell'osservazione incorporata. Pesi int8 per riga addestrati sulla griglia int8 dalla prima iterazione (QAT), attivazioni float32. PPO: 2048 ambienti × 24 passi, 5 epoche, 4 minibatch, lr 1e-3 adattivo (KL 0,01), γ 0,99, λ 0,95, clip 0,2.

## Policy

Cartella: [`robots/booster_t1/policies/`](../../robots/booster_t1/policies/) → sulla microSD `/APM/nnm/booster_t1/policies/`. `NNM_POLICY` = indice del file in ordine alfabetico.

| File | Dimensione | Descrizione |
|---|---:|---|
| `walk_booster.nnm` | 66 KB | la policy pubblicata da Booster (deploy/models/T1.pt, TorchScript 47-256-128-128-12 ELU) convertita al contratto NNMixer da tools/robots/import_booster_t1.py: permutazione delle colonne del primo strato, fattore 0,1 sulle velocità dei giunti, int8 per riga (scarto massimo 0,06 rad su osservazioni casuali). Nel nostro ambiente (PD a 200 Hz, quantizzazione PWM, ritardo 0-1 tick, gravità AHRS): avanti 0,51 m/s a comando 0,5 e 0,73 a 0,8, laterale 0,21 a 0,3, indietro 0,36 a 0,4, rotazione 0,65 rad/s a 0,8; 30 s senza cadere. Richiede `NNM_ATT_SRC 1` e l'orologio a 1 Hz (`NNM_CLOCK_HZ 1`) mentre c'è un comando, zero da fermo. |

### Risultati per versione di ambiente ed epoca

Ogni link è la policy int8, quella che gira sull'autopilota, a quel checkpoint. La versione è la configurazione di reward e comandi di quel run (`robots/booster_t1/robot/ppo.yaml` ne tiene l'ultima). Un'iterazione di training esegue 5 epoche PPO.

| Versione ambiente | Iterazione | Epoche PPO | Video | Risultato |
|---|---:|---:|---|---|
| `walk_booster (policy upstream)` | — | — | [`booster_t1_walk_booster.mp4`](../media/booster_t1_walk_booster.mp4) | La policy Booster Gym convertita in .nnm int8 esegue nel nostro ambiente: in piedi, avanti 0,5 e 0,8 m/s (0,84 reali), rotazione 0,8 rad/s, laterale 0,3, indietro 0,4, in piedi. Nessuna caduta in 30 s. Con la gravità del filtro complementare del firmware la stessa policy cade a 0,5 m/s: l'AHRS è obbligatorio. |

## Training compatibile con ArduPilot

L'ambiente è già il deployment: fisica a 200 Hz come il loop dell'autopilota, gravità dal filtro IMU del firmware, azione tagliata a `NNM_ACT_MAX` e codificata in PWM, osservazione nell'ordine del profilo. Dettagli in [training.md](training.md).

```bash
.venv/bin/python tools/robots/nnm_env.py --robot booster_t1                       # carica la scena, passo a policy zero
.venv/bin/python tools/robots/train_velocity.py --robot booster_t1 --envs 8 --iters 200 --name walk
.venv/bin/python tools/robots/train_velocity.py --robot booster_t1 --eval robots/booster_t1/policies/walk.nnm
```

Il trainer CPU serve per verifiche e rifiniture brevi. Per una policy completa (2048 ambienti × 2000 iterazioni) si usa mjlab su GPU con gli stessi due pezzi: `deploy_contract.py` per osservazione e azione, `nnm_qat.enable_qat()` sull'actor, `export_nnm_from_actor()` per il file.

## Deploy sull'autopilota

```bash
python tools/robots/pack_robot_bin.py --robot booster_t1          # robot.bin dal profilo
# microSD: /APM/nnm/booster_t1/robot.bin  e  /APM/nnm/booster_t1/policies/*.nnm
```

Parametri: `NNM_ENABLE 1`, `NNM_ROBOT 12` (riavvio), `NNM_POLICY 0`, `SERVO1..12_FUNCTION 94..105`, `INS_GYRO_FILTER 0`, `SCHED_LOOP_RATE 200`.

## Note

- Gravità: con `gravity_source ap_imu_filter` (filtro complementare, τ 0,5 s) l'errore sul vettore gravità in cammino è 0,13 (≈7,6°) e la policy Booster cade a 0,5 m/s; con `ahrs` (EKF di ArduPilot, `NNM_ATT_SRC 1`, rumore 0,6°) segue il comando. I robot piccoli tolleravano il filtro perché ci sono stati addestrati; un umanoide di 30 kg no.
- La policy guida i 12 giunti delle gambe come in Booster Gym; braccia, busto e testa restano sulla posa di preparazione tenuti dai loro PD (nel MJCF T1_locomotion sono fusi nel tronco).
- Attuatori in coppia: l'ambiente chiude il PD a 200 Hz con i guadagni di Booster (kp 200 anche e ginocchia, 50 caviglie; kd 5 / 1) e MuJoCo taglia la coppia ai limiti del motore. Sul robot il PD gira nel driver del motore: la policy manda solo posizioni, come nel nostro contratto.
- Caviglia parallela: il modello è a catena seriale; Booster converte in SDK con Jacobiano trasposto. Sul firmware ArduPilot servirebbe la stessa conversione nel backend bus.
- L'osservazione 47 coincide nei contenuti con quella dell'attore Booster (comandi, orologio del passo, gravità, giroscopio, q, q̇, azione precedente): l'orologio è nei due canali extra (`NNM_CLOCK_HZ`), Booster lo campiona a 1-2 Hz per episodio e lo azzera negli episodi da fermo (10 %).
- Manca nell'ambiente il termine `feet_swing` di Booster (piede sollevato nella sua finestra di fase, sinistro a 0,25, destro a 0,75 del ciclo, larghezza 0,2): con l'orologio già in osservazione è il prossimo termine da aggiungere, anche per MicroDuck e Microban.
- Il robot vero si collega via SDK (DDS) e non via PWM: come per i bus Feetech/Dynamixel serve un backend nel firmware. 12 ≤ 16 funzioni, quindi SITL e HIL funzionano già.
