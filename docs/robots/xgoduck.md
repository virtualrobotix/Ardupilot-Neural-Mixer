# XGO Duck (Arduino Uno Q)

[Catalogo robot](README.md) · [Training compatibile con ArduPilot](training.md) · [README](../../README.it.md)

| | |
|---|---|
| Id (cartella) | `xgoduck` |
| `NNM_ROBOT` | **21** |
| Classe | bipede |
| Progetto | XGO Duck, app XDuck per Arduino Uno Q basata su Microduck (Pollen Robotics) |
| Stato | policy upstream convertite in .nnm con parità numerica verificata; non simulate in questo repo |
| Giunti comandati | 14 |
| Osservazione | 61 valori |
| Frequenza policy | 50 Hz |
| Attuatori | 15 servo su bus seriale (protocollo SCS, 1 Mbps): 14 giunti della policy più la bocca (ID 34), pilotati dallo sketch XDuck sullo STM32 dell'Uno Q |
| Collegamento | servo su bus seriale dietro lo sketch XDuck dell'Uno Q: backend `AP_JointBus` (`NNM_OUT 1`) |
| Licenza upstream | Microduck: Apache-2.0. L'app XDuck e le sue policy ONNX non dichiarano una licenza: i `.nnm` derivati non sono versionati in questo repo |

## Repo originale e file di base

- Repository: [github.com/pollen-robotics/microduck](https://github.com/pollen-robotics/microduck)
- CAD e parti stampabili: non trovato
- BOM e montaggio: XGO Duck con 15 servo su bus seriale (ID 10–14, 20–24, 30–34) e IMU QMI8658 su Arduino Uno Q
- Training upstream: rsl_rl PPO su un fork di microduck_rl (percorsi nei metadati ONNX: xgoduck_velocity, xgoduck_roulade, xgoduck_sitstand, xgoduck_velocity_swizzle)
- Policy upstream: otto ONNX nell'app XDuck (`python/xgoduck_*.onnx`), convertiti con `tools/robots/import_xgoduck.py`

Per scaricare il repo originale e preparare la scena usata da simulazione e training:

```bash
.venv/bin/python tools/robots/fetch_upstream.py --robot xgoduck
```

## Topologia

File: [`robots/xgoduck/robot/profile.json`](../../robots/xgoduck/robot/profile.json) → `robot.bin` sulla microSD. Ordine dei giunti = ordine di osservazione, azione e uscite servo.

| # | Giunto | q0 (rad) | Uscita servo |
|---:|---|---:|---|
| 0 | `left_hip_yaw` | +0.0000 | `SERVO1_FUNCTION 94` |
| 1 | `left_hip_roll` | -0.0873 | `SERVO2_FUNCTION 95` |
| 2 | `left_hip_pitch` | -0.3840 | `SERVO3_FUNCTION 96` |
| 3 | `left_knee` | +0.0000 | `SERVO4_FUNCTION 97` |
| 4 | `left_ankle` | +0.3840 | `SERVO5_FUNCTION 98` |
| 5 | `neck_pitch` | +0.3491 | `SERVO6_FUNCTION 99` |
| 6 | `head_pitch` | +0.3491 | `SERVO7_FUNCTION 100` |
| 7 | `head_yaw` | +0.0000 | `SERVO8_FUNCTION 101` |
| 8 | `head_roll` | +0.0000 | `SERVO9_FUNCTION 102` |
| 9 | `right_hip_yaw` | +0.0000 | `SERVO10_FUNCTION 103` |
| 10 | `right_hip_roll` | +0.0873 | `SERVO11_FUNCTION 104` |
| 11 | `right_hip_pitch` | +0.3840 | `SERVO12_FUNCTION 105` |
| 12 | `right_knee` | +0.0000 | `SERVO13_FUNCTION 106` |
| 13 | `right_ankle` | -0.3840 | `SERVO14_FUNCTION 107` |

Osservazione (61): gyro FLU 3, gravità FLU 3, q−q0 14, q̇ 14, azione precedente 14, twist vx vy ωz 3, orologio 2 (peso zero nelle policy convertite) + comando testa 4 (collo, testa pitch, yaw, roll; via MAVLink `NNM_POSE`) + 4 canali non usati.
Azione (14): offset in radianti, `q_target = q0 + azione`.

## Architettura PPO

File: [`robots/xgoduck/robot/ppo.yaml`](../../robots/xgoduck/robot/ppo.yaml). Stessa rete di MicroDuck, già provata su Pixhawk 6C: **61 → 512 → 256 → 128 → 14**, ELU, normalizzazione dell'osservazione incorporata. Pesi int8 per riga addestrati sulla griglia int8 dalla prima iterazione (QAT), attivazioni float32. PPO: 2048 ambienti × 24 passi, 5 epoche, 4 minibatch, lr 1e-3 adattivo (KL 0,01), γ 0,99, λ 0,95, clip 0,2.

## Policy

Cartella: [`robots/xgoduck/policies/`](../../robots/xgoduck/policies/) → sulla microSD `/APM/nnm/xgoduck/policies/`. `NNM_POLICY` = indice del file in ordine alfabetico.

| File | Dimensione | Descrizione |
|---|---:|---|
| `0_walk.nnm` | 200 KB | camminata (`xgoduck_walk.onnx`). Comando: twist e testa. |
| `1_getup.nnm` | 200 KB | rialzata dopo una caduta (`xgoduck_getup.onnx`), `NNM_GETUP_IDX 1`. |
| `2_roller.nnm` | 200 KB | pattinata per il robot con i roller (`xgoduck_roller.onnx`): home propria, scala 0,8 incorporata nei pesi. |
| `3_pick.nnm` | 200 KB | raccolta, skill 1 (`xgoduck_pick.onnx`): 4 s, comando (cos, sin) della fase. |
| `4_roulade.nnm` | 200 KB | capriola, skill 2 (`xgoduck_roulade.onnx`): 1,7 s. |
| `5_sitstand.nnm` | 200 KB | seduto / in piedi, skill 3 e 4 (`xgoduck_sitstand.onnx`). |
| `6_kick_left.nnm` | 200 KB | calcio sinistro, skill 5 (`xgoduck_ballkick_left.onnx`): 0,4 s, scala 0,8 incorporata. |
| `7_kick_right.nnm` | 200 KB | calcio destro, skill 6 (`xgoduck_ballkick_right.onnx`): 0,4 s, scala 0,8 incorporata. |

## Training compatibile con ArduPilot

L'ambiente è già il deployment: fisica a 200 Hz come il loop dell'autopilota, gravità dal filtro IMU del firmware, azione tagliata a `NNM_ACT_MAX` e codificata in PWM, osservazione nell'ordine del profilo. Dettagli in [training.md](training.md).

```bash
.venv/bin/python tools/robots/nnm_env.py --robot xgoduck                       # carica la scena, passo a policy zero
.venv/bin/python tools/robots/train_velocity.py --robot xgoduck --envs 8 --iters 200 --name walk
.venv/bin/python tools/robots/train_velocity.py --robot xgoduck --eval robots/xgoduck/policies/walk.nnm
```

Il trainer CPU serve per verifiche e rifiniture brevi. Per una policy completa (2048 ambienti × 2000 iterazioni) si usa mjlab su GPU con gli stessi due pezzi: `deploy_contract.py` per osservazione e azione, `nnm_qat.enable_qat()` sull'actor, `export_nnm_from_actor()` per il file.

## Deploy sull'autopilota

```bash
python tools/robots/pack_robot_bin.py --robot xgoduck          # robot.bin dal profilo
# microSD: /APM/nnm/xgoduck/robot.bin  e  /APM/nnm/xgoduck/policies/*.nnm
```

Parametri: `NNM_ENABLE 1`, `NNM_ROBOT 21` (riavvio), `NNM_POLICY 0`, `SERVO1..14_FUNCTION 94..107`, `INS_GYRO_FILTER 0`, `SCHED_LOOP_RATE 200`.

## Note

- Robot reale con il backend bus del firmware: `AP_JointBus` parla i pacchetti dello sketch XDuck (15 servo e IMU sullo STM32 dell'Uno Q); ArduRover gira sul lato Linux della scheda (board `unoq`). App, build e deploy: [`unoq/`](../../unoq/).
- Le otto policy sono quelle dell'app XDuck, addestrate dai suoi autori per questo hardware: stessa rete, stessi 14 giunti e stessa osservazione a 61 di MicroDuck, quindi si convertono in `.nnm` senza riaddestrare. La conversione sposta il comando testa sui canali `NNM_POSE`, azzera i pesi dei due canali dell'orologio e incorpora la scala 0,8 di roller e calci.
- Home: 22° su anca e caviglia per tutte le policy tranne il roller, come nel deploy XDuck. I metadati ONNX di get-up e pick indicano 20°: `import_xgoduck.py --home metadata` usa quelli.
- `NNM_DEPLOY` sceglie la catena: 0 standard del firmware, 1 quella del deploy XDuck (filtro sui target per policy, smoothing di giroscopio e velocità, stima di gravità, recupero da caduta, heading hold).
- Skill: pick, roulade, sit, stand e i due calci partono con `MAV_CMD_USER_1` (param1 = id) solo da armato, in piedi e senza altre skill in corso. La tabella viaggia in `skills.bin` accanto a `robot.bin`.
- Non esiste qui un modello MuJoCo dell'XGO Duck: le policy non sono provate in simulazione. La verifica fatta è la parità numerica fra ONNX e `.nnm` sulle stesse osservazioni.
