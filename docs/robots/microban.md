# Microban

[Catalogo robot](README.md) · [Training compatibile con ArduPilot](training.md) · [README](../../README.it.md)

<img src="img/microban.jpg" alt="Microban" width="360">

*Microban, robot montato. Foto: [github.com/Rhoban/microban](https://github.com/Rhoban/microban).*

| | |
|---|---|
| Id (cartella) | `microban` |
| `NNM_ROBOT` | **1** |
| Classe | bipede |
| Progetto | Rhoban |
| Stato | walk_md.nnm addestrata sul contratto ArduPilot; walk.nnm upstream da rifinire |
| Giunti comandati | 18 |
| Osservazione | 73 valori |
| Frequenza policy | 50 Hz |
| Attuatori | 19× Dynamixel XL330-M288-T (bus); la testa non è comandata dalla policy |
| Collegamento | servo su bus seriale: serve il backend bus nel firmware (non ancora scritto) |
| Licenza upstream | Apache-2.0 (software); repo hardware GPL-3.0 / CC BY-NC-SA 4.0 |

## Repo originale e file di base

- Repository: [github.com/Rhoban/mjlab_microban](https://github.com/Rhoban/mjlab_microban)
- Hardware: [github.com/Rhoban/microban](https://github.com/Rhoban/microban)
- Scena MuJoCo upstream: [`src/mjlab_microban/robot/microban/scene.xml`](https://github.com/Rhoban/mjlab_microban/blob/main/src/mjlab_microban/robot/microban/scene.xml)
- CAD e parti stampabili: [github.com/Rhoban/microban/tree/main/cad](https://github.com/Rhoban/microban/tree/main/cad) (stl/, step/) e Onshape [cad.onshape.com/documents/d424992a192a8ce34ffce163](https://cad.onshape.com/documents/d424992a192a8ce34ffce163)
- BOM e montaggio: [github.com/Rhoban/microban/tree/main/docs](https://github.com/Rhoban/microban/tree/main/docs) (bom.md, printing.md, assembly.md)
- Training upstream: mjlab 1.3.0 + rsl_rl PPO, modello attuatore BAM XL330 (kp_fw 125)
- Policy upstream: [github.com/Rhoban/microban/blob/main/src/agents/walk.onnx](https://github.com/Rhoban/microban/blob/main/src/agents/walk.onnx)

Per scaricare il repo originale e preparare la scena usata da simulazione e training:

```bash
.venv/bin/python tools/robots/fetch_upstream.py --robot microban
```

## Topologia

File: [`robots/microban/robot/profile.json`](../../robots/microban/robot/profile.json) → `robot.bin` sulla microSD. Ordine dei giunti = ordine di osservazione, azione e uscite servo.

| # | Giunto | q0 (rad) | Uscita servo |
|---:|---|---:|---|
| 0 | `right_shoulder_pitch` | +0.0000 | `SERVO1_FUNCTION 94` |
| 1 | `right_shoulder_roll` | -0.1745 | `SERVO2_FUNCTION 95` |
| 2 | `right_elbow` | -0.3491 | `SERVO3_FUNCTION 96` |
| 3 | `right_hip_yaw` | +0.0000 | `SERVO4_FUNCTION 97` |
| 4 | `right_hip_roll` | -0.0873 | `SERVO5_FUNCTION 98` |
| 5 | `right_hip_pitch` | -0.1745 | `SERVO6_FUNCTION 99` |
| 6 | `right_knee` | +0.0000 | `SERVO7_FUNCTION 100` |
| 7 | `right_ankle_pitch` | +0.0000 | `SERVO8_FUNCTION 101` |
| 8 | `right_ankle_roll` | +0.0873 | `SERVO9_FUNCTION 102` |
| 9 | `left_shoulder_pitch` | +0.0000 | `SERVO10_FUNCTION 103` |
| 10 | `left_shoulder_roll` | +0.1745 | `SERVO11_FUNCTION 104` |
| 11 | `left_elbow` | -0.3491 | `SERVO12_FUNCTION 105` |
| 12 | `left_hip_yaw` | +0.0000 | `SERVO13_FUNCTION 106` |
| 13 | `left_hip_roll` | +0.0873 | `SERVO14_FUNCTION 107` |
| 14 | `left_hip_pitch` | -0.1745 | `SERVO15_FUNCTION 108` |
| 15 | `left_knee` | +0.0000 | `SERVO16_FUNCTION 109` |
| 16 | `left_ankle_pitch` | +0.0000 | oltre Scripting16: serve il backend bus |
| 17 | `left_ankle_roll` | -0.0873 | oltre Scripting16: serve il backend bus |

Osservazione (73): gyro FLU 3, gravità FLU 3, q−q0 18, q̇ 18, azione precedente 18, twist vx vy ωz 3, orologio sin, cos 2 (`NNM_CLOCK_HZ`) + posa comandata 8 (braccia 6, piegamento, ondeggiamento; via MAVLink `NNM_POSE`).
Azione (18): offset in radianti, `q_target = q0 + azione`.

## Architettura PPO

File: [`robots/microban/robot/ppo.yaml`](../../robots/microban/robot/ppo.yaml). Stessa rete di MicroDuck, già provata su Pixhawk 6C: **73 → 512 → 256 → 128 → 18**, ELU, normalizzazione dell'osservazione incorporata. Pesi int8 per riga addestrati sulla griglia int8 dalla prima iterazione (QAT), attivazioni float32. PPO: 2048 ambienti × 24 passi, 5 epoche, 4 minibatch, lr 1e-3 adattivo (KL 0,01), γ 0,99, λ 0,95, clip 0,2.

## Policy

Cartella: [`robots/microban/policies/`](../../robots/microban/policies/) → sulla microSD `/APM/nnm/microban/policies/`. `NNM_POLICY` = indice del file in ordine alfabetico.

| File | Dimensione | Descrizione |
|---|---:|---|
| `dance.nnm` | 207 KB | balletto a corpo intero, 16 giunti su 18: braccia di dance_arms, molleggio sulle ginocchia due volte per ciclo (anca e caviglia coordinate, piede piatto) e ondeggiamento laterale del bacino di 2,4 cm (`NNM_CLOCK_HZ 0.625`). Da dance_arms, checkpoint 400: errore 0,00-0,05 rad su tutti i giunti della coreografia, nessuna caduta, resta sul posto. W&B mjlab_microban/o57v0gih. |
| `dance_arms.nnm` | 207 KB | balletto delle sole braccia: le due braccia pompano in alternanza, ciclo di 1,6 s (`NNM_CLOCK_HZ 0.625`). Da wave_right, checkpoint 200: errore 0,01-0,05 rad su sei giunti. W&B mjlab_microban/pa1otmey. |
| `pose_cmd.nnm` | 207 KB | teleoperazione: esegue una posa comandata in tempo reale via MAVLink (`DEBUG_FLOAT_ARRAY` nome `NNM_POSE`, 8 canali dopo l'orologio: 6 offset delle braccia, piegamento delle ginocchia 0-0,55 rad, ondeggiamento ±0,15 rad) tenendo l'equilibrio (vedi gesture_imitation.md). Addestrata da dance it0400 su pose casuali, comando zero e stream delle clip, con ritardo 0-3 tick e passa-basso 0,15 s come il firmware (`NNM_POSE_WD 500`, `NNM_POSE_TAU 0.15`, `NNM_CLOCK_HZ 0`). Checkpoint 1800 di 2000 (800 + 1200 dopo la correzione di `base_height`): errore medio sui 16 giunti su pose tenute 0,03 rad, saluto in streaming a 40 Hz 0,04 rad sulle braccia, nessuna caduta. Il selettore stand/walk del trainer non misura la posa: il file è scelto sul tracking. |
| `walk.nnm` | 207 KB | walk.onnx pubblicato da Rhoban (MLP 63-512-256-128-18, stesso contratto NNMixer) convertito in int8 per riga. Nell'ambiente a contratto: in piedi 10 s, avanti cade a 6,8 s, rotazione cade a 1 s. Con la gravità esatta del simulatore regge 10 s in avanti: la policy è stata addestrata senza il filtro IMU dell'autopilota. Da rifinire con --init-onnx prima dell'uso. |
| `walk_md.nnm` | 207 KB | addestrata da zero sul contratto ArduPilot in int8 (QAT): reward e curriculum del task MicroDuck più premi sul passo (appoggio singolo, piede sollevato, alternanza, simmetria), 512 env × 3000 iterazioni, checkpoint 2700 (punteggio 0,64). Sopravvivenza 100% in tutte le modalità; avanti/indietro ~0,16-0,19 m/s a comando 0,3; rotazione 0,68-0,87 rad/s a comando 0,8; laterale 0,04 m/s a comando 0,2; passo alternato e simmetrico (3-4 cm, 0,27-0,29 s per piede). W&B mjlab_microban/h5e7djou. Allargata a 65 ingressi con due colonne a zero (uscita identica) quando Microban ha ricevuto i canali dell'orologio. |
| `wave_right.nnm` | 207 KB | saluto con la destra per imitazione di una clip (vedi gesture_imitation.md): mano alta, la spalla porta la mano in fuori e la riporta, un'onda ogni 2 s (`NNM_CLOCK_HZ 0.5`). Rifinita da walk_md, checkpoint 300: errore medio sui tre giunti 0,01 rad, in piedi senza spostarsi. W&B mjlab_microban/fqp2z48h. |

### Risultati per versione di ambiente ed epoca

Ogni link è la policy int8, quella che gira sull'autopilota, a quel checkpoint. La versione è la configurazione di reward e comandi di quel run (`robots/microban/robot/ppo.yaml` ne tiene l'ultima). Un'iterazione di training esegue 5 epoche PPO.

| Versione ambiente | Iterazione | Epoche PPO | Video | Risultato |
|---|---:|---:|---|---|
| `pose_cmd (MAVLink)` | 1800 | 9000 | [`microban_pose_cmd_mavlink_wave.mp4`](../media/microban_pose_cmd_mavlink_wave.mp4) | Teleoperazione: `tools/mocap/mocap_gcs.py --source clip` manda il saluto come stream `NNM_POSE` a 40 Hz via UDP; `play_policy.py --pose-mavlink` lo riceve e la policy `pose_cmd.nnm` lo esegue in tempo reale (errore braccia 0,04 rad). In verde la posa comandata. |
| `pose_cmd (generalizzazione)` | 1800 | 9000 | [`microban_pose_cmd_generalization.mp4`](../media/microban_pose_cmd_generalization.mp4) | Movimenti mai visti in training, da landmark MediaPipe sintetici attraverso il retarget reale: jumping jack (err 0,07 rad), pugni alternati a 1 Hz (0,15: ritardo del passa-basso 0,15 s), squat con braccia avanti (0,05), inclinazioni laterali (0,06), combo (0,06). 22 s senza cadere. |
| `pose_cmd` | 1800 | 9000 | [`microban_pose_cmd_it1800.mp4`](../media/microban_pose_cmd_it1800.mp4) | Pose comandate da script (`pose_cmd.nnm`): braccia alzate, piegamento 0,45 rad, ondeggiamento ±0,12, posa completa. Errore medio 0,06 rad lungo la sequenza, transizioni incluse; 0,03 rad a regime. |
| `pose_cmd` | 400 | 2000 | [`microban_pose_cmd_it0400.mp4`](../media/microban_pose_cmd_it0400.mp4) | Stesso run alla 400 (prima della correzione di `base_height`): braccia seguite, ginocchia ancora piegate a comando zero, errore 0,13 rad. |
| `pose_cmd` | 100 | 500 | [`microban_pose_cmd_demo.mp4`](../media/microban_pose_cmd_demo.mp4) | Stessa sequenza alla 100: le braccia seguono grosso modo, le ginocchia restano piegate (prior del balletto), errore 0,27 rad. |
| `dance` | 400 | 2000 | [`microban_dance_it0400.mp4`](../media/microban_dance_it0400.mp4) | Balletto a corpo intero per imitazione di clip (`dance.nnm`). In verde il corpo di riferimento della clip in quell'istante. Braccia, molleggio e ondeggiamento seguono la clip; 8 s sul posto senza cadere. |
| `dance` | 100 | 500 | [`microban_dance_it0100.mp4`](../media/microban_dance_it0100.mp4) | Stesso run alla 100: le braccia seguono, le gambe fanno metà del molleggio e le caviglie restano ferme. Il tronco non oscilla ancora. |
| `dance_arms` | 200 | 1000 | [`microban_dance_arms_it0200.mp4`](../media/microban_dance_arms_it0200.mp4) | Balletto delle sole braccia (`dance_arms.nnm`), partito dal saluto: la clip è seguita entro 0,05 rad già alla 200. |
| `wave_right_clip` | 300 | 1500 | [`microban_wave_right_clip_it0300.mp4`](../media/microban_wave_right_clip_it0300.mp4) | Saluto per imitazione di clip (`wave_right.nnm`): mano alta, arco della spalla ogni 2 s, quattro onde in 8 s, errore 0,01 rad. |
| `wave_right_clip` | 100 | 500 | [`microban_wave_right_clip_it0100.mp4`](../media/microban_wave_right_clip_it0100.mp4) | Stesso run alla 100: braccio alzato, l'arco non è ancora seguito. |
| `wave_right_v5 (senza orologio)` | 100 | 500 | [`microban_wave_right_v5_it0100.mp4`](../media/microban_wave_right_v5_it0100.mp4) | Ultimo tentativo con reward scritto a mano (fascia + velocità + arco): un solo arco, poi fermo. La rete senza fase non trova l'oscillazione. |
| `wave_right_v4 (senza orologio)` | 100 | 500 | [`microban_wave_right_v4_it0100.mp4`](../media/microban_wave_right_v4_it0100.mp4) | Reward su velocità del gomito: 38 inversioni in 8 s, uno scuotimento, non un saluto. |
| `wave_right (senza orologio)` | 400 | 2000 | [`microban_wave_right_it0400.mp4`](../media/microban_wave_right_it0400.mp4) | Primo tentativo, bersaglio alternato per il gomito: sta in piedi e non muove il braccio. Il premio del gesto era già a zero a braccio abbassato. |
| `walk_md` | 3000 | 15000 | [`microban_walk_md_it3000.mp4`](../media/microban_walk_md_it3000.mp4) | Ambiente allineato a MicroDuck più premi sul passo. Avanti 5 s a 0,3 m/s, destra 3 s, 180° a sinistra, avanti 5 s: nessuna caduta in 16,7 s, 2,4 m percorsi. |
| `walk_md` | 0–3000 | 0–15000 | [`microban_md_evolution.mp4`](../media/microban_md_evolution.mp4) | Evoluzione dello stesso ambiente: stessi comandi sui checkpoint successivi. |
| `walk_gait_v1` | 900 | 4500 | [`microban_gait_best_it900.mp4`](../media/microban_gait_best_it900.mp4) | Miglior checkpoint del run sul passo. Sta in piedi e ruota sul posto; l'avanzamento resta sotto 0,1 m. |
| `walk_gait_v1` | 0–1400 | 0–7000 | [`microban_evolution.mp4`](../media/microban_evolution.mp4) | 14 clip, avanti 4 s poi rotazione. Entro 100 iterazioni non cade; dalla 500 ruota; in avanti si sposta di pochi centimetri. |
| `walk_ap` | 200 | 1000 | [`microban_sequence_it200.mp4`](../media/microban_sequence_it200.mp4) | Rifinitura della policy upstream. Resta in piedi; la velocità comandata non è ancora seguita. |

### Risultato dopo 3000 iterazioni (`walk_md.nnm`)

<a href="../media/microban_walk_md_it3000.mp4"><img src="img/microban_walk_md.gif" alt="Risultato dopo 3000 iterazioni (walk_md.nnm)" width="480"></a>

*Ambiente allineato a MicroDuck più premi sul passo. Avanti 5 s a 0,3 m/s, destra 3 s, 180° a sinistra, avanti 5 s: nessuna caduta in 16,7 s, 2,4 m percorsi. Video: [`docs/media/microban_walk_md_it3000.mp4`](../media/microban_walk_md_it3000.mp4).*

## Training compatibile con ArduPilot

L'ambiente è già il deployment: fisica a 200 Hz come il loop dell'autopilota, gravità dal filtro IMU del firmware, azione tagliata a `NNM_ACT_MAX` e codificata in PWM, osservazione nell'ordine del profilo. Dettagli in [training.md](training.md).

```bash
.venv/bin/python tools/robots/nnm_env.py --robot microban                       # carica la scena, passo a policy zero
.venv/bin/python tools/robots/train_velocity.py --robot microban --envs 8 --iters 200 --name walk
.venv/bin/python tools/robots/train_velocity.py --robot microban --init-onnx robots/microban/policies/walk.onnx --name walk_ap   # rifinitura (ONNX scaricato da fetch_upstream.py)
.venv/bin/python tools/robots/train_velocity.py --robot microban --eval robots/microban/policies/walk.nnm
```

Il trainer CPU serve per verifiche e rifiniture brevi. Per una policy completa (2048 ambienti × 2000 iterazioni) si usa mjlab su GPU con gli stessi due pezzi: `deploy_contract.py` per osservazione e azione, `nnm_qat.enable_qat()` sull'actor, `export_nnm_from_actor()` per il file.

## Deploy sull'autopilota

```bash
python tools/robots/pack_robot_bin.py --robot microban          # robot.bin dal profilo
# microSD: /APM/nnm/microban/robot.bin  e  /APM/nnm/microban/policies/*.nnm
```

Con 18 giunti il firmware carica `robot.bin` e le policy (fino a 20 giunti, `NNM_MAX_JOINTS`) e gira in SITL, ma le uscite PWM coprono al massimo 16 funzioni servo Scripting: il deploy sul robot richiede il backend bus. Simulazione, training e file `.nnm` sono già pronti.

## Note

- L'osservazione dell'ONNX è gyro, gravità proiettata, q−q0, q̇, azione precedente, twist: il formato NNMixer con 18 giunti, quindi la policy pubblicata si converte senza riaddestrarla.
- 18 giunti superano le 16 funzioni servo Scripting consecutive: il firmware carica la topologia (`NNM_MAX_JOINTS` 20) e gira in SITL, ma sul robot servono le uscite del backend bus Dynamixel. Simulazione e training funzionano già.
- L'osservazione è 73: dopo il twist ci sono seno e coseno dell'orologio dei gesti (`NNM_CLOCK_HZ`, 0 per la camminata) e otto canali di posa comandata via MAVLink (`NNM_POSE`; a zero = riposo, quindi walk e clip non ne risentono). Saluto, balletto e teleoperazione sono selezionabili con `NNM_POLICY`: vedi [gesture_imitation.md](gesture_imitation.md).
