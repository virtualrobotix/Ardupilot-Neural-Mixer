# Training in MuJoCo compatibile con il deployment ArduPilot

[Catalogo robot](README.md) · [README](../../README.it.md)

L'obiettivo è che la rete addestrata in MuJoCo veda, durante il training, gli stessi numeri che le darà
AP_NNMixer sull'autopilota, e che il file prodotto dal training sia esattamente la rete che gira sulla
scheda. Due pezzi rendono questo vero per ogni robot del catalogo:

- **il contratto di deployment** ([`tools/robots/deploy_contract.py`](../../tools/robots/deploy_contract.py)):
  osservazione, gravità, azione e frequenza calcolate come nel firmware;
- **il training in int8** ([`tools/robots/nnm_qat.py`](../../tools/robots/nnm_qat.py)): l'actor impara
  direttamente sulla griglia int8 che il firmware esegue, senza una quantizzazione dopo il training.

L'ambiente [`tools/robots/nnm_env.py`](../../tools/robots/nnm_env.py) usa entrambi e legge tutto il resto
dal profilo del robot.

## Cosa replica l'ambiente

| Aspetto | Sull'autopilota (AP_NNMixer) | Nell'ambiente di training |
|---|---|---|
| Frequenza | task a 50 Hz, loop a 200 Hz | un `step()` = un tick della policy; fisica a 200 Hz dentro il passo |
| Robot a meno di 50 Hz | la policy gira ogni `50 / rate_hz` tick (25, 10, 5 Hz) | `rate_hz` del profilo deve dividere 50 |
| Gyro | IMU grezzo in FLU, `INS_GYRO_FILTER 0` | sensore gyro sul tronco, FRD → FLU, rumore |
| Gravità | filtro complementare IMU (`NNM_ATT_TAU 0,5`) al loop rate | lo stesso filtro, alimentato da gyro e accelerometro simulati a 200 Hz |
| Osservazione | gyro, gravità, q−q0, q̇, azione precedente, twist, comandi extra a zero | stesso ordine e stessa lunghezza, dal profilo (niente riempimento a 61) |
| Azione | taglio a `NNM_ACT_MAX`, `q_target = q0 + a` | stesso taglio; l'azione precedente è quella tagliata |
| Uscita | PWM 1500 + q/0,003, limiti 800..2200 µs | target giunto quantizzato a passi di 3 mrad con gli stessi limiti |
| Comandi | stick in MANUAL, zero in HOLD, velocità del modo Rover in GUIDED/AUTO/RTL (vy = 0) | 20% comandi a zero, 30% senza componente laterale, intervalli di `NNM_VX/VY/WZ_MAX` |
| Avvio | disarmato: robot fermo, filtro che converge; all'arm la policy parte in piedi | reset con tronco tenuto fermo 0,5 s e IMU statica, poi la policy parte |
| Ritardo | un tick di latenza possibile sul gyro | ritardo casuale 0–1 tick sull'osservazione |
| Rete | `nnmixer_forward_int8`: pesi int8 per riga, bias e attivazioni float32 | actor con `QATLinear`: stessa quantizzazione nel forward |

Perché conta: la policy di Microban pubblicata da Rhoban, convertita in int8 e provata in questo
ambiente, sta in piedi con la gravità esatta del simulatore e cade in meno di un secondo con la gravità
stimata dal filtro del firmware. È stata addestrata senza il filtro. MicroDuck lo tollera, Microban no:
l'unico modo di saperlo prima di volare è addestrare e valutare con il contratto.

## Perché int8 già in training

Il firmware tiene due policy in RAM, in int8 (circa 200 KB l'una). Quantizzare dopo il training introduce
un errore che la policy non ha mai visto: su MicroDuck l'azione int8 si scosta dalla float32 fino a 0,06 rad
su osservazioni casuali, su Microban fino a 0,13 rad.

Con il training in int8 (quantization-aware training) il forward dell'actor usa già i pesi arrotondati
alla griglia int8 per riga; il gradiente passa attraverso l'arrotondamento come se fosse l'identità
(straight-through estimator). La policy impara sulla rete che verrà eseguita, e l'export copia i valori
int8 così come sono. Il test `tests/test_nnm_pipeline.py` verifica che la rete in training, il file `.nnm`
e il forward C del firmware diano la stessa azione (differenza sotto 1e-4; nei run reali ~1e-6).

Solo i pesi sono int8. Attivazioni e bias restano float32, come in `nnmixer_forward_int8`.

## Architettura PPO

Ogni robot ha `robots/<id>/robot/ppo.yaml`. La rete è la stessa di MicroDuck, già provata su Pixhawk 6C:

```
obs_dim  ->  512  ->  256  ->  128  ->  n_joints      ELU, normalizzazione dell'osservazione incorporata
```

`obs_dim = 3 + 3 + 3·n_joints + 3 (+ comandi extra)`. Il critic ha la stessa forma e non va sulla scheda.
PPO come per MicroDuck: 2048 ambienti × 24 passi per iterazione, 5 epoche, 4 minibatch, lr 1e-3 adattivo
(KL 0,01), γ 0,99, λ 0,95, clip 0,2, entropia 0,01.

## Comandi

```bash
# 1. repo originale e scena MuJoCo
.venv/bin/python tools/robots/fetch_upstream.py --robot rex
.venv/bin/python tools/robots/nnm_env.py --robot rex                 # passo a policy zero

# 2. training int8 sul contratto (CPU: verifiche e robot piccoli)
.venv/bin/python tools/robots/train_velocity.py --robot rex --envs 8 --iters 200 --name walk

# 2b. rifinitura di una policy upstream compatibile (stessa rete)
.venv/bin/python tools/robots/train_velocity.py --robot microban \
    --init-onnx robots/microban/policies/walk.onnx --name walk_ap

# 3. valutazione del file che andrà sulla scheda
.venv/bin/python tools/robots/train_velocity.py --robot rex --eval robots/rex/policies/walk.nnm

# 4. file per la microSD
.venv/bin/python tools/robots/pack_robot_bin.py --robot rex         # robots/rex/robot/robot.bin
```

Il trainer CPU fa circa 700 passi di policy al secondo con 4 ambienti: serve per controllare la pipeline e
per rifiniture brevi. La policy di MicroDuck ha richiesto 98 milioni di passi (2048 ambienti × 2000
iterazioni, ~55 minuti su una RTX 3090 con MuJoCo Warp).

## Training su GPU con mjlab

Per una policy completa si usa mjlab + rsl_rl su GPU, con gli stessi pezzi:

1. **osservazione**: nei termini di osservazione del task, gravità dal filtro di `deploy_contract.GravityFilter`
   (versione vettoriale sugli ambienti) invece di `projected_gravity` dal quaternione; stesso ordine del
   profilo; comandi extra a zero se il robot non li riceve dall'autopilota;
2. **azione**: `deploy_contract.apply_action` (taglio, azione precedente dopo il taglio, quantizzazione PWM);
3. **frequenza**: `decimation = 200 / rate_hz · (dt 0,005)`, cioè 4 a 50 Hz, 8 a 25 Hz;
4. **rete**: `nnm_qat.enable_qat(runner.alg.policy.actor)` prima del training;
5. **export**: `nnm_qat.export_nnm_from_actor(actor, mean, std, q0, robot_id, path)`, poi
   `train_velocity.py --eval` sul file prodotto.

L'export ONNX di rsl_rl resta utile per confronto, ma il file per la scheda è il `.nnm`.

## Imitazione di una clip (MimicKit)

Il riferimento per integrare l'imitazione di un movimento nel training da rinforzo è
[MimicKit](https://arxiv.org/abs/2510.13794) (Peng, arXiv:2510.13794, 2025; codice
[xbpeng/MimicKit](https://github.com/xbpeng/MimicKit)). È un framework unico: lo stesso ciclo di RL,
cambiando algoritmo, rete, compito e simulatore. Questo repo non esegue quel codice. Ne riprende la
divisione dei ruoli e il metodo di tracking, dentro il contratto ArduPilot e l'export int8.

MimicKit separa quattro pezzi (sezione 3 del paper). Qui stanno già nel trainer di velocità:

| In MimicKit | Qui |
|---|---|
| Agent: algoritmo e buffer di esperienza. Il default è PPO; AWR è l'alternativa off-policy | [`train_velocity.py`](../../tools/robots/train_velocity.py), PPO on-policy come sopra |
| Model: actor, critic, e per AMP/ADD un discriminatore | MLP `obs → 512 → 256 → 128 → n_joints`. Sulla scheda va solo l'actor, in int8 |
| Environment: osservazione, reward, flag di fine episodio, azione trasformata in comando | [`nnm_env.py`](../../tools/robots/nnm_env.py) |
| Engine: fisica e modo di controllo (`pos`, `vel`, `torque`, `pd_1d`) su Isaac Gym, Isaac Lab o Newton | MuJoCo. Il comando è `q_target = q0 + a`, tagliato e quantizzato a passi di 3 mrad come il PWM |

Tra i metodi della sezione 4, quello usato per i gesti e per il rialzarsi è **DeepMimic** (sezione 4.1;
Peng, Abbeel, Levine, van de Panne, ACM TOG 2018): un controllore che insegue una clip di riferimento.
Il paper lo indica come punto di partenza quando serve replicare quel movimento con precisione, prima
di passare a metodi che imitano lo stile di un dataset intero (AMP, ASE) o che imparano da soli il
premio di tracking (ADD). La limitazione dichiarata è la stessa che accettiamo: la policy resta legata
alla clip e non la adatta a un compito nuovo. Per camminare su un twist si continua a usare il PPO di
velocità; DeepMimic è una seconda policy.

| Pezzo del tracking | Nel training NNMixer |
|---|---|
| `motion_file`: una clip, oppure un dataset di clip (sezione 6) | Pose chiave interpolate con un coseno, oppure `robots/<id>/motions/*.npz` (giunti, fase, quaternione e quota del tronco). Il formato MimicKit è un `.pkl` con posizione e rotazione del root (mappa esponenziale) e rotazioni dei giunti nell'ordine dell'albero cinematico |
| Fase della clip nell'osservazione | Seno e coseno dopo il twist. Clip periodica: `NNM_CLOCK_HZ`. Clip una volta sola (get-up): θ = π·min(t / `NNM_GETUP_S`, 1), che finisce a (0, −1) |
| Premio di posa sulla clip a quella fase | `gesture_track`, exp sull'errore quadratico medio dei giunti. Accanto, `gesture` lineare in \|q̂ − q\|, perché l'esponenziale è già ~0 a 1 rad e da solo non muove un braccio abbassato |
| Tracking del tronco | Sui gesti corti in piedi le gambe fuori clip restano su q0, con i premi di equilibrio della walk. Sul get-up la clip aggiunge gravità del tronco e quota del bacino (`getup_root_g`, `getup_root_z`) |
| Partenza lungo la clip | Sul get-up di Fase II l'episodio parte a una fase casuale (metà delle partenze nell'ultimo tratto, la fusione verso q0) |
| Fine episodio per errore di posa | Nel paper è la configurazione di default dei metodi di tracking e rende i run più stabili (sezione 7.1; la Tabella 1 è misurata senza, per confrontarli con AMP). Qui l'episodio dei gesti non si chiude sull'errore di posa; si chiude su una caduta o sul tempo |

Sullo stesso benchmark, il get-up prono dell'umanoide (clip «Getup Facedown», 3,03 s) ha un errore di
posizione di 0,023 m con DeepMimic (Tabella 1). Sul Microban da 350 g il replay in anello aperto della
clip non sta in piedi sui servo cedevoli, quindi la Fase II di tracking parte da una clip che una policy
di scoperta ha già reso fattibile: forza d'aiuto a curriculum, registrazione, rallentamento e fusione
verso q0, poi imitazione in anello chiuso con `walk.nnm` come maestro della tenuta finale.

Due usi, stesso contratto. Ricetta, reward e risultati: [gesture_imitation.md](gesture_imitation.md).

- **Gesti periodici** (saluto, balletto): clip corta, robot in piedi, orologio che gira.
- **Abilità una volta sola** (rialzarsi): `getup.nnm`, orologio one-shot.

Il firmware riempie i due canali dell'orologio e, per il get-up, commuta da solo sulla policy one-shot
(`NNM_GETUP_*`) e torna alla policy precedente a fine clip. La rete sulla scheda resta un forward int8
per tick.
