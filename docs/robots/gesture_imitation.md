# Gesti per imitazione di clip: saluto e balletto

[Catalogo robot](README.md) · [Training compatibile con ArduPilot](training.md) · [Microban](microban.md)

Oltre alla camminata, un robot del catalogo può imparare un **gesto a tempo**: un saluto, un balletto,
una posa. Questa pagina spiega come funziona l'algoritmo, perché il primo approccio non funzionava, e come
si aggiunge un gesto nuovo. Il banco di prova è Microban (braccia a 3 gradi di libertà, gambe a 6).

Risultati: [`wave_right.nnm`](../../robots/microban/policies/wave_right.nnm) saluta con la destra
([video](../media/microban_wave_right_clip_it0300.mp4)), [`dance.nnm`](../../robots/microban/policies/dance.nnm)
balla con braccia e gambe ([video](../media/microban_dance_it0400.mp4)). In verde, nei video, il corpo di
riferimento della clip in quell'istante.

## L'idea in breve

La camminata insegue una **velocità**: la rete riceve `(vx, vy, wz)` e viene premiata se il tronco si muove
a quella velocità. Un gesto insegue invece una **posa che cambia nel tempo**. Il metodo è il tracking
DeepMimic dentro [MimicKit](https://arxiv.org/abs/2510.13794) (Peng, 2025): lo stesso PPO del training di
velocità, con una fase nell'osservazione e un premio sulla posa della clip. Come entra nel processo di
addestramento è in [training.md](training.md#imitazione-di-una-clip-mimickit).
Qui il dettaglio operativo sui giunti di Microban:

1. **Una clip di riferimento**: poche pose chiave dei giunti su un periodo, interpolate con un coseno. Per il
   saluto sono tre numeri per la spalla e il gomito; per il balletto sedici giunti su diciotto.
2. **Un orologio nell'osservazione**: seno e coseno di una fase che avanza a frequenza fissa. Sono i due
   canali extra dopo il twist, gli stessi che MicroDuck usa per la posa della testa. Il firmware li genera
   con il parametro `NNM_CLOCK_HZ`.
3. **Un premio denso**: a ogni passo, quanto i giunti della clip sono vicini alla posa di *questa* fase. Un
   termine lineare nell'errore, che dà gradiente anche a un braccio abbassato, e un termine `exp(-err²)` più
   stretto che paga il seguire la clip da vicino.

La rete resta la stessa MLP a un solo forward per tick che il firmware esegue oggi: cambia solo cosa vede
(due numeri in più) e cosa viene premiato. Le gambe non nominate nella clip restano sulla posa in piedi, con
i termini di equilibrio della camminata.

```mermaid
flowchart LR
  clock["NNM_CLOCK_HZ: fase"] --> obs["osservazione 73: IMU, giunti, azione, twist, sin, cos, posa×8"]
  obs --> mlp["MLP int8 (la stessa della walk)"]
  mlp --> servo["q0 + azione sui 18 servo"]
  clip["clip: pose chiave interpolate"] --> ref["posa di riferimento della fase"]
  ref --> reward["premio: vicinanza alla posa"]
  servo --> reward
```

## Perché il metodo precedente non funzionava

Il primo saluto era descritto con un reward scritto a mano, senza orologio: "porta il gomito a −1,35, poi
a −0,35, poi di nuovo", e successivamente "resta in una fascia e muoviti a 1 rad/s". I tre tentativi sono
nei video della [pagina Microban](microban.md#risultati-per-versione-di-ambiente-ed-epoca), e falliscono
per la stessa ragione.

| Tentativo | Cosa premiava | Cosa ha imparato |
|---|---|---|
| bersaglio alternato | il gomito verso un estremo, poi l'altro | braccio fermo: il premio era già a zero a braccio abbassato, nessun gradiente |
| fascia + velocità del gomito | stare in una fascia e muoversi | 38 inversioni in 8 s, uno scuotimento veloce |
| fascia + velocità lenta + arco | coprire tutta la fascia in 1,2 s | un solo arco, poi fermo: 7 punti su 8 di premio venivano dalla posa |

La rete che gira sul Pixhawk **non ha memoria**: vede IMU, giunti e azione precedente. Un'oscillazione
richiede di sapere in che punto del ciclo ci si trova, e questa informazione non era in nessun ingresso.
La rete doveva scoprire da sola una regola del tipo "se stai salendo continua fino in cima, poi torna",
esplorando con rumore piccolo per non cadere, e con un premio che arrivava solo dopo un ciclo intero. Non
ci è arrivata, e ha preso i punti facili della posa.

Con l'orologio il problema cambia natura. La posa da raggiungere è una funzione dell'ingresso, il premio è
immediato e non ambiguo, e la rete impara una mappa "fase → angoli" con le correzioni di equilibrio. Il
saluto è arrivato in 300 iterazioni, il balletto delle braccia in 200 e quello a corpo intero in 400.

## Vantaggi rispetto al reward scritto a mano

- **Si descrive il movimento, non la regola per ottenerlo.** Un gesto nuovo sono pose chiave in
  [`nnm_env.py`](../../tools/robots/nnm_env.py) (`_setup_gesture`), nessun termine di reward da inventare.
- **Converge in fretta e in modo prevedibile.** L'errore rispetto alla clip è misurabile a ogni checkpoint
  (0,01 rad sul saluto, 0,00–0,05 sul balletto), e il video mostra riferimento e robot sovrapposti.
- **Le policy si accumulano.** Ogni gesto è un `.nnm` nella cartella del robot, scelto con `NNM_POLICY`.
  Il balletto è partito dal checkpoint del saluto, che già sapeva leggere l'orologio.
- **La camminata non cambia.** I gesti nascono da `walk_md` e ne conservano l'equilibrio; le walk esistenti
  sono state allargate a 65 ingressi con colonne a zero e hanno uscita identica al bit.
- **È la strada per le clip da video.** La pipeline del G1 (video → stima della posa umana → riadattamento
  ai giunti del robot) produce esattamente pose chiave nel tempo: entra qui senza cambiare il training.

Costo: due numeri in più nell'osservazione, un seno e un coseno per tick, circa mille pesi int8 in più nel
primo strato su duecentomila. Sotto l'1% del tempo di inferenza.

## Le clip attuali (Microban)

| Clip | Frequenza | Giunti | Descrizione |
|---|---:|---:|---|
| `wave_right` | 0,5 Hz | 3 | mano destra alta (pitch −1,10, gomito −1,00); il roll della spalla porta la mano in fuori (−0,25 → −1,05) e la riporta |
| `dance_arms` | 0,625 Hz | 6 | le braccia pompano in alternanza: una sale e si apre mentre l'altra scende e si chiude |
| `dance` | 0,625 Hz | 16 | `dance_arms` più molleggio sulle ginocchia due volte per ciclo (0,15 → 0,55 rad, anca −0,24 e caviglia −0,28 per mezzo radiante, piede piatto) e ondeggiamento laterale una volta per ciclo (anca +δ, caviglia −δ su entrambe le gambe, ±0,12 rad, bacino ±1,2 cm) |
| `arms_up` | statica | 4 | entrambe le braccia alzate |

Gli angoli delle gambe sono stati misurati sul modello MJCF cercando, per ogni piegamento del ginocchio,
l'anca e la caviglia che tengono il piede piatto e sotto il bacino. Il segno di ogni giunto è stato
verificato con la cinematica diretta prima di scrivere la clip.

## Come si addestra un gesto

```bash
# saluto: rifinitura della camminata, orologio a 0,5 Hz nella clip
.venv/bin/python tools/robots/train_velocity.py --robot microban --gesture wave_right \
    --init-nnm robots/microban/policies/walk_md.nnm --init-std 0.3 --lr 2e-4 \
    --envs 512 --workers 8 --iters 600 --save-every 100 --eval-every 100 --name wave_right

# balletto a corpo intero, partendo dal checkpoint del saluto
.venv/bin/python tools/robots/train_velocity.py --robot microban --gesture dance \
    --resume robots/microban/policies/wave_right_run/wave_right_it0300.pt --start-iter 0 --lr 2e-4 \
    --envs 512 --workers 8 --iters 800 --save-every 100 --eval-every 100 --name dance

# video con il corpo di riferimento in verde
.venv/bin/python tools/robots/play_policy.py --robot microban --nnm robots/microban/policies/dance.nnm \
    --gesture dance --stand 8 --video docs/media/microban_dance.mp4
```

Con `--gesture` il trainer tiene il comando di velocità a zero, spegne le spinte casuali, abbassa
`action_rate` a −0,02 (un radiante di braccio non va lisciato via), fissa il learning rate e mette
l'esplorazione a 0,30 sui giunti delle braccia nominati dalla clip, 0,12 su quelli delle gambe, 0,06 sul
resto. La fase iniziale di ogni episodio è casuale: la rete impara ad agganciare la clip in qualunque
punto, come succede sul robot quando si cambia policy.

## Aggiungere un gesto nuovo

1. In [`nnm_env.py`](../../tools/robots/nnm_env.py), `_setup_gesture`, aggiungere una voce a `presets`:
   `hz` (0 per una posa statica) e `keys`, un elenco `(fase, angolo)` per giunto. La clip è periodica e la
   fase va da 0 a 1.
2. Verificare i segni con la cinematica del modello (`mujoco.mj_forward` sulla posa) prima di fidarsi degli
   angoli: su Microban il pitch negativo alza la mano, il roll destro si apre per angoli negativi, il
   sinistro per angoli positivi.
3. Aggiungere il nome alle scelte di `--gesture` in `train_velocity.py` e lanciare come sopra.
4. Il file `.nnm` va in `robots/<robot>/policies/`; sul robot si seleziona con `NNM_POLICY` e si imposta
   `NNM_CLOCK_HZ` alla frequenza della clip.

Per un robot che oggi ha `extra_cmd_dim 0`, portarlo a 2 in [`catalog.py`](../../tools/robots/catalog.py),
rigenerare con `build_catalog.py` e allargare le policy esistenti con
[`pad_nnm_obs.py`](../../tools/robots/pad_nnm_obs.py): le colonne aggiunte hanno peso zero e l'uscita
non cambia.

## Deploy

| Parametro | Camminata | Saluto | Balletto | Teleop posa |
|---|---|---|---|---|
| `NNM_POLICY` | indice di `walk_md.nnm` | indice di `wave_right.nnm` | indice di `dance.nnm` | indice di `pose_cmd.nnm` |
| `NNM_CLOCK_HZ` | 0 | 0,5 | 0,625 | 0 |
| `NNM_POSE_WD` | — | — | — | 500 ms (watchdog stream) |
| `NNM_POSE_TAU` | — | — | — | 0,15 s (passa-basso) |

Il firmware fa avanzare la fase di `2π · NNM_CLOCK_HZ / rate_hz` a ogni tick della policy e scrive seno e
coseno nei due canali dopo il twist. A `NNM_CLOCK_HZ 0` i canali restano a zero, che è quello su cui le
walk sono state addestrate. Il cambio di policy usa il cross-fade di `NNM_BLEND_MS` come oggi.

## Teleoperazione: posa in ingresso via MAVLink

Oltre alle clip a orologio, Microban può eseguire una **posa comandata in tempo reale**. Una ground station
stima la posa di una persona, la riadatta a otto numeri e li manda al firmware; la policy
[`pose_cmd.nnm`](../../robots/microban/policies/pose_cmd.nnm) li esegue tenendo l'equilibrio.

### Flusso

```mermaid
flowchart LR
  cam["webcam"] --> mp["MediaPipe Pose: 33 punti 3D"]
  mp --> rt["riadattamento: 6 angoli braccia + piegamento + ondeggiamento"]
  rt --> tx["pymavlink: DEBUG_FLOAT_ARRAY name NNM_POSE"]
  tx --> fw["AP_NNMixer: watchdog + passa-basso → extra[2:10]"]
  fw --> mlp["pose_cmd.nnm (obs 73)"]
  mlp --> servo["18 servo, gambe bilanciano"]
```

Tutta l'intelligenza (stima, riadattamento, limiti) sta sulla GCS. Il firmware copia i numeri
nell'osservazione dopo l'orologio; se lo stream smette (`NNM_POSE_WD`), i canali decadono a zero con lo
stesso filtro e il robot torna in posa di riposo senza scatti.

### Layout dei canali (dopo sin, cos)

| idx | canale | unità / range |
|---|---|---|
| 0–2 | `right_shoulder_pitch/roll/elbow` | rad, offset da `q0` |
| 3–5 | `left_shoulder_pitch/roll/elbow` | rad, offset da `q0` |
| 6 | `knee_bend` | rad, 0…0,55 → ginocchio +k, anca −0,48k, caviglia −0,56k |
| 7 | `sway` | rad, ±0,15 → anche +d, caviglie −d |

### Ground station

```bash
# dipendenze: mediapipe, opencv-python (in requirements.txt)
.venv/bin/python tools/mocap/mocap_gcs.py --source camera --conn udp:127.0.0.1:14550
.venv/bin/python tools/mocap/mocap_gcs.py --source clip --clip wave_right   # prova senza webcam
.venv/bin/python tools/mocap/mocap_gcs.py --source keyboard
```

Con `--source camera`, `c` calibra la posa in piedi (baseline del piegamento), `0` manda la posa di
riposo, `q` esce. Opzione `--mirror` scambia destra/sinistra.

### Simulazione pura (senza firmware)

```bash
# terminale A: ascolta NNM_POSE e guida la policy in MuJoCo (verde = corpo comandato)
.venv/bin/python tools/robots/play_policy.py --robot microban \
    --nnm robots/microban/policies/pose_cmd.nnm \
    --pose-mavlink udp:127.0.0.1:14550 --stand 60 \
    --video docs/media/microban_pose_cmd_mavlink_wave.mp4

# terminale B: stream della clip (o della webcam). Senza firmware nessuno parla per primo alla GCS,
# quindi qui serve `udpout:` (destinazione esplicita); verso SITL/MAVProxy basta `udp:127.0.0.1:14550`
.venv/bin/python tools/mocap/mocap_gcs.py --source clip --clip wave_right --conn udpout:127.0.0.1:14550
```

Sul firmware la ricezione si vede dal `NAMED_VALUE_FLOAT` `PPO_POSE0` (primo canale filtrato): segue lo
stream e, a GCS spenta, torna a zero in `NNM_POSE_WD` + qualche `NNM_POSE_TAU`.

### Training

```bash
.venv/bin/python tools/robots/train_velocity.py --robot microban --gesture pose_cmd \
    --resume robots/microban/policies/dance_run/dance_it0400.pt \
    --envs 512 --workers 8 --iters 2000 --save-every 200 --name pose_cmd --lr 2e-4
```

Il trainer allarga automaticamente un checkpoint `.pt` da 65 a 73 ingressi (stessa logica di
`pad_nnm_obs`). L'ambiente campiona pose casuali, episodi a comando zero e il 20 % di stream dalle clip
`wave_right`/`dance`, con ritardo 0–3 tick e passa-basso 0,15 s come il firmware.

Cosa è successo nei run (`pose_cmd_v1_run`, `pose_cmd_v2_run`, `pose_cmd_run`):

- **v1** (800 iter): il premio `base_height` chiedeva 4 cm di abbassamento a piegamento pieno, ma la
  cinematica con piede piatto ne dà 0,8: il robot si accovacciava contro la posa di riferimento e le
  ginocchia restavano piegate anche a comando zero. Errore a it800: 0,10 rad. La std di esplorazione
  saliva da 0,17 a 0,37.
- **v2** (800 iter, `--std-cap`): std bloccata a 0,17 e `base_height` corretto, ma a it800 l'errore era
  0,18 rad: l'esplorazione libera del v1 era utile, non un difetto. Il tappo resta disponibile ma è
  disattivato di default.
- **v3** (= `pose_cmd_run`, 800→2000 da v1 it800 con `--keep-std` e `base_height` corretto): 0,048 rad a
  it1200, 0,030 a it1800. Il selettore "best" del trainer usa il punteggio stand/walk, che qui non
  misura nulla: `pose_cmd.nnm` è il checkpoint 1800 scelto sul tracking delle pose.

Misure con `pose_cmd.nnm`: cinque pose tenute 3 s (riposo, braccia, piegamento 0,45, ondeggiamento
0,12, posa completa) errore medio 0,026–0,038 rad; saluto in streaming MAVLink a 40 Hz 0,043 rad sulle
braccia; watchdog firmware su SITL: a stream fermo il canale passa da −1,0 a −0,02 in 0,95 s
(500 ms di attesa + costante 0,15 s), zero cadute.

## Get-up: due fasi, un solo percorso via prono

Alzarsi da terra (prono o supino → in piedi a q0) non converge in una sola fase PPO + int8. Le run
v1–v5 (~8000 iterazioni) hanno imparato solo lo squat → in piedi: il passaggio rana → seduto sui
talloni è dinamico (il baricentro deve spostarsi 7–8 cm all'indietro con l'anca a fine corsa) e la
reward è rada. È lo stesso ostacolo di [HumanUP](https://humanoid-getup.github.io/) (RSS 2025) e
[HoST](https://arxiv.org/abs/2502.08378) sul G1.

### Percorso unico

Si rotola sempre sul prono; da lì: push-up → rana → seduto sui talloni → squat → in piedi. Il
supino non ha una via propria: ha una reward di rotolamento verso la gravità da prono.

```mermaid
flowchart LR
  supine[Supino] -->|rotola| prone[Prono]
  prone --> pushup[Push-up] --> frog[Rana] --> seat[Seduto] --> squat[Squat] --> stand[In piedi q0]
  stageI["Fase I: scoperta + forza d'aiuto"] -.->|registra e rallenta| clip[Clip getup_self]
  clip -.-> stageII["Fase II: imitazione, int8, firmware"]
```

### Fase I — `getup_discover` (solo simulazione)

```bash
.venv/bin/python tools/robots/train_velocity.py --robot microban --gesture getup_discover \
    --init-nnm robots/microban/policies/getup_v5_it1500_run/getup_it1500.nnm \
    --envs 96 --workers 12 --iters 6000 --std-cap --save-every 200 --name getup_discover
```

- Partenze: 35 % prono, 25 % supino, 35 % RSI su un waypoint stabile (seduto, squat), 5 % in piedi.
- Forza d'aiuto verticale sul tronco (`xfrc_applied`): parte a 0,7 · m·g ≈ 5,6 N, **svanisce
  quando il bacino supera il 55–85 % dell'altezza in piedi** (l'equilibrio non si impara "più
  leggeri dell'aria") e scende di 0,4 N ogni volta che ≥30 % degli episodi partiti da terra finisce
  in piedi (tilt <15°, bacino >80 %, testa >85 %). Con l'init dalla v5 arriva a zero in ~2000
  iterazioni.
- Reward per stadio sull'altezza della testa (raddrizzarsi / salire / in piedi). L'altezza è la
  **media geometrica** di testa e bacino: la sola testa premiava il "seduto di lato" con il tronco
  verticale e il bacino a terra. Mentre sale: piedi sotto il baricentro (HoST), entrambi i piedi a
  terra, simmetria destra/sinistra. In piedi: bacino a quota, posa lineare verso q0, fermo, bonus di
  successo graduato sulla distanza da q0.
- **Vincoli di postura**: senza, la scoperta converge a uno stand "a papera" (anche ruotate di
  ±1,05 rad, piedi a 14 cm, braccia sature a ±2 rad come contrappeso), staticamente stabile ma
  che né `walk` né la Fase II riescono a portare a q0 (0 % in tutti i test). Nell'ambiente di
  scoperta l'azione di hip_yaw è clippata a ±0,35 rad e spalle/gomiti a ±0,7 rad da q0, con
  penalità sull'eccesso richiesto (la rete impara a restare nel range: sul firmware non c'è clip
  per giunto) e penalità di apertura anche/braccia una volta raddrizzato.
- Std esplorativa con tetto (`--std-cap`): senza, 0,50 → 1,69 in 300 iterazioni e la policy non
  imparava più nulla. lr fisso 3e-4 (1e-4 nelle riprese), regolarizzazione debole
  (`action_rate` −0,005, niente coppia). QAT int8 spento: questa policy non va sul robot.
- Criterio: ≥80 % di successo a forza zero da prono e da supino, con stand riprendibile da `walk`.

### Registrazione e clip rallentata

```bash
.venv/bin/python tools/robots/getup_record.py \
    --nnm robots/microban/policies/getup_discover.nnm \
    --slow 2 --blend-s 2 --hold-s 3 --trials 12 --check-bam
```

Sceglie i rollout riusciti (in piedi e fermo per 1 s) con jerk minimo, li rallenta di K (2 per
Microban: il rotolamento è dinamico e a K=3 la Fase II non lo riproduceva più; HumanUP sul G1 usa
8), filtra, aggiunge una **fusione min-jerk verso q0** (2 s) e una tenuta a q0
(3 s), e scrive `robots/microban/motions/getup_prone.npz` e `getup_supine_roll.npz`
(giunti, fase, quaternione e quota del tronco).
`--check-bam` ripete la clip in anello aperto sui servo BAM: su Microban **nessun K sta in piedi**
(neanche q0 da solo regge in anello aperto sui servo cedevoli), quindi la Fase II è per forza una
policy di tracking in anello chiuso, non una riproduzione.

### Fase II — `getup` (firmware)

```bash
.venv/bin/python tools/robots/train_velocity.py --robot microban --gesture getup \
    --init-nnm robots/microban/policies/getup_discover.nnm --lr 1e-4 \
    --envs 96 --workers 12 --iters 4000 --std-legs 0.20 --std-arms 0.25 --std-cap --name getup
```

Imita la clip registrata (errore lineare sui giunti + bonus stretto di tracking) con
regolarizzazione forte, QAT int8 e domain randomization come la walk. Oltre ai giunti insegue il
**tronco registrato** (gravità nel body frame, senza yaw, peso 4; quota del bacino, peso 2): il
solo tracking dei giunti si soddisfa anche restando sdraiati, e nella v9 il rotolamento da prono
non avveniva mai (0 %). Nessuna forza d'aiuto; **RSI
direttamente sulla clip** (giunti + roll/pitch del tronco registrati a fase casuale). Un solo
orologio per i due lati: la clip più corta arriva a q0 prima e lo tiene, così `NNM_GETUP_S` è la
durata della più lunga (14,2 s). Con `--init-nnm` lo schedule adattivo del lr finiva subito al pavimento
1e-5: con `--lr` esplicito il rate è fisso. Output: `robots/microban/policies/getup.nnm`.

Orologio one-shot: θ = π · min(t / `NNM_GETUP_S`, 1) → (sin, cos) finisce a (0, −1). Sul firmware
la macchina a stati `NNM_GETUP_*` passa automaticamente a questa policy se il tronco supera
`NNM_GETUP_TILT` per 300 ms (carica il `.nnm` nello slot libero), poi torna alla policy precedente
dopo l'intera clip con il robot in piedi per `NNM_GETUP_HOLD`. Verificata in SITL (plant MicroDuck, copia di
`walk.nnm` come get-up, soglia −1°): 5 cicli consecutivi trigger → `get-up active` → `get-up done,
policy 0`; batteria HIL 7/7 PASS con `service_getup()` nel loop.

```bash
.venv/bin/python tools/robots/getup_video.py --nnm robots/microban/policies/getup.nnm \
    --video docs/media/microban_getup.mp4
.venv/bin/python tools/robots/play_getup_mix.py \
    --getup robots/microban/policies/getup.nnm \
    --walk robots/microban/policies/walk_md.nnm \
    --teleop robots/microban/policies/pose_cmd.nnm \
    --video docs/media/microban_getup_walk_teleop.mp4
```

#### Risultati e lezioni della Fase II (v9 → v16, 16 500 iterazioni cumulative)

`getup.nnm` = checkpoint 16 500 (v16). Rete int8 da sdraiato, forza zero, 8 episodi per lato: in
piedi da prono 100 % (4-11 s), da supino 100 % (1,5-3 s); a fine clip errore medio da q0 0,27-0,29
rad, tronco 11-13°, comandi ai servo entro il range dei giunti (eccesso 0,6 / 0,0 rad negli ultimi
2 s); `walk.nnm` e `walk_md.nnm` prendono in carico (cross-fade 500 ms) e reggono 100 %. La
sequenza completa getup → walk → spinta → get-up automatico → walk → rotazione → teleop gira in
simulazione senza cadute non volute (`microban_getup_walk_teleop.mp4`, 44 s). Ogni versione ha
tolto un ottimo locale preciso:

- v9: tracking dei soli giunti → da prono 0 %: gli angoli della clip si riproducono anche restando
  a terra. Aggiunto il tracking del tronco registrato (gravità + quota) e K portato da 3 a 2 (il
  rotolamento è dinamico). → prono 88 %.
- v10-v12: la policy finiva sempre nella posa della Fase I (ginocchio sinistro a −0,79, gomito a
  −1,7) anche partendo da q0. Causa: a q0 il tronco di Microban è inclinato ~14° (così sta
  `walk.nnm`), ma la soglia di successo era 10° e il tronco registrato era verticale: per
  raddrizzarlo la policy tendeva le ginocchia. Soglia a 20°, tronco inseguito solo prima della
  fusione a q0, 20 % di partenze in piedi a q0 e bonus stretto a q0 nella tenuta.
- v13: **maestro** `walk.nnm` nella tenuta: azione premiata linearmente nell'errore rms rispetto a
  quella che la walk emetterebbe nello stesso stato (gradiente nello spazio delle azioni, dove il
  solo bonus di stato non trovava l'equilibrio a ginocchia piegate). Eval 100 % / 100 %.
- v14-v16: le ginocchia (range [−0,79, 2,36], q0 = 0) erano spinte **contro il fine corsa** con
  comando saturo (−2 rad): una gamba rigida gratis in simulazione, un servo in stallo sul robot.
  Penalità sul comando fuori range del giunto (`getup_cmd_limits`, −1 poi −3) oltre a quella sulla
  posizione, maestro a peso 8, lr 5e-5: eccesso di comando da 2,5-5 rad a 0-0,6 rad, braccia
  vicine a q0, passaggio alla walk 100 %. Un ginocchio resta al limite di estensione (con comando
  nel range): da verificare sui servo reali.

Il selettore del trainer (2 s in piedi con q_err < 0,35 e tronco < 20°) misura un transitorio: la
posa a fine clip, i comandi fuori range e il passaggio alla walk vanno misurati a parte. Dopo la
16 500 il run peggiora (prono 0 % alla 18 500): fermarsi al checkpoint selezionato.

## Limiti

- Un gesto "una volta sola" (get-up, inchino) usa l'orologio one-shot (`NNM_GETUP_*` / flag
  `_clock_oneshot`); le clip periodiche restano su `NNM_CLOCK_HZ`.
- Le gambe nella clip di balletto restano piccoli spostamenti a piedi fermi. Passi, salti o giri
  richiedono il tracking anche della posa del tronco, come nei lavori sul G1.
- Le pose chiave delle clip periodiche sono scritte a mano; il get-up usa invece motion
  auto-scoperte (Fase I). La teleoperazione via MediaPipe sostituisce le clip in tempo reale con un
  riadattamento cinematico semplificato (non un IK a corpo intero).
