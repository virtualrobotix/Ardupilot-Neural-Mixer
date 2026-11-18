# Guadagni dei servo dell'XGO Duck

Analisi del branch `getup-hd1910` di [ZXerniXZ/xgoduck_runtime_arduino](https://github.com/ZXerniXZ/xgoduck_runtime_arduino/tree/getup-hd1910) (commit `6bd4c55e`, 18 novembre 2026) e parametri di taratura presenti oggi nel progetto AP_NNMixer.

Documento di specifica, 18 novembre 2026. I valori sono quelli scritti nel codice, non una misura dei servo montati sul robot.

**Conclusione.** La policy non produce un guadagno per servo. L'uscita ONNX è un vettore di 14 posizioni. Il guadagno vive nel servo Feetech (protocollo SCS): EEPROM `0x15`/`0x16` per il valore che sopravvive allo spegnimento, registri live `50`/`51` per il valore immediato. Nel branch analizzato gambe e testa non ricevono un guadagno automatico: si impostano a mano, un servo alla volta. Le uniche scritture automatiche sono la bocca (KP 10, KD 20) e, solo in quel branch, i guadagni morbidi della calibrazione.

## 1. Dove sta il guadagno

In `python/rl_core.py` del branch il target in gradi è `home_deg + gradi(action_scale × action)`, poi un filtro passa-basso con alpha diverso per gambe e testa. I metadati ONNX di walk, get-up e roulade portano `joint_stiffness = 1` e `joint_damping = 0` su tutti i 14 giunti: sono segnaposto. `action_scale` nei metadati è 1,0.

Il nome del run di walk, dentro `run_path` dell'ONNX, contiene `servo09to13`. Si legge come una randomizzazione del guadagno servo fra 0,9 e 1,3 durante il training. È un'interpretazione del nome del run, da confermare con chi ha addestrato la rete. Nel file non c'è un vettore di guadagni.

| Registro SCS | Indirizzo | Cosa conserva |
| --- | ---: | --- |
| KP permanente | 0x15 | EEPROM. Resta dopo un ciclo di alimentazione del servo. |
| KD permanente | 0x16 | EEPROM, scritto insieme a 0x15. |
| KP live | 50 | Registro di lavoro. Vale subito, si perde allo spegnimento. |
| KD live | 51 | Registro di lavoro, scritto insieme a 50. |

Opcode di scrittura permanente: `SERVO_SET_PERM_KP_KD = 5`. Scrive EEPROM e registri live, poi rilegge entrambi e risponde ok solo se coincidono. Intervallo accettato: 0…255. Centro encoder fisso: 2047. Posizione di tenuta in calibrazione: 2048. Velocità 0 nel protocollo SCS significa velocità piena.

## 2. Parametri di taratura attuali

### 2.1 Guadagni servo nel branch getup-hd1910

| Uso | KP | KD | Velocità | Dove si scrive | Fonte |
| --- | ---: | ---: | ---: | --- | --- |
| Campi iniziali della pagina Servo setup | 5 | 10 | — | EEPROM 0x15/0x16 e live 50/51, sul servo scelto | `assets/servo.html` |
| Default dell'API se la richiesta omette i campi | 5 | 20 | — | Stessa scrittura. La pagina manda sempre i campi, quindi in uso normale vale 5 / 10 | `python/main.py`, `wire.py` |
| Bocca, ID 34. Boot, ARM, uscita da calibrazione o da sessione servo | 10 | 20 | — | Solo registri live | `TEMP_KP_MOUTH`, `TEMP_KD_MOUTH` |
| Slider di posizione in calibrazione (`SERVO_GOTO`) | 3 | 8 | 20 | Solo registri live, un servo alla volta | `CAL_MOVE_*` |
| Tenuta del centro dopo «Set center & hold» | 8 | 16 | 30 | Solo registri live | `CAL_HOLD_*` |
| Gambe e testa (14 servo), in marcia | — | — | piena | Nessuna scrittura automatica. Vale l'EEPROM del servo | `README.md`, `docs/architecture.md` |

### 2.2 Guadagni servo nel nostro progetto

| Uso | KP | KD | Dove si scrive | Fonte |
| --- | ---: | ---: | --- | --- |
| Form «Scrivi i guadagni», valori iniziali | 5 | 20 | EEPROM e live, via `NNM_SRV` operazione 24 | `unoq/app/assets/index.html` |
| Controllo API | 0…255 | 0…255 | Fuori scala la richiesta è rifiutata | `unoq/app/python/api.py` |
| Bocca, ID 34 | 10 | 20 | Solo registri live, a boot, ARM e uscita dalle sessioni | `unoq/app/sketch/duck_config.h` |
| Gambe e testa in marcia | — | — | Nessun valore nel profilo: vale l'EEPROM. La tabella si compila con `read_servo_gains.py` | `robots/xgoduck/robot/profile.json` |
| Calibrazione e slider | — | — | `SERVO_GOTO` (opcode 4): slider a 0…4095, un servo alla volta, KP 3 / KD 8 / velocità 20. Centro in calibrazione con KP 8 / KD 16 / velocità 30 | `unoq/app/sketch/sketch.ino`, `scs_bus.h` |

Il valore davvero scritto sui 14 servo di gambe e testa non è nel repository. Va letto dal robot con `tools/unoq/read_servo_gains.py` (operazione 25, `SERVO_READ`), a coppia spenta, e riportato in `servo_gains` del profilo.

### 2.3 Mappa dei 15 servo

Ordine degli slot sul bus, lo stesso di `NNM_JNT` e di `SLOT_IDS` in `unoq/app/python/gateway.py`. La policy usa un ordine diverso: i quattro giunti di collo e testa stanno in mezzo (indici 5…8), fra la gamba sinistra e la destra.

| Slot | ID | Giunto | Nel profilo oggi |
| ---: | ---: | --- | --- |
| 0 | 10 | left_hip_yaw | EEPROM (null nel profilo) |
| 1 | 11 | left_hip_roll | EEPROM (null nel profilo) |
| 2 | 12 | left_hip_pitch | EEPROM (null nel profilo) |
| 3 | 13 | left_knee | EEPROM (null nel profilo) |
| 4 | 14 | left_ankle | EEPROM (null nel profilo) |
| 5 | 20 | right_hip_yaw | EEPROM (null nel profilo) |
| 6 | 21 | right_hip_roll | EEPROM (null nel profilo) |
| 7 | 22 | right_hip_pitch | EEPROM (null nel profilo) |
| 8 | 23 | right_knee | EEPROM (null nel profilo) |
| 9 | 24 | right_ankle | EEPROM (null nel profilo) |
| 10 | 30 | neck_pitch | EEPROM (null nel profilo) |
| 11 | 31 | head_pitch | EEPROM (null nel profilo) |
| 12 | 32 | head_yaw | EEPROM (null nel profilo) |
| 13 | 33 | head_roll | EEPROM (null nel profilo) |
| 14 | 34 | bocca (fuori dalla policy) | Tabella: KP 10, KD 20 (live) |

Segno di tutti e 15 i motori sull'Uno Q: −1 (`MOTOR_SIGN`). Baud del bus: 1 Mbps. Poll dello stato: 100 Hz. Policy: 50 Hz.

### 2.4 Posa e filtri

Questi non sono guadagni del servo. Sono i numeri con cui il controllore del branch trasforma l'uscita della policy in angoli, e la posa di partenza.

| Parametro | Valore attuale nel branch | Note |
| --- | --- | --- |
| Action scale, cammino e modelli ordinari | 1,0 | 0 tiene la posa di default. Intervallo pagina 0…2 |
| Alpha gambe / alpha testa, cammino | 0,35 / 0,45 | Filtro sull'azione. Più alto, risposta più lenta |
| Pattinaggio (roller) | scala 0,8; α 0,4 / 0,5 | Home delle gambe fissa, vedi sotto |
| Capriola, solo mentre gira | α 0,05 / 0,05 | La scala segue lo slider del cammino. Poi ripristina |
| Calcio sinistro e destro | scala 0,8; α 0 / 0 | Durata 0,4 s. Comandi di velocità e posa a zero |
| Sit / stand | scala 1,0 | Seduto: comando [1, 0, 0]. In piedi: [0, 0, 0] per 1 s |
| Alpha giroscopio | 0,5 | `GYRO_ALPHA` |
| Alpha velocità giunti | 0,4 | `VEL_ALPHA`, riferimento 100 Hz |
| Tau del filtro di gravità | 0,3 s | `GRAVITY_TAU` |
| Heading hold | Kp 1,5 cammino, 1,0 pattinaggio | Banda morta 3°, limite ±0,3 rad/s |
| Bias IMU di fabbrica | roll 0°, pitch −10°, yaw 0° | Posa di default originale, misura del 13 novembre 2026 |
| Bias IMU della posa misurata | roll +1,641°, pitch +2,185°, yaw 0° | `BASELINE_IMU_BIAS_DEG`. Il get-up non lo usa |
| Corrente di protezione, campo iniziale | 1,5 A nel branch, 1,0 A da noi | Unità sul bus, da noi: 0,0065 A per conteggio |

Posa di default di fabbrica, gradi, ordine degli ID motore 10…14, 20…24, 30…34. È la posa ±22° di anca e caviglia, non la posa misurata su questa papera.

| Giunto | Sinistra | Destra |
| --- | ---: | ---: |
| Hip yaw | 0 | 0 |
| Hip roll | −5 | +5 |
| Hip pitch | −22 | +22 |
| Knee | 0 | 0 |
| Ankle | +22 | −22 |

Collo e testa nella stessa posa: neck pitch +20°, head pitch +20°, head yaw 0°, head roll 0°, bocca 0°. Fonte: `DEFAULT_DEG` in `rl_core.py`.

Posa in piedi misurata su questa papera HD-1910, gradi. Il reset della pagina Control ripristina questa, non la ±22°. Il get-up continua a usare la posa di fabbrica e non legge `stance.json`.

| Giunto | Sinistra | Destra |
| --- | ---: | ---: |
| Hip yaw | 0 | 0 |
| Hip roll | −5 | +5 |
| Hip pitch | −26,24 | +26,24 |
| Knee | −0,28 | +0,28 |
| Ankle | +18 | −18 |

Collo e testa restano a +20° / +20° / 0° / 0°. Fonte: README del branch, sezione «Measured standing pose», valori in `python/stance.py`.

Home delle gambe del pattinaggio, gradi, fissa per ID. Collo e testa tengono la home del modello.

| ID | Giunto | Gradi | ID | Giunto | Gradi |
| ---: | --- | ---: | ---: | --- | ---: |
| 10 | left hip yaw | 0 | 20 | right hip yaw | 0 |
| 11 | left hip roll | +2 | 21 | right hip roll | −2 |
| 12 | left hip pitch | −8,5 | 22 | right hip pitch | +8,5 |
| 13 | left knee | +8,5 | 23 | right knee | −8,5 |
| 14 | left ankle | +17 | 24 | right ankle | −17 |

Home letta dai metadati ONNX `default_joint_pos`, radianti, ordine della policy (gamba sinistra, collo e testa, gamba destra). Il get-up del branch la usa (`use_model_home`). Il nostro profilo `robots/xgoduck/robot/profile.json` tiene invece la posa di fabbrica in radianti, condivisa da tutte le policy tranne il roller.

| Giunto (ordine policy) | Walk e roulade | Get-up HD-1910 |
| --- | ---: | ---: |
| left_hip_yaw | 0 | 0 |
| left_hip_roll | −0,087 | −0,087 |
| left_hip_pitch | −0,384 | −0,457 |
| left_knee | −0,005 | −0,005 |
| left_ankle | +0,384 | +0,314 |
| neck_pitch, head_pitch | 0,349 e 0,349 | 0,349 e 0,349 |
| head_yaw, head_roll | 0 e 0 | 0 e 0 |
| right_hip_yaw | 0 | 0 |
| right_hip_roll | +0,087 | +0,087 |
| right_hip_pitch | +0,384 | +0,457 |
| right_knee | +0,005 | +0,005 |
| right_ankle | −0,384 | −0,314 |

### 2.5 Simulazione

Il plant SITL di MicroDuck, quando il pacchetto `bam` è disponibile, chiude un modello di tensione XL330 con `kp_fw = 200` e `vin = 7,4 V` (`plant/mujoco_json_plant.py`). Il profilo XGO Duck ha `sim.actuator = "position"` e non elenca kp né kd. I 200 N·m/rad del modello BAM non sono i conteggi 0…255 del servo SCS.

## 3. Cosa dice la documentazione

### Branch getup-hd1910

`README.md`, sezione Servo setup, punto 4. Un click su Set KpKd scrive e verifica sia l'EEPROM sia i registri live, così il valore vale subito e sopravvive a un ciclo di alimentazione. I campi partono da 5 e 10. La lettura del servo non sovrascrive i campi.

Stessa pagina, paragrafo sulla protezione. I KP/KD di gambe e testa sono conservati all'avvio, all'arming e durante la calibrazione. L'app scrive guadagni temporanei solo sulla bocca, ID 34: KP 10, KD 20. Un Set KpKd esplicito aggiorna comunque il servo selezionato. I valori temporanei già applicati non vengono ripristinati da soli: per ricaricare l'EEPROM serve spegnere e riaccendere l'alimentazione dei servo.

`docs/architecture.md`, in cinese, ripete la regola e precisa gli indirizzi: EEPROM `0x15/0x16`, registri 50/51, opcode 5, verifica con rilettura di entrambi. La tenuta del centro dopo la calibrazione usa i guadagni che il servo aveva già.

### Il nostro repository

`README.md`, `README.it.md` e `docs/progetto-nnmixer-ardupilot-ppo.md` non hanno una sezione sui guadagni dei servo. L'elenco parametri di quel documento (`NNM_ENABLE`, `NNM_POLICY`, `NNM_VX_MAX`, `NNM_ATT_TAU` e gli altri) non comprende un parametro di guadagno servo, perché non esiste. `docs/robots/` non ha ancora la scheda `xgoduck.md`. La voce `xgoduck` di `tools/robots/catalog.py` ha ora il blocco `servo_gains` (sezione 5.1). La scheda Servo dell'app Uno Q era un modulo su un solo ID con il numero di servo fisso a 15: ora è una tabella per servo configurato (sezione 5.7).

## 4. Differenza fra i due sketch

La meccanica KP/KD permanente (opcode 5, verifica EEPROM e live) è già la stessa. Il branch aggiunge queste parti; il nostro sketch le ha portate (sezione 5), tranne IMU e posa misurata:

- `setTempKpKd`, scrittura dei soli registri 50/51.
- Costanti `CAL_MOVE_KP 3`, `CAL_MOVE_KD 8`, `CAL_MOVE_SPEED 20`, `CAL_HOLD_KP 8`, `CAL_HOLD_KD 16`, `CAL_HOLD_SPEED 30`.
- Opcode `SERVO_GOTO = 4`, con la posizione grezza nell'unione del pacchetto servo, che resta di 12 byte.
- Velocità nel goal singolo. In calibrazione il centro si salva senza togliere prima la coppia, poi il giunto viene tenuto con i guadagni `CAL_HOLD_*`.

Il nostro sketch ha in più l'IMU su MPU-6000 SPI e I2C oltre al QMI8658, il banner LED e la modalità `DUCK_BRIDGE_SLOW`. Il branch ha il solo MPU-6000 SPI bit-banged. Quella parte di IMU, la posa misurata, `use_model_home` e la rete get-up nuova restano fuori da questo passo. Il README di quel repo non dichiara una licenza: si riscrive l'idea e si riportano le costanti, non si copiano i file.

## 5. Modifiche implementate

Stato al 18 novembre 2026: il codice è scritto e compilato dove possibile. Le verifiche sono nella sezione 6. Il firmware dello sketch non è stato compilato (manca la toolchain Zephyr in questo ambiente).

### 5.1 Tabella nel profilo e in `skills.bin`

La tabella vive in `robots/xgoduck/robot/profile.json`, chiave `servo_gains`: una voce per slot, nell'ordine del firmware (ID 10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34). `kp` e `kd` null vuol dire «lascia il valore dell'EEPROM del servo». Il profilo oggi ha solo la bocca (ID 34): KP 10, KD 20. Gambe e testa sono null in attesa della lettura dal robot.

`tools/robots/pack_robot_bin.py` scrive `skills.bin` versione 2 quando il profilo contiene `servo_gains`, altrimenti versione 1 come prima. La coda aggiunta in fondo al file è: `n` (u8), `mask` (u16, bit i = lo slot i ha un valore), poi `kp[n]` e `kd[n]`. Con 15 slot la coda è di 33 byte. Il parser dell'autopilota accetta entrambe le versioni e controlla la lunghezza esatta.

Nota rispetto al piano: non si usa un byte 0xFF come «non impostato». Il bit nella maschera è il segnale.

### 5.2 Un frame per tutta la tabella

Il piano prevedeva 15 pacchetti DQV2, uno per servo. Non si fa: il firmware tiene un solo pacchetto servo in attesa, quindi una raffica di 15 si perderebbe. La tabella invece viaggia in un solo frame `DQG1` di 40 byte: magic, `seq` (u32), `mask` (u16), `kp[15]`, `kd[15]`. Il bridge lo consegna a `receiveGains` nello sketch.

La risposta arriva come `DQS3` con opcode MCU 11 (`SERVO_SET_RUN_GAINS`). Sul lato autopilota è l'operazione `AP_JointBus::Op::SERVO_SET_RUN_GAINS = 30`. Con la coppia accesa il firmware rifiuta il frame (`ok = 0`) e lo tiene solo se la coppia è spenta.

### 5.3 Pubblicazione dall'autopilota

`AP_NNMixer::publish_run_gains()`, chiamata da `update()` dopo il servizio del bus:

- pubblica solo a bus pronto, veicolo disarmato, tabella valida e parametro `NNM_GAINS` a 1;
- riprova ogni secondo finché la risposta non è `ok`;
- se il link cade, `_gains_done` torna a falso e alla risalita si ripubblica.

Non ci sono gruppi di 8 frame: la tabella è un solo frame.

Parametro `NNM_GAINS`, indice di gruppo 32 (l'ultimo usato era `XD_ACCX` a 31). 0: non pubblicare, i servo restano sull'EEPROM. 1 (default): pubblica la tabella se presente nel profilo.

### 5.4 Firmware: applicazione ai registri live

`applyRunGains()` in `scs_bus.h` scrive i registri live 50 (KP) e 51 (KD) solo per gli slot in tabella. Viene chiamata a boot, all'uscita da calibrazione, all'ingresso e all'uscita da sessione servo, al timeout del link e all'ARM. La bocca (slot 14) segue questa regola: se è nella tabella prende i suoi valori, altrimenti resta su `setMouthKpKd()` con KP 10 e KD 20, come prima.

La tabella è in RAM e si perde allo spegnimento. L'EEPROM non viene scritta da questa strada.

### 5.5 Calibrazione e `SERVO_GOTO`

Portato dal branch `getup-hd1910` come idea, con costanti e comportamento riscritti (nessun file copiato):

- **Calibrazione, centro di un giunto** (`CAL_CENTER_ONE`): non toglie più la coppia. Sblocca il servo, lo tiene con `CAL_HOLD_KP 8`, `CAL_HOLD_KD 16` e velocità `CAL_HOLD_SPEED 30` verso `CAL_HOLD_POS 2048`. I centri già messi in fase di staging restano.
- **Slider** (`SERVO_GOTO`, autopilota op 31, MCU op 4): un servo alla volta. Accende la coppia di quel solo servo con `CAL_MOVE_KP 3`, `CAL_MOVE_KD 8`, e manda il goal con `CAL_MOVE_SPEED 20`. Un goal fuori da 0…4095 è rifiutato. Se lo slider passa a un altro servo, il precedente perde la coppia salvo che sia tenuto in calibrazione.

### 5.6 Operazioni `NNM_SRV`

| Operazione | Nome | Dove va | Note |
| ---: | --- | --- | --- |
| 24 | `set_gains` | MCU op 5 | Scrive EEPROM e live, verifica la rilettura. Invariata |
| 25 | `read` | MCU op 6 | Legge KP, KD, centro e posizione. Usata da «Leggi» e «Leggi tutti» |
| 30 | `run_gains` | MCU op 11 con `DQG1` | Manda la tabella del profilo |
| 31 | `goto` | MCU op 4 | Slider, arg0 = goal 0…4095 |

L'operazione 3 prevista dal piano non è stata introdotta: la rilettura della tabella si fa con l'operazione 25 riga per riga.

### 5.7 Pagina web

Nella scheda Servo, sezione **Servo configurati**:

- **Applica la tabella KP/KD**: manda la tabella del profilo (operazione 30). Disabilitata con robot armato o sessione servo aperta.
- **Leggi tutti**: legge ogni riga (operazione 25).
- **Una riga per servo.** Le righe seguono `joints` del telemetry (`NNM_JNT`, campo `slots`). Il numero di righe non è più il 15 fisso. La pagina ricostruisce la tabella solo se il numero cambia.

Ogni riga ha: nome e ID, stato (presente o assente), encoder grezzo, slider «Vai a» (0…4095), campi Kp e Kd, e i pulsanti **Leggi**, **Scrivi** (operazione 24, EEPROM più live), **Ping**, **Centro qui** (reset del centro). Le colonne «Kp EEPROM» e «Kd EEPROM» mostrano i valori letti con **Leggi**. I campi di una riga partono da 5 e 20 e si aggiornano con la lettura o la scrittura.

### 5.8 Strumento di lettura dal robot

`tools/unoq/read_servo_gains.py` legge KP e KD in EEPROM dei servo configurati tramite l'API del gateway. Chiede conferma, salvo con `--yes`. Stampa una tabella e il blocco `servo_gains` pronto per `profile.json`, o un JSON con `--json`. Con la coppia spenta non muove i servo.

### 5.9 Cosa resta fuori

- Guadagni diversi per policy o per skill. Richiederebbe un secondo set, applicato al cambio di policy.
- Uno scalare di azione per giunto dentro il loop della policy. Nel repo analizzato non c'è.
- Posa misurata, `use_model_home` e la rete get-up nuova del branch.
- IMU MPU-6000 del branch: il nostro sketch tiene il suo driver IMU.

## 6. Verifica: cosa è stato provato e cosa no

### 6.1 Provato

| Verifica | Esito |
| --- | --- |
| `tests/test_servo_gains.py` (packer, versioni 1 e 2, round-trip) | 7 test superati |
| Parser `AP_NNMixer_Policy` con harness C fuori dal repo | Versioni 1 e 2 lette correttamente, lunghezze errate scartate |
| Compilazione SITL di ardurover con le modifiche AP | Riuscita, nessun warning nuovo |
| `unoq/app/assets/app.js` con `node --check` | Sintassi valida |
| Compilazione Python di gateway, API, bench, emulatore | Riuscita |
| `tools/unoq/gateway_test.py --sitl --gcs-port 14750` | **60 superati, 0 falliti**. Include `goto` valido, `goto` fuori scala rifiutato, tabella rifiutata dentro una sessione |
| `tools/unoq/bench_test.py --sitl` (run pulita, senza processi SITL estranei) | 88 superati, 1 fallito, 1 saltato. Il fallito è la frequenza policy (6.2). Tutti i controlli nuovi passano: goto a 2600 raggiunto, goto 5000 rifiutato, tabella rifiutata in sessione, tabella inviata dopo la sessione con maschera 0x4000, bocca 10/20 nell'MCU, tabella applicata all'ARM. Il ritorno alla posa senza salti passa |

### 6.2 Falliti nel bench, stato

Un controllo fallisce ancora: la frequenza della policy a 50 Hz. Nella run pulita misura 41,3 Hz, con mediana 20,1 ms ma 99° percentile di 80,6 ms. Non è stato ancora dimostrato che il difetto sia nel codice.

Cause verificate e cause probabili:

1. **Processi SITL estranei (risolto).** Quattro `ardurover` SITL avviati circa 20 minuti prima (sysid 101–104) mandavano heartbeat alla porta 14550 e usavano circa il 57% di CPU. Il 14550 è la porta del gateway test: con quei processi attivi `armed` alternava vero e falso ogni 0,7 s. Terminati su richiesta. Con i processi attivi il ritorno alla posa senza salti è fallito in tutte e cinque le run (salti da 4° a 51° al disarmo). Nell'unica run pulita passa. Una sola run non basta a chiuderlo: va ripetuto.
2. **Carico della macchina (probabile).** Dopo la terminazione restava un processo Java esterno al progetto (PID 45050) a circa 207% di CPU, con load average 4,5. Non è stato toccato perché non è nostro. Il controllo sulla frequenza misura il tempo reale e sotto carico diventa instabile.
3. **Codice (da escludere).** Il codice della rampa (`drive_outputs`) non è cambiato rispetto a HEAD: nel diff di `AP_NNMixer.cpp` non ci sono righe rimosse. Una prova pulita richiede la stessa misura a carico basso, o un baseline di HEAD da confrontare.

### 6.3 Non provato

- Compilazione dello sketch Zephyr (manca la toolchain).
- Build arm64 di ArduRover per Uno Q (`unoq/build_ardurover.sh`).
- Lettura reale dei KP e KD dei 14 servo di gambe e testa (serve il robot).
- Test del browser sulla pagina.
- Prova sul robot, a coppia spenta, con sospensione.

## 7. Documentazione

- Questo file e il PDF `docs/servo-guadagni-xgoduck.pdf`, rigenerato dopo questa revisione.
- `unoq/app/THIRD_PARTY_NOTICES.md`: provenienza di calibrazione morbida e `SERVO_GOTO`. Fatto.
- `README.md` e `README.it.md`: riga del parametro `NNM_GAINS` e link al documento. Fatto. Sezione dedicata ai guadagni non ancora scritta.
- `docs/progetto-nnmixer-ardupilot-ppo.md`: `NNM_GAINS` nell'elenco dei parametri. Fatto. Operazioni `NNM_SRV` 24, 25, 30 e 31 e formato `skills.bin` v2 da aggiungere.
- `tools/robots/catalog.py`: blocco `servo_gains`. Fatto. La scheda `docs/robots/xgoduck.md` non è stata rigenerata: `build_catalog.py` senza argomenti rigenera tutte le schede, quindi va lanciato solo su richiesta.

## 8. Rischi

- I numeri 5 e 20 nei campi della pagina sono valori iniziali, non una taratura misurata sull'HD-1910. Su servo diversi non vanno copiati.
- Ogni scrittura SCS occupa il bus. L'applicazione all'ARM sta fuori dal ciclo a 100 Hz, ma va verificato che non sfori la finestra di risposta.
- L'operazione 11 va rifiutata con la coppia accesa. Il firmware lo fa, ma non è ancora provato sull'hardware.
- L'indice 32 di `NNM_GAINS` è nuovo. Gli indici già assegnati non si toccano: cambiarli rompe i parametri salvati a bordo.
- Il controllo di frequenza 50 Hz del bench dipende dal carico della macchina. Va ripetuto a carico basso prima di considerarlo chiuso.
