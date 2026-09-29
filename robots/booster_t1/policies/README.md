# Policy per Booster T1

Solo policy per `booster_t1` (12 giunti, osservazione 47). Sulla microSD: `/APM/nnm/booster_t1/policies/`.

- `walk_booster.nnm` — la policy pubblicata da Booster (deploy/models/T1.pt, TorchScript 47-256-128-128-12 ELU) convertita al contratto NNMixer da tools/robots/import_booster_t1.py: permutazione delle colonne del primo strato, fattore 0,1 sulle velocità dei giunti, int8 per riga (scarto massimo 0,06 rad su osservazioni casuali). Nel nostro ambiente (PD a 200 Hz, quantizzazione PWM, ritardo 0-1 tick, gravità AHRS): avanti 0,51 m/s a comando 0,5 e 0,73 a 0,8, laterale 0,21 a 0,3, indietro 0,36 a 0,4, rotazione 0,65 rad/s a 0,8; 30 s senza cadere. Richiede `NNM_ATT_SRC 1` e l'orologio a 1 Hz (`NNM_CLOCK_HZ 1`) mentre c'è un comando, zero da fermo.
