# Policy per Freenove Robot Dog

Solo policy per `freenove` (12 giunti, osservazione 45). Sulla microSD: `/APM/nnm/freenove/policies/`.

- `walk_v19.nnm` — iterazione 3000 di walk_v19 (punteggio 0,61), il checkpoint da cui è partito walk_v20: avanti 0,11 m/s con passo a 2 Hz e sollevamenti di 27–29 mm davanti; ruotava strisciando su una zampa e indietro stava fermo.
- `walk_v20.nnm` — stesso run, iterazione 5000 (punteggio 0,865, il massimo): avanti 0,15 m/s esatto, rotazione 0,56 rad/s, indietro 0,07, laterale 0,06. Con l'esplorazione ormai a 0,06 rad il passo è diventato asimmetrico (avanza soprattutto con FR e RL, FL e RR quasi sempre a terra): punteggio pari a walk_v20_it3400 ma passo meno naturale.
- `walk_v20_it3400.nnm` — policy di riferimento: ricetta Go2 (only_positive, contatti illegali e limiti di postura che chiudono l'episodio, critico privilegiato, action_scale 0,25) ripresa da walk_v15 → v19 con air time a touchdown, debounce dei contatti, foot_hold e no_progress. Iterazione 3400 (17000 epoche PPO, punteggio 0,86). Sopravvive sempre, tronco a 10 cm, zampe chiuse: avanti 0,14 m/s a comando 0,15, rotazione 0,80 rad/s a comando 0,6, indietro 0,08 m/s, laterale 0,07 m/s. Passo a quattro zampe in ogni direzione (2–3 Hz per zampa).
- `walk_v7.nnm` — addestrata da zero sul contratto ArduPilot in int8 (QAT), ambiente walk_v7 (passo da cane, un comando per asse). Checkpoint dell'iterazione 3000 (15000 epoche PPO, punteggio 0,59). In valutazione sopravvive sempre: avanti 0,21 m/s a comando 0,15; indietro, laterale e rotazione non ancora seguiti.
