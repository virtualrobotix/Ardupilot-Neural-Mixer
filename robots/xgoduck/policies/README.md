# Policy per XGO Duck (Arduino Uno Q)

Solo policy per `xgoduck` (14 giunti, osservazione 61). Sulla microSD: `/APM/nnm/xgoduck/policies/`.

- `0_walk.nnm` — camminata (`xgoduck_walk.onnx`). Comando: twist e testa.
- `1_getup.nnm` — rialzata dopo una caduta (`xgoduck_getup.onnx`), `NNM_GETUP_IDX 1`.
- `2_roller.nnm` — pattinata per il robot con i roller (`xgoduck_roller.onnx`): home propria, scala 0,8 incorporata nei pesi.
- `3_pick.nnm` — raccolta, skill 1 (`xgoduck_pick.onnx`): 4 s, comando (cos, sin) della fase.
- `4_roulade.nnm` — capriola, skill 2 (`xgoduck_roulade.onnx`): 1,7 s.
- `5_sitstand.nnm` — seduto / in piedi, skill 3 e 4 (`xgoduck_sitstand.onnx`).
- `6_kick_left.nnm` — calcio sinistro, skill 5 (`xgoduck_ballkick_left.onnx`): 0,4 s, scala 0,8 incorporata.
- `7_kick_right.nnm` — calcio destro, skill 6 (`xgoduck_ballkick_right.onnx`): 0,4 s, scala 0,8 incorporata.
