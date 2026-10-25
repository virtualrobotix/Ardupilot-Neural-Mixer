# PhoneBot: Pixhawk, ESP32 e telefono

Questa è l'architettura proposta per l'integrazione di PhoneBot in AP_NNMixer. Non usa
ArduPilot su Android e non sposta la policy dal flight controller.

```text
telefono Android ── Bluetooth Classic / Wi-Fi ── ESP32 ── UART MAVLink 2 ── Pixhawk
      camera, UI, ARCore, teleoperazione                                  │
                                                                          │ 200 Hz IMU,
                                                                          │ 50 Hz PPO
                                                                          ▼
                                                             backend Dynamixel TTL
                                                                          │
                                                               13× XL430-W250-T
```

## Ruoli

- **Pixhawk:** legge l'IMU solidale al busto, esegue la policy `.nnm` a 50 Hz,
  applica i modi e i failsafe ArduPilot, e chiude il loop di posizione sul bus
  Dynamixel. I dati usati dalla policy non attraversano una rete wireless.
- **ESP32:** bridge trasparente UART ↔ MAVLink 2 su Bluetooth Classic (telecomando)
  e Wi-Fi (client/access point per l'app nella custodia). Non esegue la policy.
- **Telefono:** inizialmente è una ground station/twist controller; montato sul
  busto diventa camera, interfaccia e computer di percezione. Continua a essere un
  client MAVLink, non la sorgente dell'IMU del cammino.

## Messaggi

`MANUAL_CONTROL` porta il twist durante la teleoperazione. In custodia l'app può
convertire visione e ARCore in twist/heading, e inviare `VISION_POSITION_ESTIMATE` o
`OBSTACLE_DISTANCE` se utili a funzioni di alto livello. Il video non passa su MAVLink
né Bluetooth: viene mostrato dall'app, oppure usa un flusso Wi-Fi separato.

L'IMU Android non deve entrare nell'osservazione PPO: Bluetooth/Wi-Fi aggiungono jitter,
il frame del telefono cambia con custodia e utenti, e la policy va addestrata sul sensore
solidale al Pixhawk. ROS 2 può vivere nell'app o in un companion per SLAM/pianificazione,
ma il confine verso l'autopilota resta MAVLink.

## Fasi

1. **Telefono in mano:** associazione Bluetooth Classic, joystick, telemetria, arm/disarm.
2. **Telefono nella custodia:** Wi-Fi locale, la stessa app invia obiettivi di alto livello
   ottenuti da camera, persona o SLAM; la policy e il controllo dei servo non cambiano.

## Vincoli prima dell'hardware

Il firmware deve avere un backend Dynamixel con telemetria di posizione/velocità e timeout
di comunicazione. La prima PPO PhoneBot usa il singolo sin/cos di `NNM_CLOCK_HZ` e un
premio di alternanza dei piedi; così il file `.nnm` gira già nel firmware. L'attore
upstream usa invece due fasi (quattro canali): replicarlo richiederà un'estensione
dell'orologio e un nuovo training.
