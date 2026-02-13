# ESP32 ↔ Raspberry Pi UART Link

Guía paso a paso para usar la UART (GPIO3/GPIO1) del ESP32 como enlace de control con una Raspberry Pi siguiendo el protocolo descrito en `COMMS.md`.

---

## 1. Hardware

| Raspberry Pi | ESP32             | Nota                         |
|--------------|------------------|------------------------------|
| GPIO14 (TX)  | GPIO3 / RX0       | UART0 recibe desde la Pi     |
| GPIO15 (RX)  | GPIO1 / TX0       | UART0 transmite a la Pi      |
| GND          | GND               | Masa común obligatoria       |

- Nivel lógico: **3.3 V**.  
- Para cables >50 cm considera transceptores RS‑485 (MAX3485/SN75176).

---

## 2. Firmware y tareas

El proyecto crea dos tareas FreeRTOS dedicadas al enlace:

| Tarea        | Archivo          | Frecuencia | Rol principal                                     |
|--------------|------------------|------------|---------------------------------------------------|
| `PiUartRx`   | `src/pi_comms.cpp` | ~1 kHz     | Deserializa frames `0xAA`, valida CRC‑8 y publica un snapshot seguro (`PiCommsRxSnapshot`). |
| `PiUartTx`   | `src/pi_comms.cpp` | 100 Hz     | Envía frames `0x55` con `status_flags` y `telemetry_u8` (velocidad km/h codificada). |

- Se configuran mediante `PiCommsConfig` en `src/main.cpp`.  
- El logging detallado se activa con `debug::kLogPiComms`.

---

## 3. Protocolo (resumen operativo)

### Pi → ESP32 (6 bytes)

```
0: 0xAA
1: ver_flags (bits 7-4 versión, bits 0-2 = ESTOP/DRIVE_EN/ALLOW_REVERSE)
2: steer_i8  (-100..100)
3: accel_i8  (-100..100)
4: brake_u8  (0..100)
5: CRC-8 Dallas/Maxim (bytes 0-4)
```

### ESP32 → Pi (4 bytes)

```
0: 0x55
1: status_flags (READY/FAULT/OVERCURRENT/REVERSE_REQ)
2: telemetry_u8 (0..254 = km/h, 255 = N/A)
3: CRC-8 Dallas/Maxim (bytes 0-2)
```

`telemetry_u8` se genera automaticamente desde `speed_meter` cuando
`g_txState.telemetry == 255`:

- usa `speedKmh` si el snapshot es valido y su edad es `<=500 ms`;
- envia `255` si no hay dato valido (`driverReady=false`, `hasFrame=false`,
  `speedKmh<0` o frame stale).

Si `piCommsSetTelemetry(x)` recibe `x != 255`, ese valor manual tiene prioridad
sobre la codificacion de velocidad.

Implementación de CRC: `crc8_maxim` en `src/pi_comms.cpp`.

---

## 4. Flujo de control

### 4.1 Tracción y freno (frame fresco <=120 ms)

- Si `ESTOP=1`, `taskQuadDriveControl` inhibe acelerador (`cmd=0`, duty mínimo) y fuerza `brake=100%`.
- Si `ESTOP=0` y `DRIVE_EN=1`, usa `accelEffective` como comando de tracción.
- Si `ESTOP=0` y `DRIVE_EN=0`, la tracción queda en RC (no en Pi).
- Cuando el frame de Pi está fresco, el freno siempre se toma de `brake_u8` (0..100) vía `quadBrakeApplyPercent`.

### 4.2 Reversa segura

1. La Pi envía `accel_i8 < 0`.
2. En RX, la ESP32 calcula `reverseRequestActive = wantsReverse && !allowReverse`.
3. En TX, el bit `REVERSE_REQ` se refleja desde `reverseRequestActive`.
4. Solo cuando llega `ALLOW_REVERSE=1`, `accelEffective` habilita el valor negativo real.
5. Si `ALLOW_REVERSE=0` y `accel_i8` sigue negativo, `accelEffective` se mantiene en `0`.

### 4.3 ESTOP y timeout de enlace

- `ESTOP` aplica freno inmediato y corta tracción mientras el frame esté fresco.
- Si el frame deja de estar fresco (>120 ms), `taskQuadDriveControl` y `taskPidControl` vuelven a RC.
- En `pi_comms` no hay una rutina de timeout que fuerce globalmente `brake=100`.

### 4.4 Dirección

- `steer_i8` sí se usa: `taskPidControl` toma `piSnapshot.steer` cuando Pi está fresca.
- Si el frame envejece (>120 ms), vuelve a dirección RC.
- El uso de `steer_i8` no depende de `DRIVE_EN` ni de `ESTOP`.

---

## 5. Debug y monitoreo

| Recurso                         | Descripción                                                       |
|--------------------------------|-------------------------------------------------------------------|
| `debug::kLogPiComms` (main.cpp)| Habilita logs `[PI][RX]`/`[PI][TX]` con flags decodificados.      |
| Telnet cmd `comms.status`      | Snapshot en vivo (edad del frame, flags, accel/brake efectivos). |
| Telnet cmd `comms.reset`       | Reinicia contadores de frames OK/CRC/malformed.                  |

Recomendado: activar `debug::kLogDrive` temporalmente para ver qué comandos terminan alimentando el PWM.

---

## 6. Pasos de uso

1. **Compilar y flashear** con PlatformIO (`pio run -t upload`).  
2. **Conectar hardware** según la tabla de la sección 1, compartir GND.  
3. **Iniciar la Raspberry Pi** y ejecutar `test_comms.py` (o tu proceso) a 460 800 bps.  
4. **Verificar handshake**:  
   - En Telnet ejecutar `comms.status` → debería mostrar `driver=READY` y `ageMs` estable.  
   - Mover `accel_i8` positivo con `DRIVE_EN=1`; comprobar logs `[PI][RX]` y `[DRIVE]`.  
   - Solicitar reversa (`accel<0`). Confirmar que aparece `reverse{req=Y wait=Y}` y que al activar el relé (`ALLOW_REVERSE`) cambia a `granted=Y`.  
5. **Probar ESTOP**: setear `ESTOP=1` y validar que `cmd=0`, `brake=100` y duty en mínimo.  
6. **Integrar en el vehículo** una vez que no existan CRC errors y los tiempos (`ageMs`) se mantengan <50 ms.

---

## 7. Troubleshooting rápido

| Síntoma                                   | Causa probable / acción                                        |
|-------------------------------------------|----------------------------------------------------------------|
| `driver=NOT_READY` en `comms.status`      | La UART no inicializó; revisar `piCommsInit` y cableado.       |
| `ok=0 crcErr>0`                           | Ruido o baud incorrecto; revisar velocidad y masa compartida.  |
| `reverse wait=Y` no pasa a `granted=Y`    | La Pi no activó `ALLOW_REVERSE`; chequear relé y lógica.       |
| `cmd=0 (RC)` aun con Pi en marcha         | `DRIVE_EN` en 0 o frames viejos (>120 ms); revisar envío/tiempos. |
| `ESTOP` no frena                          | Verificar que bit 0 del `ver_flags` realmente se escribe y llega (logs `[PI][RX]`). |

Con esta guía deberías poder cablear, probar y depurar la comunicación sin tocar el resto del firmware.
