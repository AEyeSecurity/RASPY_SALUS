# COMMS UART ESP32 <-> Raspberry Pi (estado real del firmware)

Este documento describe lo que **hace hoy** el codigo en `src/pi_comms.cpp`,
`src/quad_functions.cpp` y `src/pid.cpp`.

## 1. Configuracion UART usada por el proyecto

Definida en `src/main.cpp` (`g_piCommsConfig`):

- Puerto: `UART_NUM_0`
- Pines: `TX=GPIO1`, `RX=GPIO3`
- Baudrate: `460800`
- Formato: `8N1`, sin paridad, sin flow control
- Buffer driver: RX `512`, TX `256`
- Tarea RX: periodo `1 ms`, `uart_read_bytes` con timeout `2 ms`
- Tarea TX: periodo `10 ms` (100 Hz)

## 2. Formato de tramas

### 2.1 Pi -> ESP32 (6 bytes)

- Byte 0: `0xAA`
- Byte 1: `ver_flags` (nibble alto version, nibble bajo flags)
- Byte 2: `steer_i8` (`-100..100` esperado)
- Byte 3: `accel_i8` (`-100..100` esperado)
- Byte 4: `brake_u8` (`0..100` esperado)
- Byte 5: `crc8` (CRC-8 Dallas/Maxim sobre bytes 0..4)

Flags en `ver_flags` (nibble bajo):

- Bit 0: `ESTOP`
- Bit 1: `DRIVE_EN`
- Bit 2: `ALLOW_REVERSE`
- Bit 3: reservado

### 2.2 ESP32 -> Pi (4 bytes)

- Byte 0: `0x55`
- Byte 1: `status_flags`
- Byte 2: `telemetry_u8` (velocidad)
- Byte 3: `crc8` (CRC-8 Dallas/Maxim sobre bytes 0..2)

Status bits:

- Bit 0: `READY`
- Bit 1: `FAULT`
- Bit 2: `OVERCURRENT`
- Bit 3: `REVERSE_REQ`

Semantica de `telemetry_u8`:

- `0..254`: velocidad actual en `km/h`
- `255`: `N/A` (sin velocidad valida disponible)

## 3. Comportamiento RX real (`taskPiCommsRx`)

- Solo sincroniza con header `0xAA`.
- Si aparece otro `0xAA` antes de completar 6 bytes, cuenta `framesMalformed` y resincroniza.
- Si CRC falla, descarta trama y cuenta `framesCrcError`.
- Si CRC es valido, actualiza snapshot (`PiCommsRxSnapshot`) y `framesOk`.

Campos derivados que calcula RX:

- `wantsReverse = (accelRaw < 0)`
- `reverseRequestActive = wantsReverse && !allowReverse`
- `reverseGranted = wantsReverse && allowReverse`
- `accelEffective`:
  - negativo solo si `reverseGranted`
  - positivo pasa directo
  - negativo sin `ALLOW_REVERSE` se fuerza a `0`

## 4. Comportamiento TX real (`taskPiCommsTx`)

- Envia cada 10 ms: `[0x55, status, telemetry, crc]`.
- `status` sale de `g_txState.statusFlags`, pero:
  - `READY` queda forzado en `piCommsSetStatusFlags`.
  - `REVERSE_REQ` no se setea manualmente: se calcula desde RX
    (`g_reverseRequestActive`) en cada envio.
- `telemetry` por defecto inicia en `255` (`N/A`) y en ese modo se calcula
  automaticamente desde `speed_meter`:
  - `0..254` cuando hay `SpeedMeterSnapshot` valido.
  - `255` si `driverReady=false`, `hasFrame=false`, `speedKmh<0` o el ultimo
    frame tiene edad `>500 ms`.
- Override manual opcional: si se llama `piCommsSetTelemetry(x)` con `x!=255`,
  TX usa ese valor fijo y no la velocidad.

## 5. Como se usan los datos en control (importante)

### 5.1 Traccion/freno (`taskQuadDriveControl`)

Se considera frame de Pi "fresco" si su edad es <= `120 ms`.

- Si Pi esta fresca y `ESTOP=1`:
  - throttle inhibido (duty minimo)
  - `commandValue=0`
  - freno aplicado al `100%`
- Si Pi esta fresca y `DRIVE_EN=1`:
  - throttle usa `accelEffective`
- Si Pi esta fresca:
  - el freno SIEMPRE sale de `brake_u8` (0..100)
- Si Pi NO esta fresca:
  - el sistema cae a control RC para traccion y freno

### 5.2 Direccion (`taskPidControl`)

- Si Pi esta fresca (<= `120 ms`), el PID usa `piSnapshot.steer`.
- Si no, vuelve a steering de RC.
- Hoy `steer` de Pi no depende de `DRIVE_EN` ni de `ESTOP`.

## 6. Failsafe real vs supuesto comun

Lo que **si** ocurre al perder tramas frescas de Pi:

- Se dejan de usar comandos Pi.
- Drive/PID vuelven a RC cuando corresponde.

Lo que **no** ocurre automaticamente en `pi_comms`:

- No existe una rutina que fuerce globalmente `brake=100`, `drive_en=0` o
  `steer=0` por timeout dentro del modulo UART.

## 7. CRC usado

`crc8_maxim` en `src/pi_comms.cpp`:

- Init: `0x00`
- Polinomio: `0x31`
- Procesamiento MSB-first (Dallas/Maxim)

## 8. Observabilidad

Comandos Telnet utiles (`src/ota_telnet.cpp`):

- `comms.status`: snapshot actual (edad de frame, flags, accel/brake efectivos,
  contadores OK/CRC/malformed)
- `comms.reset`: resetea contadores de RX

---

Si cambias `PiCommsConfig`, `kPiSnapshotFreshTicks` o la logica de
`taskQuadDriveControl/taskPidControl`, actualiza este archivo.
