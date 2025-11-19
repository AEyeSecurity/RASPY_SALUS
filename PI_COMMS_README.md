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
| `PiUartTx`   | `src/pi_comms.cpp` | 100 Hz     | Envía frames `0x55` con los `status_flags` y `telemetry_u8`. |

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
2: telemetry_u8 (0-100 = %, 101..103 eventos, 255 N/A)
3: CRC-8 Dallas/Maxim (bytes 0-2)
```

Implementación de CRC: `crc8_maxim` en `src/pi_comms.cpp`.

---

## 4. Flujo de control

### 4.1 Aceleración positiva (0..100)

- El bit `DRIVE_EN` debe estar en 1.  
- `accel_i8` se mapea directamente a `accelEffective` y alimenta al cálculo de PWM (`quadThrottleUpdate`).  
- `brake_u8` (0..100) se traduce en `quadBrakeApplyPercent`: 0 % mantiene los servos en la posición de liberado y 100 % corresponde al ángulo de freno total definido en `QuadBrakeConfig`.  
- Cuando la Pi está comandando, `taskQuadDriveControl` ignora el valor de freno derivado del RC y solo respeta el porcentaje solicitado por `brake_u8`.

### 4.2 Reversa segura

1. La Pi envía `accel_i8 < 0` para solicitar reversa.  
2. El ESP32 levanta el flag `REVERSE_REQ` en su `status_flags` y bloquea `accelEffective` (queda en 0).  
3. Cuando la Pi activa físicamente el relé y responde con `ALLOW_REVERSE=1`, la ESP32 habilita el valor negativo real.  
4. Si `ALLOW_REVERSE` vuelve a 0 o pasan >120 ms sin paquetes, `accelEffective` se fuerza a 0 y se aplica freno.

### 4.3 ESTOP y failsafe

- `ESTOP` (bit 0) provoca freno inmediato y duty mínimo, ignorando cualquier otro comando. Mientras `ESTOP` permanezca activo se fuerza `brake_u8 = 100%` aunque la Pi envíe otro valor.  
- Si no llegan frames válidos en ~120 ms, `PiUartRx` marca el snapshot como viejo; `taskQuadDriveControl` cae a control RC con freno al 100 %.

### 4.4 Dirección y freno

- `steer_i8` todavía no se usa directamente; el PID continúa leyendo el receptor RC.  
- `brake_u8` (0..100) controla los servos mediante `quadBrakeApplyPercent`. Cuando no hay datos frescos de la Pi el proyecto vuelve a su modo original (freno automático derivado del throttle filtrado del RC).

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
5. **Probar ESTOP**: setear `ESTOP=1` y validar que `cmd=-100`, `brake=100` y `PiUartTx` quita `REVERSE_REQ`.  
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
