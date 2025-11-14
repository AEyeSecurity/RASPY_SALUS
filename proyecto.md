# 🛰️ Protocolo de Comunicación UART entre Raspberry Pi y ESP32

## 1. Objetivo

Diseñar un protocolo **rápido, eficiente y liviano** entre una **Raspberry Pi** (unidad de control) y una **ESP32** (controlador del vehículo) para transmitir comandos de movimiento y recibir estados del sistema.

### Requisitos:

* Baja carga de CPU (especialmente en la ESP32 con FreeRTOS).
* Comunicación determinista y confiable.
* Validación de integridad mediante **CRC-8**.
* Señal de **reversa** controlada desde la Raspberry (a pedido de la ESP32).

---

## 2. Capa Física

| Parámetro      | Valor / Recomendación                             |
| -------------- | ------------------------------------------------- |
| Tipo           | UART TTL 3.3 V (no 5 V)                           |
| Baudrate       | **460 800 bps** (hasta 921 600 bps si estable)    |
| Formato        | 8N1, sin control de flujo                         |
| Conexiones     | Pi TX → ESP32 RX<br>Pi RX → ESP32 TX<br>GND ↔ GND |
| Opcional       | RTS/CTS para control de flujo por hardware        |
| Cableado largo | Usar transceivers **RS-485** (MAX3485, SN75176)   |

---

## 3. Frecuencia y Temporización

| Dirección       | Frecuencia recomendada    | Comentario                    |
| --------------- | ------------------------- | ----------------------------- |
| Pi → ESP32      | 100 Hz (cada 10 ms)       | Control en tiempo real        |
| ESP32 → Pi      | 20–50 Hz o "on change"    | Telemetría y señal de reversa |
| Timeout (ESP32) | 100 ms sin paquete válido | Failsafe → neutro + freno     |

---

## 4. Estructura del Protocolo

### 4.1. Paquete **Raspberry Pi → ESP32** (6 bytes)

```
[0]  0xAA           // Start byte
[1]  ver_flags      // Alto nibble: versión (0x1) | Bajo nibble: flags
[2]  steer_i8       // -100..100 (int8)
[3]  accel_i8       // -100..100 (int8)
[4]  brake_u8       // 0..100  (uint8)
[5]  crc8           // CRC-8 Dallas/Maxim (poly 0x31, init 0x00)
```

**Flags (nibble bajo):**

| Bit | Nombre          | Descripción          |
| --- | --------------- | -------------------- |
| 0   | `ESTOP`         | Parada de emergencia |
| 1   | `DRIVE_EN`      | Habilitar tracción   |
| 2   | `ALLOW_REVERSE` | Permiso de reversa   |
| 3   | Reservado       | Futuras expansiones  |

---

### 4.2. Paquete **ESP32 → Raspberry Pi** (4 bytes)

```
[0]  0x55           // Start byte
[1]  status_flags   // Estado de la ESP32
[2]  telemetry_u8   // Valor adicional (ej. batería %, temperatura)
[3]  crc8           // CRC-8 sobre [0..2]
```

**Status Flags:**

| Bit | Nombre        | Descripción                                          |
| --- | ------------- | ---------------------------------------------------- |
| 0   | `READY`       | Sistema operativo y estable                          |
| 1   | `FAULT`       | Error crítico o de hardware                          |
| 2   | `OVERCURRENT` | Protección por sobrecorriente                        |
| 3   | `REVERSE_REQ` | Solicitud de reversa (para activar el relé en la Pi) |

---

## 5. Lógica de Reversa (con relé en la Raspberry Pi)

1. La **ESP32** detecta condición de reversa y pone `REVERSE_REQ = 1`.
2. La **Raspberry Pi** conmuta un **GPIO** que maneja el relé.
3. La **Raspberry Pi** refleja la autorización en `ALLOW_REVERSE` (flag de salida) para informar que la reversa está activa.
4. Al salir de reversa, la ESP32 limpia `REVERSE_REQ` y la Pi abre el relé.

> Si el relé está en la ESP32, simplemente omitir el control de GPIO en la Pi.

---

## 6. Manejo de Errores y Robustez

* **CRC-8** en ambos sentidos: los paquetes inválidos se descartan.
* **Timeout de recepción** (ESP32): si no llega un paquete válido en >100 ms, ejecutar **failsafe** (neutro + freno).
* **Timeout en la Pi:** si no recibe estados por >500 ms, mantener relé en estado seguro (abierto) y loggear.

---

## 7. Código de Ejemplo para Raspberry Pi (Python)

### Requisitos:

```bash
pip install pyserial RPi.GPIO
```

Deshabilitar consola serial y habilitar UART en `raspi-config`.

### Ejemplo:

```python
import time
import serial
import RPi.GPIO as GPIO

# =========================
# Configuración de hardware
# =========================
SERIAL_PORT = "/dev/ttyAMA0"  # Ajustar según modelo
BAUDRATE = 460800
TIMEOUT_S = 0.0

GPIO.setmode(GPIO.BCM)
GPIO_RELAY = 18
GPIO.setup(GPIO_RELAY, GPIO.OUT, initial=GPIO.LOW)

# =========================
# CRC-8 Dallas/Maxim
# =========================
def crc8_maxim(data: bytes, poly=0x31, init=0x00):
    c = init
    for b in data:
        c ^= b
        for _ in range(8):
            if c & 0x80:
                c = ((c << 1) ^ poly) & 0xFF
            else:
                c = (c << 1) & 0xFF
    return c

# =========================
# Flags y helpers
# =========================
VER_MAJOR = 0x1
FLAG_ESTOP = 1 << 0
FLAG_DRIVE_ENABLE = 1 << 1
FLAG_ALLOW_REVERSE = 1 << 2

STAT_READY = 1 << 0
STAT_FAULT = 1 << 1
STAT_OVERCURRENT = 1 << 2
STAT_REVERSE_REQ = 1 << 3

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def build_cmd_packet(steer, accel, brake, flags):
    steer = clamp(int(steer), -100, 100) & 0xFF
    accel = clamp(int(accel), -100, 100) & 0xFF
    brake = clamp(int(brake), 0, 100) & 0xFF

    ver_flags = ((VER_MAJOR & 0x0F) << 4) | (flags & 0x0F)
    pkt_wo_crc = bytes([0xAA, ver_flags, steer, accel, brake])
    crc = crc8_maxim(pkt_wo_crc)
    return pkt_wo_crc + bytes([crc])

def parse_status_packet(buf: bytes):
    if len(buf) < 4 or buf[0] != 0x55:
        return None
    crc = crc8_maxim(buf[:3])
    if crc != buf[3]:
        return None
    return (buf[1], buf[2])

# =========================
# Bucle principal
# =========================
def main():
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=TIMEOUT_S)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    drive_enabled = True
    allow_reverse = True
    estop = False

    target_steer = 0
    target_accel = 20
    target_brake = 0

    rx = bytearray()
    tx_period = 0.010  # 100 Hz
    next_tx = time.time()

    print("UART iniciado en", SERIAL_PORT, BAUDRATE, "baudios")

    try:
        while True:
            now = time.time()

            # --- Transmisión ---
            if now >= next_tx:
                flags = 0
                if estop: flags |= FLAG_ESTOP
                if drive_enabled: flags |= FLAG_DRIVE_ENABLE
                if allow_reverse: flags |= FLAG_ALLOW_REVERSE

                pkt = build_cmd_packet(target_steer, target_accel, target_brake, flags)
                ser.write(pkt)
                next_tx += tx_period

            # --- Recepción ---
            chunk = ser.read(32)
            if chunk:
                rx += chunk
                i = 0
                while i <= len(rx) - 4:
                    if rx[i] == 0x55:
                        candidate = rx[i:i+4]
                        parsed = parse_status_packet(candidate)
                        if parsed:
                            status_flags, telemetry = parsed
                            if status_flags & STAT_REVERSE_REQ:
                                GPIO.output(GPIO_RELAY, GPIO.HIGH)
                                allow_reverse = True
                            else:
                                GPIO.output(GPIO_RELAY, GPIO.LOW)
                            del rx[i:i+4]
                            continue
                        i += 1
                    else:
                        i += 1
                if len(rx) > 256:
                    rx = rx[-16:]

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nFinalizando...")
    finally:
        GPIO.output(GPIO_RELAY, GPIO.LOW)
        GPIO.cleanup()
        ser.close()

if __name__ == "__main__":
    main()
```

---

## 8. Estrategia de Seguridad y Failsafe

* La ESP32 debe forzar freno + neutro si no recibe paquetes en **>100 ms**.
* La Raspberry debe mantener relé abierto si no recibe estado válido por **>500 ms**.
* El CRC-8 garantiza detección de errores de un solo bit.

---

## 9. Posibles Extensiones

* Reemplazar CRC-8 por **CRC-16/CCITT** si se amplía el payload.
* Agregar campos de **diagnóstico** (tensión, temperatura, velocidad, modo).
* Adaptar el protocolo para bus **RS-485 multi-nodo**.
* Implementar versión **C/C++ (termios)** en la Pi para menor latencia.

---

**Autor:** Nicolás Leszezyñski
**Versión:** 1.0 (Noviembre 2025)
**Licencia:** MIT
