# Guía para implementar el firmware UART en la ESP32

Este documento orienta la implementación del lado **ESP32** para comunicarse con la Raspberry Pi usando el protocolo definido en `proyecto.md` y ejercitado por `test_comms.py`.

---

## 1. Hardware y cableado

| Señal Pi (GPIO) | Señal ESP32 | Nota |
| --------------- | ----------- | ---- |
| GPIO14 / UART0 TX (`/dev/serial0`) | `RX0` (GPIO3) o cualquier RX disponible | Nivel lógico 3.3 V |
| GPIO15 / UART0 RX | `TX0` (GPIO1) | Ajustar según el puerto UART elegido |
| GND | GND | Referencia común obligatoria |

Opcional: expone la línea de sentido inverso (relé) a la ESP32 si necesitás leer un estado físico. Si vas a usar cableados largos, interponé transceptores RS‑485.

---

## 2. Configuración UART en la ESP32

```c
uart_config_t cfg = {
    .baud_rate = 460800,        // hasta 921600 si el enlace lo soporta
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_APB,
};
uart_param_config(UART_NUM_0, &cfg);
uart_set_pin(UART_NUM_0, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
uart_driver_install(UART_NUM_0, 256, 256, 0, NULL, 0);
```

* Recomendado: `uart_write_bytes()` con paquetes de 6 bytes y `uart_read_bytes()` en modo no bloqueante (timeout ≤ 5 ms).
* Habilita una **task de TX** que dispare cada 10 ms y una **task de RX** que procese estados entrantes.

---

## 3. Estructura de paquetes

### 3.1. Raspberry Pi → ESP32 (6 bytes)

```
0: 0xAA
1: ver_flags  (bits 7‑4 = versión mayor, bits 3‑0 = flags)
2: steer_i8   (-100..100, int8)
3: accel_i8   (-100..100, int8)
4: brake_u8   (0..100, uint8)
5: crc8       (Dallas/Maxim sobre bytes 0‑4)
```

Flags (nibble bajo):

| Bit | Nombre | Acción |
| --- | ------ | ------ |
| 0   | `ESTOP` | Parada inmediata |
| 1   | `DRIVE_EN` | Habilitar tracción |
| 2   | `ALLOW_REVERSE` | La Pi confirma que el relé está activo |
| 3   | Reservado | Mantener en 0 |

### 3.2. ESP32 → Raspberry Pi (4 bytes)

```
0: 0x55
1: status_flags
2: telemetry_u8  (ej. porcentaje de batería o código de estado)
3: crc8 (Dallas/Maxim sobre bytes 0‑2)
```

Status flags:

| Bit | Flag | Significado |
| --- | ---- | ----------- |
| 0   | `READY` | Firmware operativo |
| 1   | `FAULT` | Error crítico |
| 2   | `OVERCURRENT` | Protección activada |
| 3   | `REVERSE_REQ` | Solicitud de reversa (la Pi conmuta el relé) |

---

## 4. CRC‑8 Dallas/Maxim (C)

```c
uint8_t crc8_maxim(const uint8_t *data, size_t len) {
    uint8_t c = 0x00;
    for (size_t i = 0; i < len; ++i) {
        c ^= data[i];
        for (int b = 0; b < 8; ++b) {
            if (c & 0x80) c = (c << 1) ^ 0x31;
            else          c <<= 1;
        }
    }
    return c;
}
```

* Para TX: calcula el CRC con los primeros 5 bytes antes de enviar el paquete.
* Para RX: descarta el paquete si `crc8_maxim(frame, len-1) != frame[len-1]`.

---

## 5. Flujo recomendado en FreeRTOS

1. **Task TX (100 Hz)**  
   * Lee las consignas del control (joystick, path planner, etc.).  
   * Construye el paquete de 6 bytes.  
   * Usa `uart_write_bytes(UART_NUM_0, pkt, 6)`.  
   * Resetea un watchdog de comunicación si es necesario.

2. **Task RX (~1 kHz, no bloqueante)**  
   * Acumula bytes en un buffer circular.  
   * Busca `0xAA` para comandos y `0x55` para estados (si también escuchas eco).  
   * Procesa el paquete de 4 bytes proveniente de la Pi:  
     - Si `REVERSE_REQ` es necesario, setea la bandera.  
     - Cualquier flag `FAULT` debe loggearse y reflejarse en la UI.

3. **Lógica de reversa**  
   * Cuando la ESP32 necesita marcha atrás, setea `REVERSE_REQ` en el siguiente paquete TX.  
   * Espera a que la Pi responda con `ALLOW_REVERSE` activado antes de accionar motores.

4. **Failsafe ESP32**  
   * Si no llega un comando válido en >100 ms: `steer=0`, `accel=0`, `brake=100`.  
   * Si `ESTOP` llega activo, aplica freno inmediato y bloquea tracción hasta que desaparezca.

---

## 6. Interoperabilidad con `test_comms.py`

1. En la Raspberry Pi:
   ```bash
   python test_comms.py
   ```
   Usa el menú para ajustar steer/accel/brake y revisar el estado actual.
2. La ESP32 debe:  
   * Enviar periódicamente paquetes `[0x55 ...]` con `REVERSE_REQ` cuando necesite habilitar el relé.  
   * Procesar los paquetes `[0xAA ...]` para actualizar consignas.
3. Verifica con un analizador lógico que los CRC coincidan y que la tasa de 100 Hz se cumpla (el script usa `timeout=0.001`, por lo que no bloquea la transmisión).

---

## 7. Checklist antes de integrarse al vehículo

- [ ] UART probado en ambos sentidos a 460 800 bps.  
- [ ] CRC validado con datos aleatorios (tests de fuzz).  
- [ ] Failsafe de 100 ms implementado y probado.  
- [ ] Relé de reversa responde al `REVERSE_REQ` ↔ `ALLOW_REVERSE`.  
- [ ] Documentaste qué valor envía la ESP32 en `telemetry_u8`.  
- [ ] Logs o métricas disponibles para `FAULT` y `OVERCURRENT`.

Con estos pasos, tu firmware de la ESP32 quedará alineado con el protocolo y el tester de la Raspberry Pi, permitiendo iterar rápidamente sobre nuevas funcionalidades.
