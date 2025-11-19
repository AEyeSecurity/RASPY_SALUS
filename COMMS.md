# 📘 Guía para implementar el firmware UART en la ESP32

**(Versión refinada con mejoras y aclaraciones)**

Este documento orienta la implementación del lado **ESP32** para comunicarse con la Raspberry Pi usando el protocolo definido en `protocolo_uart.md` y probado mediante `test_comms.py`.

---

# 1. Hardware y cableado

| Señal Raspberry Pi                 | Señal ESP32                                          | Nota                         |
| ---------------------------------- | ---------------------------------------------------- | ---------------------------- |
| GPIO14 / UART0 TX (`/dev/serial0`) | `RX0` (GPIO3) o cualquier `RX` de un UART disponible | Nivel 3.3 V                  |
| GPIO15 / UART0 RX                  | `TX0` (GPIO1)                                        | Ajustar si usás UART1/2      |
| GND                                | GND                                                  | Obligatorio referencia común |

> **Nota:** Para cableados largos (>50 cm) o zonas con mucho ruido (motores, ESC, drivers), conviene agregar transceptores **RS-485** (MAX3485 / SN75176).

---

# 2. Configuración del UART en la ESP32

```c
uart_config_t cfg = {
    .baud_rate = 460800,        // Se recomienda este valor como base
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_APB,
};
uart_param_config(UART_NUM_0, &cfg);

// Reemplazar TX_PIN y RX_PIN según el hardware usado
uart_set_pin(UART_NUM_0, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

// Instalar driver con buffers moderados
uart_driver_install(UART_NUM_0, 256, 256, 0, NULL, 0);
```

### 💡 Notas importantes sobre baudios

* **460 800** → nivel recomendado inicial (estable en casi todos los casos).
* **921 600** → funciona si el cable es corto y limpio.
* **1–2 Mbps** → posible si ambos lados lo soportan, pero requiere pruebas adicionales.

---

# 3. Estructuras de paquetes

## 3.1. Raspberry Pi → ESP32 (6 bytes)

```
0: 0xAA              // Start byte
1: ver_flags         // Bits 7–4: versión | Bits 3–0: flags
2: steer_i8          // -100..100 (int8)
3: accel_i8          // -100..100 (int8)
4: brake_u8          // 0..100 (uint8)
5: crc8              // CRC-8 Dallas/Maxim sobre bytes 0–4
```

### Flags de `ver_flags` (nibble bajo):

| Bit | Nombre          | Acción                                 |
| --- | --------------- | -------------------------------------- |
| 0   | `ESTOP`         | Freno inmediato                        |
| 1   | `DRIVE_EN`      | Habilita tracción                      |
| 2   | `ALLOW_REVERSE` | La Pi confirma que el relé está activo |
| 3   | Reservado       | Mantener en 0                          |

---

## 3.2. ESP32 → Raspberry Pi (4 bytes)

```
0: 0x55
1: status_flags
2: telemetry_u8
3: crc8        // CRC-8 Dallas/Maxim sobre bytes 0–2
```

### Status Flags:

| Bit | Flag          | Significado                    |
| --- | ------------- | ------------------------------ |
| 0   | `READY`       | Firmware operativo             |
| 1   | `FAULT`       | Error crítico                  |
| 2   | `OVERCURRENT` | Protección por sobrecorriente  |
| 3   | `REVERSE_REQ` | ESP32 solicita activar el relé |

---

# 4. CRC-8 Dallas/Maxim (Implementación en C)

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

* En **RX**, descartar si `crc != frame[len-1]`.
* En **TX**, calcula el CRC justo antes del envío.

---

# 5. Flujo recomendado en FreeRTOS

## 5.1. Task de recepción (RX) — ~1 kHz, NO bloqueante

🔹 *IMPORTANTE:*
La ESP32 **solo recibe paquetes con header `0xAA`**.
El header `0x55` **solo aparece en sentido ESP32 → Raspberry Pi**, por lo que la ESP32 **no debe procesarlo**.

### Flujo sugerido:

1. Leer datos con `uart_read_bytes()` (timeout ≈ 1–5 ms).
2. Acumular en buffer circular.
3. Buscar `0xAA` como primer byte válido.
4. Verificar que haya 6 bytes disponibles.
5. Verificar CRC.
6. Actualizar:

   * `steer`
   * `accel`
   * `brake`
   * flags recibidos
7. Actualizar timestamp de “último paquete válido”.

---

## 5.2. Task de transmisión (TX) — 100 Hz

Cada 10 ms:

1. Armar paquete `[0x55 status telemetry crc]`.
2. Completar `status_flags`:

   * `READY`, `FAULT`, `OVERCURRENT`
   * `REVERSE_REQ` si corresponde
3. Enviar 4 bytes con `uart_write_bytes(UART_NUM_0, pkt, 4)`.

### Nota sobre `telemetry_u8`

Se recomienda estandarizar:

* 0–100 → Porcentaje de batería
* 101 → Temperatura alta
* 102 → Error de sensor
* 103 → Error de encoder
* 255 → “Valor no disponible”

---

# 6. Failsafe de la ESP32

1. Medir tiempo con `xTaskGetTickCount()` (no usar `esp_timer_get_time()` en este caso).
2. Si pasan **>100 ms** sin un paquete válido:

   * `steer = 0`
   * `accel = 0`
   * `brake = 100`
   * Deshabilitar tracción (`DRIVE_EN = 0`)
3. Si llega `ESTOP`, aplicar freno inmediato sin esperar timeout.

---

# 7. Lógica de reversa

## Solicitud desde la ESP32

* La ESP32 activa el bit `REVERSE_REQ` en su paquete TX.
* La Raspberry Pi responde activando el relé físico.
* La Pi luego incluye `ALLOW_REVERSE` en `ver_flags` del paquete hacia la ESP32.

## Comportamiento recomendado:

* La ESP32 **NO debe accionar motores en reversa** hasta ver `ALLOW_REVERSE = 1`.
* Si la Pi quita `ALLOW_REVERSE`, cortar reversa inmediatamente.

---

# 8. Interoperabilidad con `test_comms.py`

El script del lado Raspberry Pi:

* Envía paquetes a 100 Hz con el formato correcto.
* Valida CRC de los paquetes ESP32 → Pi.
* Muestra estado, controla el relé y permite ajustar steer/accel/brake en vivo.

**Recomendación:**
Antes de probar en el robot, usar el script en modo ciclo continuo y verificar:

* No hay paquetes inválidos.
* No hay timeouts.
* Reversa funciona (`REVERSE_REQ` ↔ relé ↔ `ALLOW_REVERSE`).

---

# 9. Checklist previo a integrar en el vehículo

* [ ] UART probado en ambos sentidos a 460800 bps.
* [ ] CRC validado con paquetes aleatorios (fuzz test).
* [ ] “Gap” entre paquetes Pi → ESP32 ≤ 15 ms.
* [ ] `failsafe` actúa en <120 ms sin un paquete válido.
* [ ] Reversa bloqueada hasta recibir `ALLOW_REVERSE`.
* [ ] `telemetry_u8` definido y documentado.
* [ ] Flags de fault probados manual
