# Protocolo UART Pi <-> ESP32 (nota de mantenimiento)

Este documento quedo como nota de proyecto.
La especificacion actual y verificada del enlace esta en:

- COMMS.md
- PI_COMMS_README.md

## Politica de documentacion

Para evitar inconsistencias:

- No mantener especificaciones paralelas aqui.
- Cualquier cambio de protocolo (frames, flags, CRC, timeout, reversa) se documenta solo en los dos archivos canonicos.

## Referencias de implementacion

- Lado Pi: test_comms.py, control_uart_bridge.py
- Lado ESP32: src/pi_comms.cpp, src/quad_functions.cpp, src/pid.cpp

## Verificacion recomendada

1. Confirmar CRC y headers (0xAA / 0x55).
2. Verificar REVERSE_REQ <-> ALLOW_REVERSE.
3. Probar ESTOP y timeout de enlace.
4. Validar comms.status en ESP32 y estado RX/TX en Pi.

Si hay diferencias entre este archivo y COMMS.md, considerar correcto COMMS.md.
