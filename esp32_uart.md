# Guia UART ESP32 (referencia actual)

Este archivo se mantiene por compatibilidad historica.

La referencia vigente del protocolo y comportamiento real es:

- COMMS.md (estado real del firmware ESP32)
- PI_COMMS_README.md (flujo operativo y pruebas)

## Fuente de verdad

No dupliques reglas de protocolo en este archivo.

Al hacer cambios en el enlace Pi <-> ESP32, actualiza primero:

1. COMMS.md
2. PI_COMMS_README.md

## Resumen rapido vigente

- Pi -> ESP32: frame de 6 bytes con header 0xAA + CRC-8 Dallas/Maxim.
- ESP32 -> Pi: frame de 4 bytes con header 0x55 + CRC-8 Dallas/Maxim.
- Flags de comando: ESTOP, DRIVE_EN, ALLOW_REVERSE.
- Flag de estado: REVERSE_REQ reflejado por ESP32 cuando hay reversa solicitada sin permiso.
- Timeout de frescura en control del firmware ESP32: 120 ms (fallback a RC en drive/pid).

Para detalle byte a byte y comportamiento exacto, usar los dos documentos canonicos.
