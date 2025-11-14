#!/usr/bin/env python3
import asyncio
import json
import threading
import time

import websockets  # pip install websockets

from test_comms import (
    CommsTester,
    STAT_REVERSE_REQ,
    FLAG_ESTOP,
    FLAG_DRIVE_ENABLE,
    FLAG_ALLOW_REVERSE,
)

# =========================
#  Configuración
# =========================

WS_HOST = "0.0.0.0"
WS_PORT = 8765
FAILSAFE_S = 0.3  # si no llega nada del navegador en >300 ms -> freno

tester = CommsTester()
running = True
last_command_time = 0.0  # se actualiza cuando llega un comando web


def apply_command_from_web(data: dict):
    """
    Traduce el JSON del navegador a los targets de CommsTester.
    Espera claves: throttle (-1..1), steer (-1..1), brake (bool), gear (1/2).
    """
    global last_command_time

    try:
        throttle = float(data.get("throttle", 0.0))
    except (TypeError, ValueError):
        throttle = 0.0

    try:
        steer_norm = float(data.get("steer", 0.0))
    except (TypeError, ValueError):
        steer_norm = 0.0

    brake_bool = bool(data.get("brake", False))
    gear = int(data.get("gear", 1))

    # CamX / CamY los ignoramos por ahora (serían para mover cámara/gimbal)
    # camX = float(data.get("camX", 0.0))
    # camY = float(data.get("camY", 0.0))

    # Clamp -1..1
    throttle = max(-1.0, min(1.0, throttle))
    steer_norm = max(-1.0, min(1.0, steer_norm))

    # Mapping a rango del protocolo:
    # steer: -100..100
    steer_cmd = int(steer_norm * 100)

    # gear como "modo tortuga": en marcha 1 limitamos la aceleración
    if gear <= 1:
        accel_max = 50  # 50%
    else:
        accel_max = 100  # 100%

    accel_cmd = int(throttle * accel_max)

    if brake_bool:
        # Freno fuerte → accel 0, brake 100
        tester.targets.accel = 0
        tester.targets.brake = 100
    else:
        tester.targets.accel = accel_cmd
        tester.targets.brake = 0

    tester.targets.steer = steer_cmd

    # Aseguramos que Drive esté habilitado siempre que haya comandos
    tester.drive_enabled = True

    # La lógica fina de reversa (REVERSE_REQ / ALLOW_REVERSE)
    # se sigue manejando igual que en test_comms.py / ESP32.
    # Acá solo mandamos accel negativo cuando throttle < 0.

    print(
        f"[CMD] throttle={throttle:.2f} "
        f"-> accel={tester.targets.accel} | "
        f"steer_norm={steer_norm:.2f} -> steer={tester.targets.steer} | "
        f"brake={tester.targets.brake} | gear={gear}"
    )



    last_command_time = time.time()


def tick_loop():
    """
    Hilo de fondo que mantiene el 100 Hz de test_comms.CommsTester.tick()
    y aplica un failsafe si el navegador deja de mandar comandos.
    """
    global running, last_command_time

    print("[WS-BRIDGE] Iniciando loop de tick()")
    while running:
        now = time.time()

        # Failsafe de lado Pi: si no llega comando web hace rato, freno.
        if last_command_time > 0 and (now - last_command_time) > FAILSAFE_S:
            tester.targets.accel = 0
            tester.targets.brake = 100

        tester.tick()  # esto maneja UART + GPIO + protocolo
        # test_comms.tick() ya hace sleep(0.001), así que no dormimos acá.

async def handle_client(websocket):
    """
    Cada cliente WebSocket (tu navegador) entra por ac�.
    """
    addr = websocket.remote_address
    print(f"[WS-BRIDGE] Cliente conectado: {addr}")
    try:
        async for message in websocket:
            try:
                data = json.loads(message)
            except json.JSONDecodeError:
                print("[WS-BRIDGE] Mensaje no es JSON:", message)
                continue

            apply_command_from_web(data)

    except websockets.ConnectionClosed:
        print(f"[WS-BRIDGE] Cliente desconectado: {addr}")
    except Exception as e:
        print(f"[WS-BRIDGE] Error en handler: {e}")


async def main_async():
    # Lanzamos el hilo de tick()
    tick_thread = threading.Thread(target=tick_loop, daemon=True)
    tick_thread.start()

    # Servidor WebSocket
    async with websockets.serve(handle_client, WS_HOST, WS_PORT):
        print(f"[WS-BRIDGE] Servidor WebSocket en {WS_HOST}:{WS_PORT}")
        print("[WS-BRIDGE] Esperando conexiones...")
        await asyncio.Future()  # corre para siempre


def main():
    global running
    try:
        asyncio.run(main_async())
    except KeyboardInterrupt:
        print("\n[WS-BRIDGE] KeyboardInterrupt, cerrando...")
    finally:
        running = False
        try:
            tester.close()
        except Exception as e:
            print("[WS-BRIDGE] Error al cerrar tester:", e)


if __name__ == "__main__":
    main()
