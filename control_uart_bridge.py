#!/usr/bin/env python3
"""
Puente WebSocket -> UART para Control.html.

- Escucha en ws://0.0.0.0:8765/controls (ajusta WS_HOST/PORT si cambia la IP).
- Recibe JSON con throttle (-1..1), steer (-1..1), brake (bool), gear (1/2).
- Traduce a CommsTester (test_comms.py) para mandar por UART al vehiculo.
- Freno de emergencia si no llega comando en INACTIVITY_BRAKE_S.
"""
import asyncio
import json
import threading
import time

import websockets  # pip install websockets

from test_comms import CommsTester

WS_HOST = "0.0.0.0"
WS_PORT = 8765
WS_PATH = "/controls"
INACTIVITY_BRAKE_S = 0.4  # si el navegador deja de enviar >400 ms -> frena

tester = CommsTester()
running = True
last_command_ts = 0.0
_tick_thread = None
_start_ts = time.time()
_command_count = 0

def _clamp(value, lo, hi, default=0.0):
    try:
        v = float(value)
    except (TypeError, ValueError):
        v = default
    return max(lo, min(hi, v))

def apply_web_command(payload):
    """
    Convierte el JSON recibido en targets para CommsTester.
    """
    global last_command_ts, _command_count

    throttle = _clamp(payload.get("throttle"), -1.0, 1.0, 0.0)
    steer_norm = _clamp(payload.get("steer"), -1.0, 1.0, 0.0)
    brake = bool(payload.get("brake", False))

    try:
        gear = int(payload.get("gear", 1))
    except (TypeError, ValueError):
        gear = 1

    accel_limit = 50 if gear <= 1 else 100
    accel_cmd = int(throttle * accel_limit)
    steer_cmd = int(steer_norm * 100)

    if brake:
        tester.targets.accel = 0
        tester.targets.brake = 100
    else:
        tester.targets.accel = accel_cmd
        tester.targets.brake = 0

    tester.targets.steer = steer_cmd
    tester.drive_enabled = True  # cada comando reactiva el drive

    _command_count += 1
    now = time.time()
    last_command_ts = now

    print(
        f"[WS] #{_command_count} +{(now - _start_ts)*1000:.0f}ms | "
        f"throttle={throttle:.2f} accel={tester.targets.accel} | "
        f"steer={tester.targets.steer} | brake={tester.targets.brake} | gear={gear}"
    )

def tick_loop():
    """
    Mantiene vivo CommsTester.tick() (100 Hz) y aplica failsafe por inactividad.
    """
    global running

    print("[WS-BRIDGE] Loop de tick iniciado")
    while running:
        now = time.time()
        if last_command_ts and (now - last_command_ts) > INACTIVITY_BRAKE_S:
            tester.targets.accel = 0
            tester.targets.brake = 100
            tester.drive_enabled = False

        tester.tick()

def _get_ws_path(websocket):
    """
    Devuelve el path del handshake segun la version de websockets.
    ServerConnection (API nueva) no tiene .path, pero expone .request.path.
    """
    path = getattr(websocket, "path", None)
    if path:
        return path
    req = getattr(websocket, "request", None)
    if req:
        return getattr(req, "path", None) or getattr(req, "target", None)
    return None

async def handle_client(websocket):
    """
    Maneja un cliente WebSocket (Control.html).
    """
    path = _get_ws_path(websocket)
    if path and path != WS_PATH:
        print(f"[WS-BRIDGE] Path inesperado: {path} (esperado {WS_PATH}) -> cerrando")
        await websocket.close(code=4000, reason="Path no soportado")
        return
    elif not path:
        print("[WS-BRIDGE] Path no disponible (API websockets nueva); aceptando cliente")

    addr = websocket.remote_address
    print(f"[WS-BRIDGE] Cliente conectado: {addr}")

    try:
        async for message in websocket:
            if isinstance(message, bytes):
                try:
                    message = message.decode("utf-8")
                except Exception:
                    continue

            try:
                payload = json.loads(message)
            except json.JSONDecodeError:
                print(f"[WS-BRIDGE] Mensaje no es JSON: {message!r}")
                continue

            if not isinstance(payload, dict):
                print(f"[WS-BRIDGE] Payload inesperado: {payload!r}")
                continue

            apply_web_command(payload)

    except websockets.ConnectionClosed:
        print(f"[WS-BRIDGE] Cliente desconectado: {addr}")
    except Exception as exc:
        print(f"[WS-BRIDGE] Error en handler: {exc}")

async def main_async():
    global _tick_thread

    _tick_thread = threading.Thread(target=tick_loop, daemon=True)
    _tick_thread.start()

    async with websockets.serve(handle_client, WS_HOST, WS_PORT):
        print(f"[WS-BRIDGE] Escuchando en ws://{WS_HOST}:{WS_PORT}{WS_PATH}")
        await asyncio.Future()  # correr hasta Ctrl+C

def main():
    global running

    try:
        asyncio.run(main_async())
    except KeyboardInterrupt:
        print("\n[WS-BRIDGE] Ctrl+C, cerrando...")
    finally:
        running = False
        try:
            if _tick_thread and _tick_thread.is_alive():
                _tick_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            tester.close()
        except Exception as exc:
            print("[WS-BRIDGE] Error al cerrar tester:", exc)

if __name__ == "__main__":
    main()
