#!/usr/bin/env python3
"""Simple console menu to test Raspberry Pi GPIO pins."""

import os
import shlex
import signal
import subprocess
import sys
import time

try:
    import RPi.GPIO as GPIO
except Exception as exc:
    print("Error: RPi.GPIO no disponible. Ejecuta esto en la Raspberry Pi.")
    print(f"Detalle: {exc}")
    sys.exit(1)

PINS = [16, 23, 25, 26]
BRIDGE_SCRIPT = "control_uart_bridge.py"


def setup():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    for pin in PINS:
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)


def cleanup():
    GPIO.cleanup()


def set_pin(pin, state):
    GPIO.output(pin, GPIO.HIGH if state else GPIO.LOW)


def pin_states():
    states = {}
    for pin in PINS:
        states[pin] = GPIO.input(pin)
    return states


def print_menu():
    states = pin_states()
    print("\nGPIO Test Menu")
    print("Pines disponibles:")
    for pin in PINS:
        status = "ON" if states[pin] else "OFF"
        print(f"  - GPIO {pin}: {status}")
    print("\nOpciones:")
    print("  1) Encender un pin")
    print("  2) Apagar un pin")
    print("  3) Apagar todos")
    print("  4) Salir")


def prompt_pin():
    raw = input("Pin (16, 23, 25, 26): ").strip()
    if not raw.isdigit():
        return None
    pin = int(raw)
    return pin if pin in PINS else None


def _list_bridge_processes():
    try:
        output = subprocess.check_output(["ps", "-eo", "pid,args"], text=True)
    except Exception as exc:
        print(f"No pude listar procesos: {exc}")
        return []

    processes = []
    for line in output.splitlines()[1:]:
        line = line.strip()
        if not line:
            continue
        parts = line.split(None, 1)
        if len(parts) != 2:
            continue
        pid_str, cmd = parts
        if BRIDGE_SCRIPT not in cmd or "gpio_test_menu.py" in cmd:
            continue
        try:
            pid = int(pid_str)
        except ValueError:
            continue
        processes.append({"pid": pid, "cmd": cmd})
    return processes


def _is_process_alive(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _wait_for_exit(pid, timeout_s=2.0):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if not _is_process_alive(pid):
            return True
        time.sleep(0.1)
    return False


def _stop_bridge_processes(processes):
    stopped = []
    still_running = []
    for proc in processes:
        pid = proc["pid"]
        try:
            os.kill(pid, signal.SIGTERM)
        except PermissionError:
            print(f"No tengo permisos para detener PID {pid}.")
            still_running.append(proc)
            continue
        except ProcessLookupError:
            stopped.append(proc)
            continue

        if _wait_for_exit(pid):
            stopped.append(proc)
        else:
            still_running.append(proc)

    if still_running:
        pids = ", ".join(str(p["pid"]) for p in still_running)
        print(f"No se pudo detener PID(s): {pids}.")
    return stopped, still_running


def _restart_bridge_processes(processes):
    for proc in processes:
        cmd = proc["cmd"]
        try:
            args = shlex.split(cmd)
        except ValueError as exc:
            print(f"No pude reiniciar '{cmd}': {exc}")
            continue
        try:
            subprocess.Popen(args, cwd=os.getcwd())
        except Exception as exc:
            print(f"No pude reiniciar '{cmd}': {exc}")


def maybe_stop_bridge():
    processes = _list_bridge_processes()
    if not processes:
        return []

    print("Detectado control_uart_bridge.py en ejecucion:")
    for proc in processes:
        print(f"  PID {proc['pid']}: {proc['cmd']}")
    choice = input("Detenerlo mientras pruebas GPIO? (s/n): ").strip().lower()
    if choice not in ("s", "si", "y", "yes"):
        return []

    stopped, still_running = _stop_bridge_processes(processes)
    if still_running:
        print("No se pudieron liberar los GPIO. Saliendo.")
        return []
    return stopped


def main():
    stopped_bridge = []
    gpio_ready = False
    try:
        stopped_bridge = maybe_stop_bridge()
        setup()
        gpio_ready = True
        while True:
            print_menu()
            choice = input("Selecciona una opcion: ").strip()
            if choice == "1":
                pin = prompt_pin()
                if pin is None:
                    print("Pin invalido.")
                    continue
                set_pin(pin, True)
            elif choice == "2":
                pin = prompt_pin()
                if pin is None:
                    print("Pin invalido.")
                    continue
                set_pin(pin, False)
            elif choice == "3":
                for pin in PINS:
                    set_pin(pin, False)
            elif choice == "4":
                break
            else:
                print("Opcion invalida.")
    except Exception as exc:
        print(f"Error al iniciar GPIO: {exc}")
    finally:
        if gpio_ready:
            cleanup()
        if stopped_bridge:
            _restart_bridge_processes(stopped_bridge)


if __name__ == "__main__":
    main()
