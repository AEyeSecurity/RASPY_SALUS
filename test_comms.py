import os
import time
import threading
import serial
from dataclasses import dataclass

try:
    import RPi.GPIO as GPIO
except ImportError:  # pragma: no cover - hardware specific
    class MockGPIO:
        LOW = 0
        HIGH = 1

        @staticmethod
        def setmode(mode):
            print(f"[GPIO] setmode({mode})")

        @staticmethod
        def setup(pin, mode, initial=0):
            print(f"[GPIO] setup(pin={pin}, mode={mode}, initial={initial})")

        @staticmethod
        def output(pin, value):
            print(f"[GPIO] Pin {pin} -> {value}")

        @staticmethod
        def cleanup():
            print("[GPIO] cleanup")

        BCM = "BCM"
        OUT = "OUT"

    GPIO = MockGPIO()

SERIAL_PORT = os.environ.get("SALUS_SERIAL_PORT", "/dev/serial0")
BAUDRATE = int(os.environ.get("SALUS_BAUDRATE", "460800"))
TIMEOUT_S = float(os.environ.get("SALUS_TIMEOUT_S", "0.001"))
GPIO_RELAY = int(os.environ.get("SALUS_GPIO_RELAY", "8"))
STATUS_TIMEOUT_S = float(os.environ.get("SALUS_STATUS_TIMEOUT_S", "0.5"))
CMD_TX_PERIOD_S = 0.010  # 10 ms
DIRECTION_CHANGE_DELAY_S = 2.0
ACCEL_RAMP_TIME_S = 0.05  # ramp-up/down mA?s rA?pido para pedal responsive

GPIO_OFF_LIST = {GPIO_RELAY, 7, 8, 16, 24, 25}
for token in os.environ.get("SALUS_GPIO_OFF_LIST", "").split(","):
    token = token.strip()
    if not token:
        continue
    try:
        GPIO_OFF_LIST.add(int(token))
    except ValueError:
        continue


def _set_all_gpio_low():
    for pin in GPIO_OFF_LIST:
        try:
            GPIO.output(pin, GPIO.LOW)
        except AttributeError:
            return


def initialize_gpio():
    try:
        GPIO.setmode(GPIO.BCM)
        for pin in GPIO_OFF_LIST:
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
    except AttributeError:
        pass
    else:
        _set_all_gpio_low()


initialize_gpio()

VER_MAJOR = 0x1
FLAG_ESTOP = 0x01
FLAG_DRIVE_ENABLE = 0x02
FLAG_ALLOW_REVERSE = 0x04

STAT_READY = 0x01
STAT_FAULT = 0x02
STAT_OVERCURRENT = 0x04
STAT_REVERSE_REQ = 0x08


def crc8_maxim(data, poly=0x31, init=0x00):
    c = init
    for b in data:
        c ^= b
        for _ in range(8):
            if c & 0x80:
                c = ((c << 1) ^ poly) & 0xFF
            else:
                c = (c << 1) & 0xFF
    return c


def clamp(value, lo, hi):
    return max(lo, min(hi, int(value)))


def to_int8_byte(value):
    v = clamp(value, -100, 100)
    return (v + 256) % 256


def build_cmd_packet(steer, accel, brake, flags):
    steer_v = to_int8_byte(steer)
    accel_v = to_int8_byte(accel)
    brake_v = clamp(brake, 0, 100) & 0xFF

    ver_flags = ((VER_MAJOR & 0x0F) << 4) | (flags & 0x0F)
    pkt_wo_crc = bytes([0xAA, ver_flags, steer_v, accel_v, brake_v])
    crc = crc8_maxim(pkt_wo_crc)
    return pkt_wo_crc + bytes([crc])


def parse_status_packet(packet):
    if len(packet) != 4 or packet[0] != 0x55:
        return None
    if crc8_maxim(packet[:3]) != packet[3]:
        return None
    status_flags = packet[1]
    telemetry = packet[2]
    return status_flags, telemetry


@dataclass
class TargetState:
    steer: int = 0
    accel: int = 0
    brake: int = 0


@dataclass
class Esp32Status:
    ready: bool = False
    fault: bool = False
    overcurrent: bool = False
    reverse_req: bool = False
    telemetry: int = 0
    last_packet_ts: float = 0.0
    packets_ok: int = 0
    crc_errors: int = 0


class CommsTester:
    def __init__(self):
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=TIMEOUT_S)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except Exception as e:
            print("No se pudo abrir el puerto serie:", e)
            raise

        # Control/flags
        self.drive_enabled = True    # logical DRIVE_EN flag to send
        self.allow_reverse = False   # logical ALLOW_REVERSE flag to send
        self.estop = False

        # Relay state (physical output) is separate from allow_reverse flag
        self.relay_state = False

        # Auto-grant behavior: if True, when ESP32 requests REVERSE the script
        # will automatically activate the relay and start sending ALLOW_REVERSE.
        # If the operator toggles ALLOW_REVERSE manually, auto-grant is disabled.
        self.auto_grant_reverse = True
        self.allow_reverse_manual = False

        self.targets = TargetState()
        self.status = Esp32Status()
        self.tx_count = 0
        self.status_timeout_logged = False

        self.rx = bytearray()
        self.tx_period = CMD_TX_PERIOD_S  # 100 Hz
        self.next_tx = time.time()

        self.last_status_ts = 0.0
        self._ramp_value = 0.0
        self._last_ramp_update = time.time()

        # Garantizamos que el rele arranque apagado (GPIO bajo).
        self._update_relay_output(False)

        self._stop_event = threading.Event()
        self._thread = None
        self._direction_block_until = 0.0
        self._last_accel_sign = 1 if self.targets.accel > 0 else (-1 if self.targets.accel < 0 else 0)

        print("UART iniciado en", SERIAL_PORT, BAUDRATE, "baudios")

    def _build_flags(self, drive_enabled=None):
        flags = 0
        if self.estop:
            flags |= FLAG_ESTOP
        use_drive = self.drive_enabled if drive_enabled is None else drive_enabled
        if use_drive:
            flags |= FLAG_DRIVE_ENABLE
        if self.allow_reverse:
            flags |= FLAG_ALLOW_REVERSE
        return flags

    def _update_relay_output(self, on: bool):
        if on != self.relay_state:
            self.relay_state = on
            try:
                GPIO.output(GPIO_RELAY, GPIO.HIGH if on else GPIO.LOW)
            except AttributeError:
                # MockGPIO prints actions
                GPIO.output(GPIO_RELAY, GPIO.HIGH if on else GPIO.LOW)

    def _handle_negative_accel_reverse(self):
        # Reversa ahora se habilita respondiendo a REVERSE_REQ de la ESP32.
        return

    def _apply_accel_ramp(self, target: int, now: float) -> int:
        """
        Rampa lineal: alcanza el target en ACCEL_RAMP_TIME_S desde el valor actual.
        Evita tirones al cambiar de signo o magnitud.
        """
        if self._last_ramp_update is None:
            self._last_ramp_update = now
        dt = max(0.0, now - self._last_ramp_update)
        self._last_ramp_update = now

        current = self._ramp_value
        diff = float(target) - current
        if abs(diff) < 0.5 or ACCEL_RAMP_TIME_S <= 0.0:
            self._ramp_value = float(target)
            return int(round(self._ramp_value))

        # Velocidad necesaria para llegar al target en el tiempo de rampa.
        rate = abs(diff) / ACCEL_RAMP_TIME_S  # unidades por segundo
        step = rate * dt
        if step > abs(diff):
            step = abs(diff)
        self._ramp_value = current + (step if diff > 0 else -step)
        # Limitar a rango válido [-100, 100]
        self._ramp_value = max(-100.0, min(100.0, self._ramp_value))
        return int(round(self._ramp_value))

    def _update_direction_change_block(self, now: float):
        accel = self.targets.accel
        if accel > 0:
            sign = 1
        elif accel < 0:
            sign = -1
        else:
            sign = 0

        if sign == 0:
            self._direction_block_until = 0.0
            self._last_accel_sign = 0
            return False

        if self._last_accel_sign == 0:
            self._last_accel_sign = sign
            return False

        if sign != self._last_accel_sign:
            self._direction_block_until = now + DIRECTION_CHANGE_DELAY_S
            self._last_accel_sign = sign
            print(
                f"[DIR] Cambio de direccion -> esperando "
                f"{DIRECTION_CHANGE_DELAY_S:.1f}s sin acelerar"
            )
            return True

        if self._direction_block_until:
            if now >= self._direction_block_until:
                print("[DIR] Cambio de direccion habilitado.")
                self._direction_block_until = 0.0
            else:
                return True

        return False

    def _apply_reverse_policy(self):
        """
        Mantiene sincronizados el relé físico y el flag ALLOW_REVERSE de acuerdo
        con el status recibido y el modo (auto o manual). La lógica responde a
        REVERSE_REQ tal como figura en COMMS.md.
        """
        reverse_req = self.status.reverse_req
        if reverse_req:
            self._update_relay_output(True)
            if self.auto_grant_reverse and not self.allow_reverse:
                print("[REVERSE] REVERSE_REQ activo -> habilitando ALLOW_REVERSE")
                self.allow_reverse = True
                self.allow_reverse_manual = False
        else:
            if self.auto_grant_reverse and not self.allow_reverse_manual and self.allow_reverse:
                print("[REVERSE] ESP32 liberó reversa -> limpiando ALLOW_REVERSE")
                self.allow_reverse = False
            if not self.allow_reverse_manual:
                self._update_relay_output(False)

    def _handle_incoming_status(self, status_flags, telemetry):
        # Track timestamp for debugging/failsafe observation
        now = time.time()
        self.last_status_ts = now
        prev_status = Esp32Status(
            ready=self.status.ready,
            fault=self.status.fault,
            overcurrent=self.status.overcurrent,
            reverse_req=self.status.reverse_req,
            telemetry=self.status.telemetry,
            last_packet_ts=self.status.last_packet_ts,
            packets_ok=self.status.packets_ok,
            crc_errors=self.status.crc_errors,
        )

        self.status.ready = bool(status_flags & STAT_READY)
        self.status.fault = bool(status_flags & STAT_FAULT)
        self.status.overcurrent = bool(status_flags & STAT_OVERCURRENT)
        self.status.reverse_req = bool(status_flags & STAT_REVERSE_REQ)
        self.status.telemetry = telemetry
        self.status.last_packet_ts = now
        self.status.packets_ok += 1
        self.status_timeout_logged = False

        if self.status.ready and not prev_status.ready:
            print("[STATUS] ESP32 READY")
        elif not self.status.ready and prev_status.ready:
            print("[STATUS] ESP32 NOT_READY")

        if self.status.fault and not prev_status.fault:
            print("[STATUS] FAULT reportado -> enviando E-Stop preventivo")
            self.estop = True
            self.drive_enabled = False
        elif not self.status.fault and prev_status.fault:
            print("[STATUS] FAULT limpiado por ESP32")

        if self.status.overcurrent and not prev_status.overcurrent:
            print("[STATUS] OVERCURRENT -> aplicando freno")
            self.estop = True
            self.targets.accel = 0
            self.targets.brake = 100
        elif not self.status.overcurrent and prev_status.overcurrent:
            print("[STATUS] OVERCURRENT liberado")

        if self.status.reverse_req != prev_status.reverse_req:
            print(f"[STATUS] REVERSE_REQ -> {self.status.reverse_req}")

        if self.status.telemetry != prev_status.telemetry:
            print(f"[STATUS] telemetry={self.status.telemetry}")

        self._apply_reverse_policy()

    def tick(self):
        now = time.time()
        throttle_blocked = self._update_direction_change_block(now)

        # Decide concesión de reversa: se habilita ALLOW cuando hay REVERSE_REQ
        # o cuando el operador está pidiendo aceleración negativa, siempre que
        # estemos en modo auto y no haya override manual.
        reverse_requested = self.status.reverse_req or (self.targets.accel < 0)
        if reverse_requested:
            if self.auto_grant_reverse and not self.allow_reverse_manual:
                if not self.allow_reverse:
                    print(
                        f"[REVERSE][AUTO] req={'ESP32' if self.status.reverse_req else 'accel<0'} "
                        f"-> ALLOW_REVERSE=ON, relay=ON"
                    )
                self.allow_reverse = True
                self._update_relay_output(True)
            elif not self.auto_grant_reverse and not self.allow_reverse_manual:
                print(
                    f"[REVERSE][AUTO-OFF] req={'ESP32' if self.status.reverse_req else 'accel<0'} "
                    "pero auto_grant_reverse=OFF; no se habilita"
                )
        else:
            # Si ya no se solicita reversa y estamos en modo auto sin override,
            # limpia ALLOW y baja el relé para evitar quedar pegados en reversa.
            if self.auto_grant_reverse and not self.allow_reverse_manual and self.allow_reverse:
                print("[REVERSE][AUTO] sin pedido -> ALLOW_REVERSE=OFF, relay=OFF")
                self.allow_reverse = False
                self._update_relay_output(False)

        if now >= self.next_tx:
            desired_accel = clamp(self.targets.accel, -100, 100)
            # Se envía el valor con signo para que la ESP32 pueda procesar reversa real.
            send_accel = desired_accel
            send_brake = clamp(self.targets.brake, 0, 100)
            send_drive_enabled = self.drive_enabled

            if self.estop:
                send_accel = 0
                send_brake = 100
                send_drive_enabled = False
                # E-Stop corta la rampa inmediatamente.
                self._ramp_value = 0.0
                self._last_ramp_update = now
            else:
                if throttle_blocked or desired_accel == 0:
                    send_accel = 0
                if not send_drive_enabled:
                    send_accel = 0
                # Freno inmediato: resetea la rampa para que el paro sea instantA?neo.
                if send_brake > 0:
                    send_accel = 0
                    self._ramp_value = 0.0
                    self._last_ramp_update = now

            # Aplicar rampa progresiva para evitar tirones mecánicos.
            send_accel = self._apply_accel_ramp(send_accel, now)

            flags = self._build_flags(drive_enabled=send_drive_enabled)

            pkt = build_cmd_packet(
                self.targets.steer,
                send_accel,
                send_brake,
                flags,
            )
            try:
                self.ser.write(pkt)
                self.tx_count += 1
            except Exception as e:
                print("Error escribiendo puerto serie:", e)
            self.next_tx += self.tx_period
            if self.next_tx < now:
                self.next_tx = now + self.tx_period

        # Read incoming bytes non-blocking (timeout small)

        try:
            chunk = self.ser.read(64)
        except Exception as e:
            print("Error leyendo puerto serie:", e)
            chunk = b""

        if chunk:
            self.rx += chunk
            i = 0
            # scan for 0x55 packets
            while i <= len(self.rx) - 4:
                if self.rx[i] == 0x55:
                    candidate = self.rx[i : i + 4]
                    parsed = parse_status_packet(candidate)
                    if parsed:
                        status_flags, telemetry = parsed
                        self._handle_incoming_status(status_flags, telemetry)
                        # remove consumed bytes
                        del self.rx[i : i + 4]
                        # continue scanning from same index (new bytes shifted)
                        continue
                    else:
                        self.status.crc_errors += 1
                        # invalid candidate (CRC o similar) -> descartar byte actual
                        del self.rx[i]
                        continue
                else:
                    i += 1

            # keep buffer bounded
            if len(self.rx) > 1024:
                self.rx = self.rx[-64:]

        self._enforce_status_timeout(now)

        # small sleep to avoid pegging CPU; keeps responsiveness
        time.sleep(0.001)

    def _run_loop(self):
        while not self._stop_event.is_set():
            self.tick()

    def start_background_loop(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        print("[LOOP] Envio continuo cada 10 ms INICIADO")

    def loop_running(self):
        return self._thread is not None and self._thread.is_alive()

    def stop_background_loop(self):
        if not (self._thread and self._thread.is_alive()):
            return
        self._stop_event.set()
        self._thread.join(timeout=1.0)
        self._thread = None
        self._stop_event.clear()
        print("[LOOP] Envio continuo DETENIDO")

    def _enforce_status_timeout(self, now: float):
        """
        Si no llegan estados válidos en STATUS_TIMEOUT_S se retorna a un estado
        seguro (relé abierto y sin ALLOW_REVERSE) tal como se describe en
        COMMS.md/PI_COMMS_README.md.
        """
        last = self.status.last_packet_ts
        if last == 0.0:
            return
        elapsed = now - last
        if elapsed > STATUS_TIMEOUT_S:
            if not self.status_timeout_logged:
                print(
                    f"[WARN] {elapsed*1000:.0f} ms sin estado de ESP32 -> "
                    "deshabilitando reversa"
                )
                self.status_timeout_logged = True
            # Estado seguro: sin drive, con freno y reversa deshabilitada.
            self.drive_enabled = False
            self.allow_reverse = False
            self.allow_reverse_manual = False
            self.auto_grant_reverse = True
            self.estop = True
            self.targets.accel = 0
            self.targets.brake = 100
            self._update_relay_output(False)

    def close(self):
        self.stop_background_loop()
        _set_all_gpio_low()
        GPIO.cleanup()
        try:
            self.ser.close()
        except Exception:
            pass

    def describe(self):
        status_bits = []
        if self.status.ready:
            status_bits.append("READY")
        if self.status.fault:
            status_bits.append("FAULT")
        if self.status.overcurrent:
            status_bits.append("OVERCURRENT")
        if self.status.reverse_req:
            status_bits.append("REVERSE_REQ")
        if not status_bits:
            status_bits.append("NONE")

        if self.status.last_packet_ts:
            age_ms = (time.time() - self.status.last_packet_ts) * 1000
            age_desc = f"{age_ms:.0f}ms"
        else:
            age_desc = "N/A"

        return (
            f"Drive={'ON' if self.drive_enabled else 'OFF'}, "
            f"ReverseFlag={'ON' if self.allow_reverse else 'OFF'}, "
            f"Relay={'ON' if self.relay_state else 'OFF'}, "
            f"AutoReverse={'ON' if self.auto_grant_reverse else 'OFF'}, "
            f"E-Stop={'ON' if self.estop else 'OFF'}, "
            f"Targets(steer={self.targets.steer}, "
            f"accel={self.targets.accel}, brake={self.targets.brake}), "
            f"ESP32(status={'+'.join(status_bits)}, "
            f"telemetry={self.status.telemetry}, age={age_desc}, "
            f"rx_ok={self.status.packets_ok}, crc_err={self.status.crc_errors}, "
            f"tx={self.tx_count})"
        )


def read_int(prompt, default=0):
    try:
        raw = input(prompt).strip()
        return int(raw) if raw else default
    except ValueError:
        print("Valor invalido, se mantiene el actual.")
        return default


def interactive_menu():
    tester = CommsTester()
    tester.start_background_loop()
    try:
        while True:
            print("\n================ ESTADO ================")
            print(tester.describe())
            print(
                f"\nEnvio continuo: {'ON' if tester.loop_running() else 'OFF'} "
                f"(periodo {CMD_TX_PERIOD_S*1000:.0f} ms)"
            )
            print(
                "\nOpciones:\n"
                "1) Ajustar target steer (-100..100)\n"
                "2) Ajustar target accel (-100..100)\n"
                "3) Ajustar target brake (0..100)\n"
                "4) Alternar Drive Enable\n"
                "5) Gestionar reversa (AUTO/MANUAL)\n"
                "6) Alternar E-Stop\n"
                "7) Pausar/Reanudar envio continuo\n"
                "8) Refrescar estado\n"
                "0) Salir\n"
            )
            choice = input("Selecciona una opcion y presiona Enter: ").strip().lower()

            if choice == "1":
                tester.targets.steer = read_int(
                    "Nuevo steer (-100..100): ", tester.targets.steer
                )
            elif choice == "2":
                tester.targets.accel = read_int(
                    "Nuevo accel (-100..100): ", tester.targets.accel
                )
            elif choice == "3":
                tester.targets.brake = read_int(
                    "Nuevo brake (0..100): ", tester.targets.brake
                )
            elif choice == "4":
                tester.drive_enabled = not tester.drive_enabled
                print("Drive enable ->", tester.drive_enabled)
            elif choice == "5":
                if tester.allow_reverse_manual or not tester.auto_grant_reverse:
                    tester.allow_reverse_manual = False
                    tester.auto_grant_reverse = True
                    tester.allow_reverse = False
                    tester._update_relay_output(False)
                    print("Allow reverse vuelve a modo AUTO")
                else:
                    # manual toggle: disable auto-grant and flip allow_reverse
                    tester.allow_reverse = not tester.allow_reverse
                    tester.allow_reverse_manual = True
                    tester.auto_grant_reverse = False
                    # physical relay should reflect manual choice
                    tester._update_relay_output(tester.allow_reverse)
                    print("Allow reverse MANUAL ->", tester.allow_reverse)
            elif choice == "6":
                tester.estop = not tester.estop
                print("E-Stop ->", tester.estop)
            elif choice == "7":
                if tester.loop_running():
                    tester.stop_background_loop()
                else:
                    tester.start_background_loop()
            elif choice == "8":
                print(tester.describe())
            elif choice == "0":
                break
            else:
                print("Opcion no valida.")

    except KeyboardInterrupt:
        print("\nFinalizando menu...")
    finally:
        tester.close()


def main():
    interactive_menu()


if __name__ == "__main__":
    main()
