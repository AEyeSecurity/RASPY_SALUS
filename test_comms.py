import os
import time
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
GPIO_RELAY = int(os.environ.get("SALUS_GPIO_RELAY", "17"))

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
    accel: int = 20
    brake: int = 0


class CommsTester:
    def __init__(self):
        self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=TIMEOUT_S)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        self.drive_enabled = True
        self.allow_reverse = True
        self.estop = False
        self.targets = TargetState()

        self.rx = bytearray()
        self.tx_period = 0.010  # 100 Hz
        self.next_tx = time.time()

        print("UART iniciado en", SERIAL_PORT, BAUDRATE, "baudios")

    def _build_flags(self):
        flags = 0
        if self.estop:
            flags |= FLAG_ESTOP
        if self.drive_enabled:
            flags |= FLAG_DRIVE_ENABLE
        if self.allow_reverse:
            flags |= FLAG_ALLOW_REVERSE
        return flags

    def tick(self):
        now = time.time()

        if now >= self.next_tx:
            pkt = build_cmd_packet(
                self.targets.steer,
                self.targets.accel,
                self.targets.brake,
                self._build_flags(),
            )
            self.ser.write(pkt)
            self.next_tx += self.tx_period

        chunk = self.ser.read(32)
        if chunk:
            self.rx += chunk
            i = 0
            while i <= len(self.rx) - 4:
                if self.rx[i] == 0x55:
                    candidate = self.rx[i : i + 4]
                    parsed = parse_status_packet(candidate)
                    if parsed:
                        status_flags, telemetry = parsed
                        if status_flags & STAT_REVERSE_REQ:
                            GPIO.output(GPIO_RELAY, GPIO.HIGH)
                            self.allow_reverse = True
                        else:
                            GPIO.output(GPIO_RELAY, GPIO.LOW)
                        del self.rx[i : i + 4]
                        continue
                    i += 1
                else:
                    i += 1
            if len(self.rx) > 256:
                self.rx = self.rx[-16:]

        time.sleep(0.001)

    def close(self):
        _set_all_gpio_low()
        GPIO.cleanup()
        self.ser.close()

    def describe(self):
        return (
            f"Drive={'ON' if self.drive_enabled else 'OFF'}, "
            f"Reverse={'ON' if self.allow_reverse else 'OFF'}, "
            f"E-Stop={'ON' if self.estop else 'OFF'}, "
            f"Targets(steer={self.targets.steer}, "
            f"accel={self.targets.accel}, brake={self.targets.brake})"
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
    try:
        while True:
            print(
                "\nMenu de test:\n"
                "1) Ejecutar ciclo continuo (Ctrl+C para parar)\n"
                "2) Ejecutar N ciclos manuales\n"
                "3) Ajustar target steer\n"
                "4) Ajustar target accel\n"
                "5) Ajustar target brake\n"
                "6) Alternar drive enable\n"
                "7) Alternar allow reverse\n"
                "8) Alternar E-Stop\n"
                "9) Mostrar estado actual\n"
                "0) Salir\n"
            )
            choice = input("Selecciona una opcion: ").strip()

            if choice == "1":
                print("Ejecutando... Ctrl+C para volver al menu.")
                try:
                    while True:
                        tester.tick()
                except KeyboardInterrupt:
                    print("\nDetenido, regresando al menu.")
            elif choice == "2":
                count = read_int("Cantidad de ciclos: ", default=0)
                for _ in range(max(count, 0)):
                    tester.tick()
                print(f"Se ejecutaron {max(count, 0)} ciclos.")
            elif choice == "3":
                tester.targets.steer = read_int("Nuevo steer: ", tester.targets.steer)
            elif choice == "4":
                tester.targets.accel = read_int("Nuevo accel: ", tester.targets.accel)
            elif choice == "5":
                tester.targets.brake = read_int("Nuevo brake: ", tester.targets.brake)
            elif choice == "6":
                tester.drive_enabled = not tester.drive_enabled
                print("Drive enable ->", tester.drive_enabled)
            elif choice == "7":
                tester.allow_reverse = not tester.allow_reverse
                print("Allow reverse ->", tester.allow_reverse)
            elif choice == "8":
                tester.estop = not tester.estop
                print("E-Stop ->", tester.estop)
            elif choice == "9":
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
