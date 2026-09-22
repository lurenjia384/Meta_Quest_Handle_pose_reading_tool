import os
import select
import termios
import threading
import time

INPUT_REGISTER = 0x07D0
OUTPUT_REGISTER = 0x03E8
REGISTER_COUNT = 3
STATE_NAMES = {0: "reset", 1: "activating", 2: "reserved", 3: "active"}
OBJECT_NAMES = {
    0: "moving",
    1: "contact_while_opening",
    2: "contact_while_closing",
    3: "at_requested_position",
}


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def _read_exact(fd: int, size: int, timeout: float) -> bytes:
    deadline = time.monotonic() + timeout
    result = bytearray()
    while len(result) < size:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("timed out after {}/{} bytes".format(len(result), size))
        readable, _, _ = select.select([fd], [], [], remaining)
        if not readable:
            continue
        chunk = os.read(fd, size - len(result))
        if not chunk:
            raise RuntimeError("serial device closed while reading")
        result.extend(chunk)
    return bytes(result)


def _write_all(fd: int, data: bytes, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    offset = 0
    while offset < len(data):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("serial write timed out after {}/{} bytes".format(offset, len(data)))
        _, writable, _ = select.select([], [fd], [], remaining)
        if writable:
            offset += os.write(fd, data[offset:])


class RobotiqGripper:
    def __init__(self, device="/dev/ttyACM0", device_id=9, baudrate=115200,
                 open_speed=48, close_speed=96, force=100, timeout=2.0):
        self.device = device
        self.device_id = device_id
        self.baudrate = baudrate
        self.open_speed = open_speed
        self.close_speed = close_speed
        self.force = force
        self.timeout = timeout
        self._fd = None
        self._lock = threading.Lock()
        self._fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self._configure_serial()
        self._activate_if_needed()

    def _configure_serial(self):
        baud_constant = getattr(termios, "B{}".format(self.baudrate), None)
        if baud_constant is None:
            raise ValueError("unsupported baud rate {}".format(self.baudrate))
        attrs = termios.tcgetattr(self._fd)
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[3] = 0
        attrs[4] = baud_constant
        attrs[5] = baud_constant
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self._fd, termios.TCSANOW, attrs)
        termios.tcflush(self._fd, termios.TCIOFLUSH)

    def _read_status(self):
        request = bytes(
            (self.device_id, 0x04, INPUT_REGISTER >> 8, INPUT_REGISTER & 0xFF, 0, REGISTER_COUNT)
        )
        request += crc16_modbus(request).to_bytes(2, "little")
        _write_all(self._fd, request, self.timeout)
        header = _read_exact(self._fd, 3, self.timeout)
        if header[0] != self.device_id or header[1] != 0x04:
            raise RuntimeError("unexpected FC04 header: {}".format(header.hex(" ")))
        response = header + _read_exact(self._fd, header[2] + 2, self.timeout)
        received_crc = int.from_bytes(response[-2:], "little")
        expected_crc = crc16_modbus(response[:-2])
        if received_crc != expected_crc:
            raise RuntimeError("bad CRC: received 0x{:04x}, expected 0x{:04x}".format(
                received_crc, expected_crc))
        data = list(response[3:-2])
        if len(data) != 6:
            raise ValueError("status must be 6 bytes, got {}".format(len(data)))
        return data[0], data[2], data[3], data[4], data[5]

    def _write_command(self, action, position=0, speed=0, force=0):
        payload = bytes((action, 0, 0, position, speed, force))
        request = bytes((self.device_id, 0x10, 0x03, 0xE8, 0, REGISTER_COUNT, len(payload)))
        request += payload
        request += crc16_modbus(request).to_bytes(2, "little")
        _write_all(self._fd, request, self.timeout)
        response = _read_exact(self._fd, 8, self.timeout)
        expected = bytes((self.device_id, 0x10, 0x03, 0xE8, 0, REGISTER_COUNT))
        if response[:6] != expected:
            raise RuntimeError("unexpected FC16 response: {}".format(response.hex(" ")))

    def _wait_until(self, predicate, phase, timeout_s=15.0, tolerate_fault=False):
        deadline = time.monotonic() + timeout_s
        while True:
            status, fault, requested, position, current = self._read_status()
            if predicate(status, fault, requested, position):
                return
            if time.monotonic() > deadline:
                raise TimeoutError(
                    "{} timed out: gSTA={} gOBJ={} gPO={} gFLT=0x{:02X}".format(
                        phase, (status >> 4) & 0x03, (status >> 6) & 0x03, position, fault))
            time.sleep(0.05)

    def _activate_if_needed(self):
        status, fault, _requested, _position, _current = self._read_status()
        state = (status >> 4) & 0x03
        if bool(status & 0x01) and state == 3 and fault == 0:
            return
        self._write_command(action=0)
        self._wait_until(
            lambda s, f, r, p: not bool(s & 0x01) and ((s >> 4) & 0x03) == 0,
            "reset", tolerate_fault=True,
        )
        time.sleep(0.5)
        self._write_command(action=0x01)
        self._wait_until(
            lambda s, f, r, p: bool(s & 0x01) and ((s >> 4) & 0x03) == 3 and f == 0,
            "activate",
        )

    def open(self):
        self._write_command(action=0x09, position=0, speed=self.open_speed, force=self.force)

    def close(self):
        self._write_command(action=0x09, position=255, speed=self.close_speed, force=self.force)

    def states(self):
        status, fault, requested, position, current = self._read_status()
        return {
            "active": bool(status & 0x01),
            "state": (status >> 4) & 0x03,
            "object_state": (status >> 6) & 0x03,
            "fault": fault,
            "requested": requested,
            "position": position,
            "current_amps": current / 100.0,
        }

    def close_port(self):
        if self._fd is not None:
            try:
                os.close(self._fd)
            except OSError:
                pass
            self._fd = None
