import argparse
import os
import select
import termios
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


def read_exact(fd: int, size: int, timeout: float) -> bytes:
    deadline = time.monotonic() + timeout
    result = bytearray()
    while len(result) < size:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(
                "timed out after receiving {}/{} bytes".format(len(result), size)
            )
        readable, _, _ = select.select([fd], [], [], remaining)
        if not readable:
            continue
        chunk = os.read(fd, size - len(result))
        if not chunk:
            raise RuntimeError("serial device closed while reading")
        result.extend(chunk)
    return bytes(result)


def write_all(fd: int, data: bytes, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    offset = 0
    while offset < len(data):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("serial write timed out after {}/{} bytes".format(offset, len(data)))
        _, writable, _ = select.select([], [fd], [], remaining)
        if writable:
            offset += os.write(fd, data[offset:])


def configure_serial(fd: int, baudrate: int) -> None:
    baud_constant = getattr(termios, "B{}".format(baudrate), None)
    if baud_constant is None:
        raise ValueError("unsupported baud rate {}".format(baudrate))
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0
    attrs[4] = baud_constant
    attrs[5] = baud_constant
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)


def open_port(device: str) -> int:
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        configure_serial(fd, 115200)
    except Exception:
        os.close(fd)
        raise
    return fd


def read_status(fd: int, device_id: int, timeout: float):
    request = bytes(
        (device_id, 0x04, INPUT_REGISTER >> 8, INPUT_REGISTER & 0xFF, 0, REGISTER_COUNT)
    )
    request += crc16_modbus(request).to_bytes(2, "little")
    write_all(fd, request, timeout)
    header = read_exact(fd, 3, timeout)
    if header[0] != device_id or header[1] != 0x04:
        raise RuntimeError("unexpected FC04 header: {}".format(header.hex(" ")))
    response = header + read_exact(fd, header[2] + 2, timeout)
    received_crc = int.from_bytes(response[-2:], "little")
    expected_crc = crc16_modbus(response[:-2])
    if received_crc != expected_crc:
        raise RuntimeError("bad CRC: received 0x{:04x}, expected 0x{:04x}".format(received_crc, expected_crc))
    data = list(response[3:-2])
    if len(data) != 6:
        raise ValueError("status must be 6 bytes, got {}".format(len(data)))
    status, fault, requested, position, current = data[0], data[2], data[3], data[4], data[5]
    return status, fault, requested, position, current


def write_command(fd: int, device_id: int, timeout: float, action: int, position: int = 0, speed: int = 0, force: int = 0) -> None:
    payload = bytes((action, 0, 0, position, speed, force))
    request = bytes((device_id, 0x10, 0x03, 0xE8, 0, REGISTER_COUNT, len(payload)))
    request += payload
    request += crc16_modbus(request).to_bytes(2, "little")
    write_all(fd, request, timeout)
    response = read_exact(fd, 8, timeout)
    expected = bytes((device_id, 0x10, 0x03, 0xE8, 0, REGISTER_COUNT))
    if response[:6] != expected:
        raise RuntimeError("unexpected FC16 response: {}".format(response.hex(" ")))


def describe(status: int, fault: int, position: int, current: int) -> str:
    state = (status >> 4) & 0x03
    object_state = (status >> 6) & 0x03
    active = bool(status & 0x01)
    return (
        "gACT={} state={}({}) gOBJ={}({}) gPO={}/255 gCU={} gFLT=0x{:02X}".format(
            int(active),
            state,
            STATE_NAMES.get(state, "?"),
            object_state,
            OBJECT_NAMES.get(object_state, "?"),
            position,
            current,
            fault,
        )
    )


def wait_until(fd, device_id, timeout, predicate, phase: str) -> None:
    deadline = time.monotonic() + timeout
    while True:
        status, fault, requested, position, current = read_status(fd, device_id, 2.0)
        if predicate(status, fault, requested, position):
            return
        if time.monotonic() > deadline:
            raise TimeoutError(
                "{} timed out: {}".format(phase, describe(status, fault, position, current))
            )
        time.sleep(0.05)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/ttyACM0")
    parser.add_argument("--device-id", type=int, default=9)
    parser.add_argument("--open-speed", type=int, default=48)
    parser.add_argument("--close-speed", type=int, default=96)
    parser.add_argument("--force", type=int, default=100)
    parser.add_argument("--timeout-s", type=float, default=15.0)
    parser.add_argument("--cycles", type=int, default=1, help="close/open cycles")
    args = parser.parse_args()

    fd = open_port(args.device)
    try:
        print("[1/4] reading initial gripper state...", flush=True)
        status, fault, requested, position, current = read_status(fd, args.device_id, 2.0)
        print("  " + describe(status, fault, position, current), flush=True)
        state = (status >> 4) & 0x03
        already_active = bool(status & 0x01) and state == 3 and fault == 0
        if not already_active:
            print("[2/4] activating gripper...", flush=True)
            write_command(fd, args.device_id, 2.0, action=0)
            wait_until(
                fd, args.device_id, args.timeout_s,
                lambda s, f, r, p: not bool(s & 0x01) and ((s >> 4) & 0x03) == 0,
                "reset",
            )
            time.sleep(0.5)
            write_command(fd, args.device_id, 2.0, action=0x01)
            wait_until(
                fd, args.device_id, args.timeout_s,
                lambda s, f, r, p: bool(s & 0x01) and ((s >> 4) & 0x03) == 3 and f == 0,
                "activate",
            )
            status, fault, requested, position, current = read_status(fd, args.device_id, 2.0)
            print("  activated: " + describe(status, fault, position, current), flush=True)
        else:
            print("[2/4] gripper already active, skipping activation", flush=True)

        for cycle in range(1, args.cycles + 1):
            print("[3/4] cycle {}/{} closing gripper...".format(cycle, args.cycles), flush=True)
            write_command(
                fd, args.device_id, 2.0,
                action=0x09, position=255, speed=args.close_speed, force=args.force,
            )
            wait_until(
                fd, args.device_id, args.timeout_s,
                lambda s, f, r, p: r == 255 and ((s >> 6) & 0x03) != 0,
                "close",
            )
            status, fault, requested, position, current = read_status(fd, args.device_id, 2.0)
            print("  closed: " + describe(status, fault, position, current), flush=True)
            time.sleep(0.5)

            print("[4/4] opening gripper...", flush=True)
            write_command(
                fd, args.device_id, 2.0,
                action=0x09, position=0, speed=args.open_speed, force=args.force,
            )
            wait_until(
                fd, args.device_id, args.timeout_s,
                lambda s, f, r, p: r == 0 and ((s >> 6) & 0x03) != 0,
                "open",
            )
            status, fault, requested, position, current = read_status(fd, args.device_id, 2.0)
            print("  opened: " + describe(status, fault, position, current), flush=True)
            time.sleep(0.5)

        print("gripper test OK", flush=True)
        return 0
    finally:
        os.close(fd)


if __name__ == "__main__":
    import sys
    sys.exit(main())
