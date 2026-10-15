"""Prime LD06 in deterministic raw mode and require CRC-checked physical data."""
import os
import select
import termios
import time


def crc8(data):
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0x4d) & 255 if crc & 128 else (crc << 1) & 255
    return crc


def packets(buffer):
    count = 0
    while len(buffer) >= 47:
        offset = buffer.find(b'\x54\x2c')
        if offset < 0:
            del buffer[:-1]
            break
        if offset:
            del buffer[:offset]
        if len(buffer) < 47:
            break
        if crc8(buffer[:46]) == buffer[46]:
            count += 1
            del buffer[:47]
        else:
            del buffer[:1]
    return count


def prepare_lidar(device='/dev/ld06', timeout=15.0):
    fd = os.open(device, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        settings = termios.tcgetattr(fd)
        settings[0] = settings[1] = settings[3] = 0
        settings[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        settings[4] = settings[5] = termios.B230400
        settings[6][termios.VMIN] = settings[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, settings)
        buffer = bytearray()
        valid = received = 0
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if select.select([fd], [], [], .1)[0]:
                try:
                    chunk = os.read(fd, 8192)
                except BlockingIOError:
                    continue
                received += len(chunk)
                buffer.extend(chunk)
                valid += packets(buffer)
                if valid >= 5:
                    result = {'valid_packets': valid, 'bytes': received, 'baud': 230400}
                    print('LD06_PREFLIGHT=' + str(result), flush=True)
                    return result
        raise RuntimeError('No CRC-valid LD06 stream; no hardware stack launched')
    finally:
        # Leave this dedicated sensor UART in known raw mode, never canonical /
        # parity / hardware flow control inherited from another startup program.
        os.close(fd)
