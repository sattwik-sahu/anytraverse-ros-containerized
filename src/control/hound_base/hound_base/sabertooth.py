"""Sabertooth packet-serial encoding (same protocol as the hound teleop script)."""

SET_COMMAND = 40


def command_packet(address, command, value, data=()):
    buf = bytearray([address & 0xFF, command & 0xFF, value & 0xFF,
                     (address + command + value) & 0x7F])
    if data:
        buf.extend(b & 0xFF for b in data)
        buf.append(sum(data) & 0x7F)
    return buf


def set_motor_packet(address, motor, value):
    """Drive motor 1 or 2 at value in [-2047, 2047]."""
    magnitude = min(abs(int(value)), 2047)
    data = [magnitude & 0x7F, (magnitude >> 7) & 0x7F, ord('M'), ord(str(motor))]
    return command_packet(address, SET_COMMAND, 1 if value < 0 else 0, data)
