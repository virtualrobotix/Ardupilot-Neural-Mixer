# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""Framing of the sketch packets on the hop between the relay and ArduPilot.

The packets of sketch/protocol.h start with a four-byte magic that fixes their length and carry
no checksum: inside the Arduino Bridge they travel as one MessagePack value each. ArduPilot reads
them from a UDP "serial" port, which is a byte stream, so on that hop every packet is followed by
a CRC16 (CCITT, initial value 0, little endian). AP_JointBus_XDuck.cpp is the other end.
"""
from __future__ import annotations

import binascii

# MCU -> host
STATE, DIAGNOSTIC, VOLTAGE, CAL_REPLY, RAW, SERVO_REPLY = b'DQS1', b'DQD1', b'DQV1', b'DQA1', b'DQR2', b'DQS3'
# host -> MCU
COMMAND, CAL, SERVO = b'DQC1', b'DQL2', b'DQV2'

LENGTH = {
    STATE: 240, DIAGNOSTIC: 68, VOLTAGE: 29, CAL_REPLY: 10, RAW: 38, SERVO_REPLY: 14,
    COMMAND: 80, CAL: 10, SERVO: 12,
}

# Arduino Bridge RPC name of each packet (sketch.ino)
RPC_FROM_MCU = {
    'duck_state': STATE, 'duck_diagnostics': DIAGNOSTIC, 'duck_voltage': VOLTAGE,
    'duck_cal_reply': CAL_REPLY, 'duck_raw': RAW, 'duck_servo_reply': SERVO_REPLY,
}
RPC_TO_MCU = {COMMAND: 'duck_command', CAL: 'duck_cal', SERVO: 'duck_servo'}


def crc16(data: bytes) -> int:
    return binascii.crc_hqx(data, 0)


def frame(packet: bytes) -> bytes:
    """Packet followed by its CRC16."""
    if LENGTH.get(bytes(packet[:4])) != len(packet):
        raise ValueError('unknown packet or wrong length')
    return bytes(packet) + crc16(packet).to_bytes(2, 'little')


class Deframer:
    """Byte stream in, checked packets out. Bytes that do not form a packet are skipped."""

    def __init__(self, accept=None):
        self.accept = None if accept is None else set(accept)
        self.buffer = bytearray()
        self.crc_errors = 0

    def feed(self, data: bytes) -> list[bytes]:
        self.buffer += data
        out = []
        buf = self.buffer
        pos = 0
        while len(buf) - pos >= 4:
            magic = bytes(buf[pos:pos + 4])
            length = LENGTH.get(magic)
            if length is None or (self.accept is not None and magic not in self.accept):
                pos += 1
                continue
            if len(buf) - pos < length + 2:
                break
            packet = bytes(buf[pos:pos + length])
            if crc16(packet) != int.from_bytes(buf[pos + length:pos + length + 2], 'little'):
                self.crc_errors += 1
                pos += 1
                continue
            out.append(packet)
            pos += length + 2
        del buf[:pos]
        if len(buf) > 4096:     # a stream with no packet in it
            del buf[:-3]
        return out
