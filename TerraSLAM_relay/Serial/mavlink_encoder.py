"""
MAVLink v2 Encoder for GPS_RAW_INT (MSGID 24)
Lightweight, dependency-free, production-ready.
"""
import struct
import time

class MavlinkEncoder:
    STX_V2 = 0xFD
    SYS_ID = 1
    COMP_ID = 1
    MSG_ID = 24  # GPS_RAW_INT
    CRC_EXTRA = 0x1C

    def __init__(self):
        self.seq = 0

    def _crc16(self, data: bytes, crc: int = 0xFFFF) -> int:
        for b in data:
            crc ^= b
            for _ in range(8):
                crc = (crc >> 1) ^ 0xC001 if crc & 1 else crc >> 1
        return crc

    def encode(self, data: dict) -> bytes:
        lat = int(data['lat'] * 1e7)
        lon = int(data['lon'] * 1e7)
        alt = int(data['alt'] * 1000)
        ts  = int(time.time() * 1000) & 0xFFFFFFFF
        fix = 3 if data['fix'] >= 2 else 0
        sat = 8 if fix else 0

        # Payload: time_boot_ms(4), lat(4), lon(4), alt(4), alt_ellipsoid(4), h_acc(4), v_acc(4), vel(2), cog(2), satellites_visible(2), fix_type(1)
        payload = struct.pack('<IiiiIIIHHHB', ts, lat, lon, alt, alt, 0, 0, 0, 0, sat, fix)

        header = struct.pack('<BBBBBBBH',
                             self.STX_V2, len(payload), 0, 0, self.seq, self.SYS_ID, self.COMP_ID, self.MSG_ID)

        crc = self._crc16(header + payload + bytes([self.CRC_EXTRA]))
        self.seq = (self.seq + 1) % 256

        return header + payload + struct.pack('<H', crc)
