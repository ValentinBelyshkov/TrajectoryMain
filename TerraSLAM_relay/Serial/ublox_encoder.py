"""
U-Blox UBX Encoder for NAV-POSLLH (Class 0x01, ID 0x02)
Standard binary protocol used in GNSS modules.
"""
import struct
import time

class UbloxEncoder:
    SYNC1 = 0xB5
    SYNC2 = 0x62
    CLASS = 0x01
    ID    = 0x02  # NAV-POSLLH

    def _checksum(self, data: bytes) -> tuple:
        ck_a = ck_b = 0
        for b in data:
            ck_a = (ck_a + b) % 256
            ck_b = (ck_b + ck_a) % 256
        return ck_a, ck_b

    def encode(self, data: dict) -> bytes:
        lat  = int(data['lat'] * 1e7)
        lon  = int(data['lon'] * 1e7)
        alt  = int(data['alt'] * 1000)  # mm
        itow = int(time.time() * 1000) & 0xFFFFFFFF

        # Payload: iTOW(4), lon(4), lat(4), height(4), hMSL(4), hAcc(4), vAcc(4)
        payload = struct.pack('<IiiIIII', itow, lon, lat, alt, alt, 0, 0)

        header = struct.pack('<BBH', self.CLASS, self.ID, len(payload))
        ck_a, ck_b = self._checksum(header + payload)

        return bytes([self.SYNC1, self.SYNC2]) + header + payload + struct.pack('BB', ck_a, ck_b)
