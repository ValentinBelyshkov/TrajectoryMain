"""
MSP v2 Encoder for MSP_SET_RAW_GPS (ID 201)
Injects GPS data into flight controller.
Payload: 14 bytes exactly.
"""
import struct

class MspEncoder:
    """
    MSP v1 Encoder for MSP_SET_RAW_GPS (ID 201)
    Protocol: $M< (not $X< which is v2!)
    Payload: 14 bytes exactly.
    """
    HEADER = b'$M<'      # MSP v1 (NOT $X< which is v2!)
    MSG_ID = 201         # MSP_SET_RAW_GPS

    def _crc8(self, data: bytes) -> int:
        """MSP v1 CRC: простой XOR всех байтов"""
        crc = 0
        for b in data:
            crc ^= b
        return crc & 0xFF

    def encode(self, data: dict) -> bytes:
        # Масштабирование координат
        lat = int(data['lat'] * 10_000_000)
        lon = int(data['lon'] * 10_000_000)
        alt = int(data['alt'] * 100)  # метры → сантиметры
        
        fix = 3 if data['fix'] >= 2 else 0
        sat = data.get('sat', 12)
        speed = data.get('speed', 0)

        # Строго 14 байт: <BBiiHH
        payload = struct.pack('<BBiiHH', fix, sat, lat, lon, alt, speed)

        if len(payload) != 14:
            raise ValueError(f"MSP payload must be 14 bytes, got {len(payload)}")

        # MSP v1 формат:
        # Header (3) + Size (1) + ID (1) + Payload (14) + CRC (1) = 20 байт
        size_byte = struct.pack('B', len(payload))
        id_byte = struct.pack('B', self.MSG_ID)
        crc = self._crc8(size_byte + id_byte + payload)

        return self.HEADER + size_byte + id_byte + payload + struct.pack('B', crc)
