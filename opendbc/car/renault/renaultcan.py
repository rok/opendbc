# Renault CRC-8, identical to GWM Haval H6 (poly 0x1D, init 0, per-message XOR constant).
# Validated at 100.0000% across 19M+ frames / 173 segments / 20 active drives.
# Data covers the 7 bytes immediately AFTER the CRC byte.

POLY = 0x1D

# Per-message XOR constants. For multi-subframe messages each subframe has its own
# CRC byte and its own XOR constant.
# (msg_addr, subframe_index) -> xor_out
XOR_CONSTANTS = {
  (0x08C, 0): 0xF5, (0x12F, 0): 0x74, (0x17C, 0): 0x52, (0x44C, 0): 0x08,
  (0x453, 0): 0xD3, (0x5CF, 0): 0xEB, (0x455, 0): 0x0C, (0x534, 0): 0x47,
  (0x5E4, 0): 0x3C, (0x434, 0): 0x22, (0x546, 0): 0x2D,
  (0x1AB, 0): 0x6A, (0x1AB, 1): 0x17, (0x1AB, 2): 0xCC,
  (0x226, 0): 0x97, (0x226, 1): 0x08, (0x226, 2): 0x76,
  (0x112, 0): 0x7E, (0x112, 1): 0xA2, (0x112, 2): 0x2A, (0x112, 3): 0xC7,
}


def renault_checksum(data_7_bytes: bytes, xor_out: int) -> int:
  """CRC-8 poly 0x1D init 0 with final XOR. Takes the 7 data bytes immediately
  after the CRC byte (e.g. frame[5:12] when CRC is at byte 4)."""
  crc = 0
  for b in data_7_bytes:
    crc ^= b
    for _ in range(8):
      crc = ((crc << 1) ^ POLY) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
  return crc ^ xor_out
