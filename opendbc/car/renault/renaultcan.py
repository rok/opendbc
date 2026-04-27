from collections.abc import Callable

from opendbc.car.crc import CRC8J1850, mk_crc8_fun

_renault_crc8 = mk_crc8_fun(CRC8J1850, init_crc=0x00, xor_out=0x00)


def _renault_588_checksum_a_xor(dat: bytes | bytearray) -> int:
  return 0x9D if (dat[5] & 0x0F) == 0x0F else 0x61


def _renault_588_checksum_b_xor(dat: bytes | bytearray) -> int:
  return 0x9D if (dat[17] & 0x0F) == 0x0F else 0x61


_XorValue = int | Callable[[bytes | bytearray], int]

# Most Renault checksums are CRC-8 poly 0x1D, init 0, over a 7-byte subframe body plus a message/subframe-specific XOR.
_RENAULT_CHECKSUM_CONFIG: dict[tuple[int, str], tuple[int, _XorValue]] = {
  (0x08C, "CHECKSUM"): (5, 0xF5),
  (0x112, "CHECKSUM_1"): (5, 0x7E),
  (0x112, "CHECKSUM_2"): (14, 0xA2),
  (0x112, "CHECKSUM_3"): (24, 0x2A),
  (0x112, "CHECKSUM_4"): (34, 0xC7),
  (0x12F, "CHECKSUM"): (5, 0x74),
  (0x134, "CHECKSUM"): (17, 0xEB),
  (0x17C, "CHECKSUM"): (5, 0x52),
  (0x1AB, "CHECKSUM_A"): (5, 0x6A),
  (0x1AB, "CHECKSUM_B"): (15, 0x17),
  (0x1AB, "CHECKSUM_C"): (27, 0xCC),
  (0x226, "CHECKSUM_A"): (5, 0x97),
  (0x226, "CHECKSUM_B"): (16, 0x08),
  (0x226, "CHECKSUM_C"): (28, 0x76),
  (0x588, "CHECKSUM_A"): (5, _renault_588_checksum_a_xor),
  (0x588, "CHECKSUM_B"): (17, _renault_588_checksum_b_xor),
}


def _resolve_xor_value(xor_value: _XorValue, dat: bytes | bytearray) -> int:
  if isinstance(xor_value, int):
    return xor_value
  return xor_value(dat)


def renault_checksum(address: int, sig, dat: bytes | bytearray) -> int:
  try:
    start, xor_value = _RENAULT_CHECKSUM_CONFIG[(address, sig.name)]
  except KeyError as e:
    raise ValueError(f"No Renault checksum config for address {hex(address)} signal {sig.name}") from e

  end = start + 7
  if len(dat) < end:
    raise ValueError(f"Message {hex(address)} too short for Renault checksum window {start}:{end}")

  xor_out = _resolve_xor_value(xor_value, dat)
  return _renault_crc8(dat[start:end]) ^ xor_out
