from dataclasses import dataclass, field
from enum import IntFlag

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, structs
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts


@dataclass
class RenaultCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))


@dataclass
class RenaultPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'renault_5_etech'})


class CAR(Platforms):
  RENAULT_5_ETECH = RenaultPlatformConfig(
    [
      RenaultCarDocs("Renault 5 E-Tech 2024-25"),
    ],
    # Published R5 E-Tech 150hp: kerb mass ~1524 kg, wheelbase 2.54 m, steering ratio ~15.0
    CarSpecs(mass=1524., wheelbase=2.54, steerRatio=15.0),
  )


# NOTE: FW_QUERY_CONFIG omitted for the initial dashcam-only port. UDS queries will
# be added once we do a tester-present sweep on the actual car.


# 0x12F.GEAR_STATE enum values (PARK_ACTIVE bit distinguishes P)
# See VAL_ 303 GEAR_STATE 1 "R" 2 "N" 3 "B" 8 "D" in renault_5_etech.dbc
GEAR_MAP = {
  0: structs.CarState.GearShifter.unknown,
  1: structs.CarState.GearShifter.reverse,
  2: structs.CarState.GearShifter.neutral,
  3: structs.CarState.GearShifter.brake,  # B = one-pedal regen (EV-specific)
  8: structs.CarState.GearShifter.drive,
}


class CarControllerParams:
  # 0x134 LKA_CMD cadence is 100 Hz on the ADAS bus. We pre-divide to 50 Hz via
  # STEER_STEP=2 for conservative jerk. These numbers satisfy ISO 11270 jerk
  # limits against the torque_data/override.toml row; retune once we have
  # engaged-driving data.
  STEER_STEP = 2
  # TORQUE_CMD is an 8-bit field at byte 10 of 0x134. Maximum observed in
  # captured engaged drives is below 128; conservative cap.
  STEER_MAX = 100
  STEER_DELTA_UP = 2            # per STEER_STEP frame
  STEER_DELTA_DOWN = 4
  STEER_DRIVER_ALLOWANCE = 150  # matches the LKAS_TORQUE_CMD 8-bit signed range
  STEER_DRIVER_MULTIPLIER = 2
  STEER_DRIVER_FACTOR = 1

  def __init__(self, CP):
    pass


class RenaultSafetyFlags(IntFlag):
  pass


DBC = CAR.create_dbc_map()
