from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.renault.carcontroller import CarController
from opendbc.car.renault.carstate import CarState


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "renault"
    # Dashcam-only until the safety module (opendbc/safety/modes/renault.h) and
    # an active carcontroller are wired up and validated on-vehicle. The 0x134
    # LKA_CMD checksum was cracked 2026-04-23 (standard Renault CRC-8, XOR 0xEB,
    # body=bytes[17:24]) so transmit-path work is no longer cryptographically
    # blocked — just requires the usual controls-allowed wiring.
    ret.dashcamOnly = True

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.noOutput)]

    ret.steerActuatorDelay = 0.2
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    ret.steerControlType = structs.CarParams.SteerControlType.torque
    ret.radarUnavailable = True
    ret.alphaLongitudinalAvailable = False

    return ret
