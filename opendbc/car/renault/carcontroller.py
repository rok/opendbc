from opendbc.can.packer import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.renault.values import CarControllerParams


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.pt])

    self.apply_torque_last = 0

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends: list = []

    # This car port is DASHCAM ONLY (dashcamOnly=True in interface.py); no LKA
    # frames are actually transmitted yet. The scaffolding below mirrors the
    # MG/BYD shape so the interface passes opendbc test_car_interfaces.
    # 0x134 CHECKSUM (byte 16, CRC-8 poly 0x1D, XOR 0xEB over bytes[17:24]) is
    # handled automatically by the DBC + CANPacker once tx is enabled; the
    # follow-up work is the safety module and an active carcontroller impl.
    if self.frame % CarControllerParams.STEER_STEP == 0:
      if CC.latActive:
        new_torque = int(round(actuators.torque * CarControllerParams.STEER_MAX))
        apply_torque = apply_driver_steer_torque_limits(
          new_torque, self.apply_torque_last,
          CS.out.steeringTorque, CarControllerParams,
        )
      else:
        apply_torque = 0

      self.apply_torque_last = apply_torque
      # TODO: build the 0x134 LKA_CMD frame here (TORQUE_CMD at byte 10,
      # CHECKSUM at byte 16, COUNTER at byte 17 high nibble) and append to
      # can_sends once the safety module is in place.

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / CarControllerParams.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last

    self.frame += 1
    return new_actuators, can_sends
