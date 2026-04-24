from opendbc.can.parser import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.renault.values import DBC, GEAR_MAP
from opendbc.car.common.conversions import Conversions as CV

GearShifter = structs.CarState.GearShifter


class CarState(CarStateBase):
  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_body = can_parsers[Bus.body]
    ret = structs.CarState()
    # Bus map (verified 2026-04-23 via cadence analysis; architecture refined 2026-04-24):
    #   bus 0 (pt):   chassis CAN — 0x08C, 0x12F, 0x17C, 0x1AB, 0x226, 0x3FA, 0x58B, 0x5EF, 0x588
    #   bus 1 (body): ITS2-FD — 0x112, 0x224, 0x4B5
    #   bus 2 (cam):  panda relay output (CAR-side of chassis pair under ALLOUTPUT;
    #                 under SILENT it's a mirror of bus 0 since the relay is closed).
    # Note: FCAM has 4 CAN pairs on its mini50 16-pin connector (D/E/F/G). Current
    # comma harness taps D and E; F/G pass through. 0x134 LKA_CMD appears on the
    # CAR side of the relay meaning either the ADAS emitter uses F/G (bypassing our
    # intercept) or EPS itself is the emitter — phase-alignment analysis on 43k
    # historical frames shows 0x17C (EPS telemetry) co-schedules with 0x134 at
    # 41% mode-fraction, and 0x1AB leads them both by 4ms, consistent with
    # "0x1AB command -> EPS processes -> 0x134+0x17C status response".

    # Wheel speeds (0x226 bus 0, 50Hz, 0.005 km/h scale) — sets vEgo from the 4-wheel mean
    self.parse_wheel_speeds(ret,
      cp.vl["WHEEL_SPEEDS_226"]["WHEEL_SPEED_FL"],
      cp.vl["WHEEL_SPEEDS_226"]["WHEEL_SPEED_FR"],
      cp.vl["WHEEL_SPEEDS_226"]["WHEEL_SPEED_RL"],
      cp.vl["WHEEL_SPEEDS_226"]["WHEEL_SPEED_RR"],
    )
    ret.standstill = ret.vEgoRaw < 0.01

    # Gas / accelerator pedal (0x08C bus 0, 100Hz, 0-100%)
    ret.gasPressed = cp.vl["ACC_PEDAL_8C"]["ACCEL_PEDAL"] > 0.1

    # Brake — analog (0x224 bus 1, 50Hz) + binary (0x112 bus 1, 100Hz)
    ret.brake = cp_body.vl["PEDAL_AND_GEAR_224"]["BRAKE_PRESSURE"] / 255.0
    ret.brakePressed = cp_body.vl["STEERING_SENSOR_112"]["BRAKE_PRESSED"] == 1

    # Steering wheel angle/rate from 0x1AB (bus 0, 100Hz; signed; scale 0.1 deg / 0.01 deg/s)
    ret.steeringAngleDeg = cp.vl["STEERING_CTRL_1AB"]["STEER_ANGLE"]
    ret.steeringRateDeg = cp.vl["STEERING_CTRL_1AB"]["STEER_RATE"]
    # Driver-applied torque: biased-unsigned 16-bit on 0x17C (bus 0, 100Hz) centered on 0
    ret.steeringTorque = cp.vl["EPS_17C"]["DRIVER_TORQUE"]
    # LKAS-commanded torque echo (8-bit signed offset -150, 0x1AB byte 7)
    ret.steeringTorqueEps = cp.vl["STEERING_CTRL_1AB"]["LKAS_TORQUE_CMD"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > 150, 5)

    # Gear — 4-bit enum with separate PARK_ACTIVE bit on 0x12F (bus 0, 50Hz)
    if cp.vl["GEAR_12F"]["PARK_ACTIVE"] == 1:
      ret.gearShifter = GearShifter.park
    else:
      ret.gearShifter = GEAR_MAP.get(int(cp.vl["GEAR_12F"]["GEAR_STATE"]), GearShifter.unknown)

    # Turn signals — R5 splits L/R across different messages and buses:
    #   LEFT  is on 0x112 (bus 1, body) byte 14 bit 0
    #   RIGHT is on 0x58B (bus 0, ADAS) byte 11 bit 1
    ret.leftBlinker = cp_body.vl["STEERING_SENSOR_112"]["TURN_SIGNAL_LEFT"] == 1
    ret.rightBlinker = cp.vl["TURN_SIGNAL_RIGHT_58B"]["TURN_SIGNAL_RIGHT"] == 1

    # ADAS engagement (0x4B5 bus 1, 10Hz; multi-state enum, non-zero == armed/active)
    adas_engaged = cp_body.vl["EPS_STATE_4B5"]["ADAS_ENGAGED"]
    ret.cruiseState.available = adas_engaged != 0
    ret.cruiseState.enabled = adas_engaged >= 44  # observed "driving-engaged" band
    ret.cruiseState.standstill = False
    # Cruise setpoint displayed on the cluster: 0x3FA bytes 10-11 (16-bit BE, scale 0.125 km/h)
    # Raw 1600 (= 200 km/h) is the UNSET sentinel when cruise has no target — map to 0.
    raw_set = cp.vl["VEHICLE_SPEED_3FA"]["CRUISE_SET_SPEED"]
    ret.cruiseState.speed = 0.0 if raw_set >= 200 else raw_set * CV.KPH_TO_MS

    # LKAS state (0x1AB byte 9 bits 1-2: 1=active, 2=suppressed, 3=unavailable)
    ret.steerFaultTemporary = cp.vl["STEERING_CTRL_1AB"]["LKAS_STATE"] == 3

    # Doors / seatbelt — 0x588 (bus 0, 10Hz); current decoding is tentative
    ret.doorOpen = cp.vl["BELTS_DOORS_588"]["DOOR_DRIVER_OPEN_CANDIDATE"] == 1
    ret.seatbeltUnlatched = cp.vl["BELTS_DOORS_588"]["BELT_UNBUCKLED_CANDIDATE"] == 1

    return ret

  @staticmethod
  def get_can_parsers(CP):
    # Three logical buses on R5:
    #   Bus.pt   = bus 0 (HS-CAN, powertrain/chassis FD)
    #   Bus.body = bus 1 (ITS2-FD, body)
    #   Bus.cam  = bus 2 (ADAS camera side; same physical traffic as bus 0 when relay closed)
    pt_messages = [
      ("ACC_PEDAL_8C", 100),
      ("GEAR_12F", 50),
      ("EPS_17C", 100),
      ("STEERING_CTRL_1AB", 100),
      ("WHEEL_SPEEDS_226", 50),
      ("VEHICLE_SPEED_3FA", 10),
      ("TURN_SIGNAL_RIGHT_58B", 10),
      ("ACC_SETPOINT_5EF", 1),
      ("BELTS_DOORS_588", 10),
    ]
    body_messages = [
      ("STEERING_SENSOR_112", 100),
      ("PEDAL_AND_GEAR_224", 50),
      ("EPS_STATE_4B5", 10),
    ]
    return {
      Bus.pt:   CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages,   0),
      Bus.body: CANParser(DBC[CP.carFingerprint][Bus.pt], body_messages, 1),
      Bus.cam:  CANParser(DBC[CP.carFingerprint][Bus.pt], [],            2),
    }
