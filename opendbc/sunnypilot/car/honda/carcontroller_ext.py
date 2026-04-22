"""
Copyright (c) 2026-, David Gong, RoenPilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from math import sqrt

from opendbc.car.honda.carcontroller import compute_gb_honda_bosch
from opendbc.car.honda.values import CAR, HONDA_BOSCH
from opendbc.car.common.pid import PIDController

from opendbc.roenpilot.common.numpy_fast import clip, interp

from opendbc.sunnypilot.car.honda.values_ext import HondaFlagsSP

_BrakeModifier = 0.0


def compute_gb_honda_bosch(accel, speed):
  # TODO returns 0s, is unused
  return 0.0, 0.0


def compute_gb_honda_nidec(accel, speed):
  creep_brake = 0.0
  creep_speed = 2.3
  creep_brake_value = 0.15
  if speed < creep_speed:
    creep_brake = (creep_speed - speed) / creep_speed * creep_brake_value
  gb = float(accel) / 4.8 - creep_brake
  return clip(gb, 0.0, 1.0), clip(-gb, 0.0, 1.0)


def compute_gb_honda_nidec_brake_modifier(accel, speed):
  global _BrakeModifier
  if accel < -3.9:
    _BrakeModifier += 0.01
  else:
    _BrakeModifier = 0.0
  creep_brake = 0.0
  creep_speed = 2.3
  creep_brake_value = 0.15
  if speed < creep_speed:
    creep_brake = (creep_speed - speed) / creep_speed * creep_brake_value
  gb = float(accel) / interp(float(accel), [4.0, 3.5], [4.0, 4.8]) - creep_brake
  just_brake = float(accel) / (-4.8 + _BrakeModifier) + creep_brake
  return clip(gb, 0.0, 1.0), clip(just_brake, 0.0, 1.0)


def compute_gas_brake(accel, speed, fingerprint):
  if fingerprint in HONDA_BOSCH:
    return compute_gb_honda_bosch(accel, speed)
  else:
    return compute_gb_honda_nidec_brake_modifier(accel, speed)

class CarControllerExt:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP

    # New longitudinal behavior is preserved for all Bosch cars and the Nidec Clarity.
    # All other Nidec platforms fall back to the legacy longitudinal behavior.
    self.use_new_long_logic = (CP.carFingerprint in HONDA_BOSCH) or (CP.carFingerprint == CAR.HONDA_CLARITY)

    # Enable steering override behavior only when the modified EPS firmware is detected.
    # Stock EPS cars rely on driver "assist" to achieve tighter curvature; forcing torque-to-zero
    # and additional filtering on stock EPS can degrade lateral performance.
    self.eps_modified = bool(getattr(CP_SP, "flags", 0) & HondaFlagsSP.EPS_MODIFIED.value)

    self.gasfactor = 1.0
    self.gasfactor_before_maxgas = 1.0
    self.windfactor = 1.0
    self.windfactor_before_maxgas = 1.0
    self.windfactor_before_brake = 0.0
    self.pitch = 0.0

    self.torque_lpf = 0.0
    self.prev_torque_cmd = 0.0

    self.override_ramp_down_s = 0.5
    self.override_ramp_up_s = 2.0
    self.steering_pressed_prev = False
    self.override_state = "normal"
    self.override_phase_start_nanos = 0
    self.override_start_torque = 0.0

    self.driver_override_until_nanos = 0

    # Bosch extra-brake controller
    self.brake_pid = PIDController(k_p=([0,], [0,]),
                                   k_i=([0.], [0.5]),
                                   pos_limit=0.0,
                                   neg_limit=-2.0,
                                   rate=50)
    self.brake_pid.reset()

  def accel(self, a, a_ego):
    return 10000.0 if (self.CP.carFingerprint in (CAR.ACURA_MDX_3G, CAR.ACURA_MDX_3G_MMR)) and (a > max(0, a_ego) + 0.1) else a
  
  def compute_gas_brake(self, a, v_ego):
    if self.CP.carFingerprint in HONDA_BOSCH:
        gas, brake = compute_gb_honda_bosch(a + hill_brake, v_ego)
    else:
        if self.use_new_long_logic:
            gas, brake = compute_gb_honda_nidec_new(a + hill_brake, v_ego)
        else:
            gas, brake = compute_gb_honda_nidec_legacy(a, v_ego)

  #def update(self, ret: structs.CarState, can_parsers: dict[StrEnum, CANParser]) -> None:
   # cp = can_parsers[Bus.pt]
   # cp_cam = can_parsers[Bus.cam]

   # if self.CP_SP.flags & HondaFlagsSP.NIDEC_HYBRID:
   #   ret.accFaulted = bool(cp.vl["HYBRID_BRAKE_ERROR"]["BRAKE_ERROR_1"] or cp.vl["HYBRID_BRAKE_ERROR"]["BRAKE_ERROR_2"])
   #   ret.stockAeb = bool(cp_cam.vl["BRAKE_COMMAND"]["AEB_REQ_1"] and cp_cam.vl["BRAKE_COMMAND"]["COMPUTER_BRAKE_HYBRID"] > 1e-5)

   # if self.CP_SP.flags & HondaFlagsSP.HYBRID_ALT_BRAKEHOLD:
   #   ret.brakeHoldActive = cp.vl["BRAKE_HOLD_HYBRID_ALT"]["BRAKE_HOLD_ACTIVE"] == 1

   # if self.CP_SP.enableGasInterceptor:
   #   # Same threshold as panda, equivalent to 1e-5 with previous DBC scaling
   #   gas = (cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS"] + cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS2"]) // 2
   #   ret.gasPressed = gas > 512 if self.CP.carFingerprint in HONDA_NIDEC_PEDAL_DEADZONE else gas > 492

def _clamp01(x: float) -> float:
  return 0.0 if x <= 0.0 else (1.0 if x >= 1.0 else x)


def _quick_start_curve(x: float) -> float:
  return sqrt(_clamp01(x))


def _driver_override_speed_factor(v_ego: float) -> float:
  full_cut_mph = 20.0
  no_cut_mph = 25.0

  return interp(v_ego,
                         [full_cut_mph * CV.MPH_TO_MS, no_cut_mph * CV.MPH_TO_MS],
                         [1.0, 0.5])


def _torque_lpf_tau(torque_cmd: float, prev_torque_cmd: float, v_ego: float) -> float:
  if v_ego > 45.0 * CV.MPH_TO_MS: # Speed at which low-pass filter becomes static value listed below:
    return 0.1

  torque_delta = abs(float(torque_cmd) - float(prev_torque_cmd))
  sign_change = (float(torque_cmd) * float(prev_torque_cmd)) < 0.0

  if sign_change:
    # Unwinding from turn: prioritize fast response to reduce lag
    if torque_delta > 0.15:
      return 0.000
    elif torque_delta > 0.05:
      return 0.050
    else:
      return 0.100

  if torque_delta > 0.50:
    return 0.020
  elif torque_delta > 0.20:
    return 0.050
  elif torque_delta > 0.05:
    return 0.075
  else:
    return 0.100