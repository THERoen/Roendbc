"""
Copyright (c) 2026-, David Gong, RoenPilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from math import ceil, radians, sin, sqrt

from opendbc.car.honda.values import CAR, HONDA_BOSCH
from opendbc.car.common.pid import PIDController

from opendbc.roenpilot.car.honda.helper_gb import compute_gb_honda_bosch, compute_gb_honda_nidec
from opendbc.roenpilot.common.numpy_fast import clip, interp

from opendbc.sunnypilot.car.honda.values_ext import HondaFlagsSP

_BrakeModifier = 0.0


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
            gas, brake = compute_gb_honda_nidec(a + hill_brake, v_ego)
        else:
            gas, brake = compute_gb_honda_nidec_brake_modifier(a, v_ego)