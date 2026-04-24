"""
Copyright (c) 2026-, David Gong, RoenPilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from math import sin

from opendbc.car import ACCELERATION_DUE_TO_GRAVITY
from opendbc.car.common.pid import PIDController

from opendbc.roenpilot.common.numpy_fast import clip, interp

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

    self.gasfactor = 1.0
    self.gasfactor_before_maxgas = 1.0
    self.windfactor = 1.0
    self.windfactor_before_maxgas = 1.0
    self.windfactor_before_brake = 0.0
    self.pitch = 0.0

    # Bosch extra-brake controller
    self.brake_pid = PIDController(k_p=([0,], [0,]),
                                   k_i=([0.], [0.5]),
                                   pos_limit=0.0,
                                   neg_limit=-2.0,
                                   rate=50)
    self.brake_pid.reset()

  def update(self, CC):
    gas_pedal_force = 0.0

    if len(CC.orientationNED) == 3:
      self.pitch = CC.orientationNED[1]
    hill_brake = sin(self.pitch) * ACCELERATION_DUE_TO_GRAVITY