"""
Copyright (c) 2026-, David Gong, RoenPilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car.honda.values import HONDA_BOSCH

from opendbc.roenpilot.common.numpy_fast import clip

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

def compute_gas_brake(accel, speed, fingerprint):
  if fingerprint in HONDA_BOSCH:
    return compute_gb_honda_bosch(accel, speed)
  else:
    return compute_gb_honda_nidec(accel, speed)
