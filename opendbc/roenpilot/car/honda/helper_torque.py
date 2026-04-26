"""
Copyright (c) 2026-, David Gong, RoenPilot, Aragon, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from math import sqrt

from opendbc.roenpilot.common.numpy_fast import clip, interp
from opendbc.car.common.conversions import Conversions as CV


def quick_start_curve(x: float) -> float:
  return sqrt(clip(x, 0.0, 1.0))


def driver_override_speed_factor(v_ego: float) -> float:
  full_cut_mph = 20.0
  no_cut_mph = 25.0

  return interp(v_ego,
                [full_cut_mph * CV.MPH_TO_MS, no_cut_mph * CV.MPH_TO_MS],
                [1.0, 0.5])


def torque_lpf_tau(torque_cmd: float, prev_torque_cmd: float, v_ego: float) -> float:
  if v_ego > 45.0 * CV.MPH_TO_MS:
    return 0.1

  torque_delta = abs(float(torque_cmd) - float(prev_torque_cmd))
  sign_change = (float(torque_cmd) * float(prev_torque_cmd)) < 0.0

  if sign_change:
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
