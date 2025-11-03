"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import IntFlag


class HondaFlagsSP(IntFlag):
  CLARITY = 2**0
  EPS_MODIFIED = 2**1


class HondaSafetyFlagsSP:
  CLARITY = 2**0
  GAS_INTERCEPTOR = 2**1
