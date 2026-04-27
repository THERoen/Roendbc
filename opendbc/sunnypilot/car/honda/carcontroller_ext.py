"""
Copyright (c) 2026-, David Gong, RoenPilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from math import sin

from opendbc.car import structs
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, DT_CTRL
from opendbc.car.common.pid import PIDController
from opendbc.car.honda import hondacan
from opendbc.car.honda.values import CAR, HONDA_BOSCH

from opendbc.roenpilot.car.honda.helper_gb import compute_gb_honda_bosch, compute_gb_honda_nidec, compute_gb_honda_nidec_brake_modifier
from opendbc.roenpilot.car.honda.helper_torque import quick_start_curve, driver_override_speed_factor, torque_lpf_tau
from opendbc.roenpilot.common.numpy_fast import clip, interp, rate_limit_numpy_fast

from opendbc.sunnypilot.car.honda.gas_interceptor import GasInterceptorCarController
from opendbc.sunnypilot.car.honda.values_ext import HondaFlagsSP

LongCtrlState = structs.CarControl.Actuators.LongControlState


class CarControllerExt:
  def __init__(self):
    # New longitudinal behavior is preserved for all Bosch cars and the Nidec Clarity.
    # All other Nidec platforms fall back to the legacy longitudinal behavior.
    self.use_new_long_logic = (self.CP.carFingerprint in HONDA_BOSCH) or (self.CP.carFingerprint == CAR.HONDA_CLARITY)

    # Enable steering override behavior only when the modified EPS firmware is detected.
    # Stock EPS cars rely on driver "assist" to achieve tighter curvature; forcing torque-to-zero
    # and additional filtering on stock EPS can degrade lateral performance.
    self.eps_modified = bool(self.CP_SP.flags & HondaFlagsSP.EPS_MODIFIED.value)

    self.gasfactor = 1.0
    self.gasfactor_before_maxgas = 1.0
    self.windfactor = 1.0
    self.windfactor_before_maxgas = 1.0
    self.windfactor_before_brake = 0.0
    self.pitch = 0.0
    
    self.hill_brake = 0.0
    self.speed_control = 0
    self.gas_pedal_force = 0.0

    self.torque_lpf = 0.0
    self.prev_torque_cmd = 0.0

    self.override_ramp_down_s = 0.5
    self.override_ramp_up_s = 2.0
    self.steering_pressed_prev = False
    self.override_state = "normal"
    self.override_phase_start_nanos = 0
    self.override_start_torque = 0.0

    self.driver_override_until_nanos = 0

    self.wind_brake = 0.0
    self.wind_brake_ms2 = 0.0

    # Bosch extra-brake controller
    self.brake_pid = PIDController(k_p=([0,], [0,]),
                                   k_i=([0.], [0.5]),
                                   pos_limit=0.0,
                                   neg_limit=-2.0,
                                   rate=50)
    self.brake_pid.reset()

  def update(self, CC: structs.CarControl) -> None:
    if len(CC.orientationNED) == 3:
      self.pitch = CC.orientationNED[1]
    self.hill_brake = sin(self.pitch) * ACCELERATION_DUE_TO_GRAVITY

  def accel(self, CS: structs.CarState, a):
    return 10000.0 if self.CP.carFingerprint in (CAR.ACURA_MDX_3G, CAR.ACURA_MDX_3G_MMR) and a > max(0, CS.out.aEgo) + 0.1 else a # help with lagged accel until pedal tuning is inserted

  def compute_gas_brake(self, CS: structs.CarState, a):
    if self.CP.carFingerprint in HONDA_BOSCH:
      gas, brake = compute_gb_honda_bosch(a + self.hill_brake, CS.out.vEgo)
    else:
      if self.use_new_long_logic:
        gas, brake = compute_gb_honda_nidec(a + self.hill_brake, CS.out.vEgo)
      else:
        gas, brake = compute_gb_honda_nidec_brake_modifier(a, CS.out.vEgo)
    return gas, brake

  def convert_torque_to_torque_cmd(self, CC: structs.CarControl, CS: structs.CarState, now_nanos, torque):
    # *** steer command conditioning (driver interaction + low-pass filter + rate limit) ***
    torque_cmd = torque

    if CC.latActive:
      steering_pressed = CS.out.steeringPressed

      if self.eps_modified:
        override_factor = driver_override_speed_factor(CS.out.vEgo)

        steering_rising = (not self.steering_pressed_prev) and steering_pressed
        steering_falling = self.steering_pressed_prev and (not steering_pressed)

        if steering_rising:
          self.override_state = "ramp_down"
          self.override_phase_start_nanos = now_nanos
          self.override_start_torque = float(self.torque_lpf)

        if steering_pressed:
          if self.override_state == "ramp_down":
            fade_ns = int(self.override_ramp_down_s * 1e9)
            dt_ns = now_nanos - self.override_phase_start_nanos
            x = 1.0 if fade_ns <= 0 else clip(float(dt_ns) / float(fade_ns), 0.0, 1.0)

            torque_target = self.override_start_torque * (1.0 - x * override_factor)

            self.torque_lpf = torque_target
            torque_cmd = self.torque_lpf

            if x >= 1.0:
              self.override_state = "holding"
          else:
            self.override_state = "holding"
            torque_hold = float(torque_cmd) * (1.0 - override_factor)
            self.torque_lpf = torque_hold
            torque_cmd = torque_hold

        else:
          if steering_falling:
            self.override_state = "ramp_up"
            self.override_phase_start_nanos = now_nanos

          if self.override_state == "ramp_up":
            fade_ns = int(self.override_ramp_up_s * 1e9)
            dt_ns = now_nanos - self.override_phase_start_nanos
            x = 1.0 if fade_ns <= 0 else clip(float(dt_ns) / float(fade_ns), 0.0, 1.0)

            scale = quick_start_curve(x)
            ramp_scale = (1.0 - override_factor) + (override_factor * scale)
            torque_target = float(torque_cmd) * ramp_scale

            self.torque_lpf = torque_target
            torque_cmd = self.torque_lpf

            if x >= 1.0:
              self.override_state = "normal"

          if self.override_state == "normal":
            tau = torque_lpf_tau(torque_cmd, self.prev_torque_cmd, CS.out.vEgo)
            alpha = DT_CTRL / (tau + DT_CTRL)

            if torque_cmd * self.torque_lpf < 0.0:
              self.torque_lpf = torque_cmd
            else:
              self.torque_lpf = alpha * torque_cmd + (1.0 - alpha) * self.torque_lpf

            self.prev_torque_cmd = float(torque_cmd)
            torque_cmd = self.torque_lpf

        self.steering_pressed_prev = steering_pressed

      else:
        self.torque_lpf = float(torque_cmd)
        self.prev_torque_cmd = float(torque_cmd)
        self.steering_pressed_prev = steering_pressed
        self.override_state = "normal"
        self.override_phase_start_nanos = 0
        self.override_start_torque = 0.0

    else:
      self.torque_lpf = 0.0
      self.prev_torque_cmd = 0.0
      self.last_torque = 0.0
      self.driver_override_until_nanos = 0
      self.steering_pressed_prev = False
      self.override_state = "normal"
      self.override_phase_start_nanos = 0
      self.override_start_torque = 0.0

    return torque_cmd
  
  def update_speed_control(self, CS: structs.CarState, a) -> None:
     self.speed_control = 1 if (a <= 0.0 and CS.out.vEgo == 0) else 0

  def rate_limit_3_up(self, new_value, last_value, dw_step, up_step):
    return rate_limit_numpy_fast(new_value, last_value, dw_step, 3 * up_step) if self.use_new_long_logic else rate_limit_numpy_fast(new_value, last_value, dw_step, up_step)

  def update_wind_brake(self, CS: structs.CarState) -> None:
    wind_brake = interp(CS.out.vEgo, [0.0, 2.3, 35.0], [0.001, 0.002, 0.15])
    self.wind_brake = wind_brake * self.windfactor # not in m/s2 units
    self.wind_brake_ms2 = interp(CS.out.vEgo, [0.0, 13.4, 22.4, 31.3, 40.2], [0.000, 0.049, 0.136, 0.267, 0.441]) # in m/s2 units

  def zero_speed_control(self) -> None:
    self.speed_control = 0

  def update_self_accel(self, CS: structs.CarState, a) -> None:
    if a < 0 and CS.out.vEgo > 1e-3: # MVL
      brake_addon = self.brake_pid.update(error = a - CS.out.aEgo, speed = CS.out.vEgo)
      targetaccel = min(a,a + brake_addon)
    else:
      self.brake_pid.reset()
      targetaccel = a

    self.accel = clip(targetaccel, self.params.BOSCH_ACCEL_MIN, self.params.BOSCH_ACCEL_MAX)
    self.gas_pedal_force = self.accel + self.wind_brake_ms2 * self.windfactor + self.hill_brake

  def update_self_gas(self, lcs, CS: structs.CarState, a) -> None:
    # live-learn gas pedal adjustments when openpilot is controlling gas
    if lcs == LongCtrlState.pid and not CS.out.gasPressed:
      gas_error = self.accel - CS.out.aEgo
      if self.CP.carFingerprint == CAR.ACURA_RDX_3G and CS.out.vEgo < 1e-3:
        self.gasfactor = 3.0 # max due to turbolag
      if gas_error != 0.0 and self.gas_pedal_force > 0.0:
        learn_speed = 150 if self.CP.carFingerprint == CAR.HONDA_INSIGHT else 50 # Insight gas pedal reacts too slowly
        self.gasfactor = clip(self.gasfactor + gas_error / learn_speed * self.gas_pedal_force, 0.1, 3.0)
      if gas_error != 0.0 and not CS.out.brakePressed and CS.out.vEgo > 0.0:
        wind_adjust = 1 + self.wind_brake_ms2 / 1000
        self.windfactor = clip(self.windfactor * (wind_adjust if gas_error > 0 else 1.0 / wind_adjust), 0.1, 3.0)
      if self.gas_pedal_force <= 0.0: # don't reduce windfactor while braking, allow increases
        self.windfactor = max(self.windfactor, self.windfactor_before_brake)
      else:
        self.windfactor_before_brake = self.windfactor
      if self.gas_pedal_force >= self.params.BOSCH_ACCEL_MAX: # don't increase gasfactor nor windfactor at accel max, allow decreases
        self.gasfactor = min(self.gasfactor, self.gasfactor_before_maxgas)
        self.windfactor = min(self.windfactor, self.windfactor_before_maxgas)
      else:
        self.gasfactor_before_maxgas = self.gasfactor
        self.windfactor_before_maxgas = self.windfactor
    self.gas = interp(self.gas_pedal_force * self.gasfactor, self.params.BOSCH_GAS_LOOKUP_BP, self.params.BOSCH_GAS_LOOKUP_V)

  def apply_brake(self):
    return clip(self.brake_last - self.wind_brake, 0.0, 1.0) if self.use_new_long_logic else clip(self.brake_last - (self.wind_brake if self.brake_last <= 0.95 else 0.0), 0.0, 1.0) # mike8643 Increase Nidec Braking Force

  def GasInterceptorCarController_update(self, CC: structs.CarControl, CS: structs.CarState, actuators, gas, brake):
    if self.CP_SP.enableGasInterceptor and self.use_new_long_logic:
      gas_error = actuators.accel - CS.out.aEgo
      if not CS.out.gasPressed and actuators.longControlState == LongCtrlState.pid:
        if gas_error != 0.0 and gas > 0.0:
          self.gasfactor = clip(self.gasfactor + gas_error / 50 * (gas * 4.8), 0.1, 3.0)
        if gas_error != 0.0 and not CS.out.brakePressed and CS.out.vEgo > 0.0:
          wind_adjust = 1 + (self.wind_brake * 4.8) / 1000
          self.windfactor = clip(self.windfactor * (wind_adjust if gas_error > 0 else 1.0 / wind_adjust), 0.1, 5.0)
        if gas <= 0.0: # don't reduce windfactor while braking, allow increases
          self.windfactor = max(self.windfactor, self.windfactor_before_brake)
        else:
          self.windfactor_before_brake = self.windfactor

      return GasInterceptorCarController.update(self, CC, CS, gas * self.gasfactor, brake, self.wind_brake, self.packer, self.frame)
    else:
      return GasInterceptorCarController.update(self, CC, CS, gas, brake, self.wind_brake, self.packer, self.frame)

  def create_acc_hud(self, CC: structs.CarControl, CS: structs.CarState, p_s, p_a, h_c, h_v_c):
    return hondacan.create_acc_hud(self.packer, self.CAN.pt, self.CP, CC.enabled, p_s, p_a, h_c, h_v_c, CS.is_metric, CS.acc_hud, self.speed_control)

  def create_lkas_hud(self, CC: structs.CarControl, CS: structs.CarState, h_c, a_s_r, a_t, s_a):
    reduced_steering = CS.out.steeringPressed
    steer_maxed = abs(a_t) >= self.params.STEER_MAX
    return hondacan.create_lkas_hud(self.packer, self.CAN.lkas, self.CP, h_c, CC.latActive,
                                                s_a, reduced_steering, a_s_r, CS.lkas_hud, self.dashed_lanes,
                                                steer_maxed)

  def set_new_actuator_gas_brake(self):
    if self.use_new_long_logic:
      gas = self.gasfactor
      brake = self.windfactor
    else:
      gas = self.gas
      brake = self.brake
    return gas, brake
