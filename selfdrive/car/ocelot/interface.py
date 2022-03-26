#!/usr/bin/env python3
from cereal import car
from selfdrive.car.ocelot.values import CAR, BUTTON_STATES
from selfdrive.car.ocelot.tunes import LatTunes, LongTunes, set_long_tune, set_lat_tune
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.config import Conversions as CV

EventName = car.CarEvent.EventName

class CarInterface(CarInterfaceBase):
  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 3.0

  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)

    self.cruise_enabled_prev = False
    self.buttonStatesPrev = BUTTON_STATES.copy()

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=[]):  # pylint: disable=dangerous-default-value
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)

    ret.carName = "ocelot"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.allOutput)]
    ret.steerActuatorDelay = 0.04
    ret.steerLimitTimer = 0.4

    if candidate == CAR.SMART_ROADSTER_COUPE:
        set_long_tune(ret.longitudinalTuning, LongTunes.SMART_PEDAL)
        set_lat_tune(ret.lateralTuning, LatTunes.SMART_PID)
        ret.wheelbase = 2.36
        ret.steerRatio = 20
        tire_stiffness_factor = 0.444
        ret.mass = 810 + STD_CARGO_KG
        ret.steerRateCost = 1.
        ret.centerToFront = ret.wheelbase * 0.44

    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

    ret.enableGasInterceptor = True
    ret.stoppingControl = True      #should these be enabled for long control
    ret.openpilotLongitudinalControl = True
    ret.minEnableSpeed = -1.



    #ret.stoppingBrakeRate = 0.16 # reach stopping target smoothly
    #ret.startingBrakeRate = 2.0 # release brakes fast
    #ret.startAccel = 1.2 # Accelerate from 0 faster

    return ret

  #returns a car.CarState
  def update(self, c, can_strings):
    buttonEvents = []
    # ******************* do can recv *******************
    self.cp.update_strings(can_strings)
    self.cp_body.update_strings(can_strings)

    ret = self.CS.update(self.cp, self.cp_body, c.enabled)

    ret.canValid = self.cp.can_valid and self.cp_body.can_valid

    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

    # events
    events = self.create_common_events(ret)
    if not ret.cruiseState.enabled:
      events.add(EventName.pcmDisable)
    # Attempt OP engagement only on rising edge of stock ACC engagement.
    elif not self.cruise_enabled_prev:
      events.add(EventName.pcmEnable)

    ret.events = events.to_msg()

    # update previous car states

    self.cruise_enabled_prev = ret.cruiseState.enabled

    self.CS.out = ret.as_reader()
    return self.CS.out

  #Pass in a car.CarControl, to be called @ 100hz
  def apply(self, c):
    ret = self.CC.update(c.enabled, c.active, self.CS, self.frame,
                               c.actuators, c.cruiseControl.cancel)
    self.frame += 1
    return ret
