#!/usr/bin/env python3
from cereal import car
from selfdrive.car.ocelot.values import CAR, BUTTON_STATES
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase

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
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=[], experimental_long=False):  # pylint: disable=dangerous-default-value
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)

    ret.carName = "ocelot"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.allOutput)]
    ret.steerActuatorDelay = 0.04
    ret.steerLimitTimer = 0.4

    if candidate == CAR.SMART_ROADSTER_COUPE:
      ret.longitudinalTuning.deadzoneBP = [0., 9.]
      ret.longitudinalTuning.deadzoneV = [0., .15]

      #Longitudinal Proportional values
      ret.longitudinalTuning.kpBP = [0., 5., 20., 30.]
      ret.longitudinalTuning.kpV = [0.4, 0.6, 0.75, 0.8]
      ret.longitudinalTuning.kiBP = [0., 5., 12., 20., 27.]
      ret.longitudinalTuning.kiV = [.3, .2, .1, .05, .03]



      ret.lateralTuning.init('pid')
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0., 20.0], [0., 20.0]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.05, 0.04], [0.015, 0.008]]
      ret.lateralTuning.pid.kf = 0.000013
      ret.wheelbase = 2.36
      ret.steerRatio = 20
      tire_stiffness_factor = 1.0
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


    return ret

  #returns a car.CarState
  def update(self, c, can_strings):
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