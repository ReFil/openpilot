from cereal import car
from common.numpy_fast import clip, interp
from selfdrive.config import Conversions as CV
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.ocelot.ocelotcan import create_steer_command, create_ibst_command, \
                                           create_pedal_command, create_msg_command
from selfdrive.car.ocelot.values import SteerLimitParams
from opendbc.can.packer import CANPacker

VisualAlert = car.CarControl.HUDControl.VisualAlert

# Accel limits

PEDAL_SCALE = 2
BRAKE_SCALE = 2



class CarController():
  def __init__(self, dbc_name, CP, VM):

    self.last_steer = 0
    self.alert_active = False
    self.last_standstill = False

    self.last_fault_frame = -200
    self.steer_rate_limited = False

    self.packer = CANPacker(dbc_name)
    self.gas = 0
    self.brake = 0

  def update(self, active, enabled, CS, frame, actuators, pcm_cancel_cmd):

    # *** compute control surfaces ***

    # gas and brake
    MAX_INTERCEPTOR_GAS = interp(CS.out.vEgo, [0., 2., 6., 35], [0.3, 0.25, 0.3, 0.45])
    apply_gas = clip(actuators.accel / PEDAL_SCALE, 0., MAX_INTERCEPTOR_GAS)

    MAX_BRAKE = interp(CS.out.vEgo, [0., 50.], [.65, .55])
    apply_brakes = clip(-actuators.accel / BRAKE_SCALE, 0., MAX_BRAKE)

    # steer torque
    new_steer = int(round(actuators.steer * SteerLimitParams.STEER_MAX))
    apply_steer = apply_std_steer_torque_limits(new_steer, self.last_steer, CS.out.steeringTorque, SteerLimitParams)
    self.steer_rate_limited = new_steer != apply_steer
    self.last_steer = apply_steer

    # only cut torque when steer state is a known fault
    if CS.brakeUnavailable:
      self.last_fault_frame = frame

    # Cut steering for 2s after fault
    if not active or (frame - self.last_fault_frame < 200):
      apply_steer = 0
      apply_steer_req = 0
    else:
      apply_steer_req = 1

    self.gas = apply_gas
    self.brake= apply_brakes

    can_sends = []

    #*** control msgs ***
    #print("steer {0} {1} {2} {3}".format(apply_steer, min_lim, max_lim, CS.steer_torque_motor)

    if CS.out.vEgo < 0.1 and apply_gas < 0.01:
      apply_brakes = 0.4


    can_sends.append(create_steer_command(self.packer, apply_steer, apply_steer_req, frame))
    can_sends.append(create_ibst_command(self.packer, active, apply_brakes, frame))
    can_sends.append(create_pedal_command(self.packer, active, apply_gas, frame))

    #UI mesg is at 100Hz but we send asap if:
    if (frame % 10 == 0):
      can_sends.append(create_msg_command(self.packer, enabled, CS.out.cruiseState.speed * CV.MS_TO_MPH, CS.out.vEgo * CV.MS_TO_MPH))

    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / SteerLimitParams.STEER_MAX
    new_actuators.gas = self.gas
    new_actuators.brake = self.brake


    return new_actuators, can_sends
