#!/usr/bin/env python3
from enum import Enum

class LongTunes(Enum):
  SMART_PEDAL = 0

class LatTunes(Enum):
  SMART_PID = 0

###### LONG ######
def set_long_tune(tune, name):
  # Improved longitudinal tune
  if name == LongTunes.SMART_PEDAL:
    #Longitudinal deadzone values
    tune.deadzoneBP = [0., 9.]
    tune.deadzoneV = [0., .15]

    #Longitudinal Proportional values
    tune.kpBP = [0., 5., 20., 30.]
    tune.kpV = [0.4, 0.6, 0.75, 0.8]
    tune.kiBP = [0., 5., 12., 20., 27.]
    tune.kiV = [.3, .2, .1, .05, .03]
  else:
    raise NotImplementedError('This longitudinal tune does not exist')


###### LAT ######
def set_lat_tune(tune, name):
  if name == LatTunes.SMART_PID:
    tune.init('pid')
    tune.pid.kiBP, tune.pid.kpBP = [[0., 20.0], [0., 20.0]]
    tune.pid.kpV, tune.pid.kiV = [[0.05, 0.04], [0.015, 0.008]]
    tune.pid.kf = 0.00003
  else:
    raise NotImplementedError('This lateral tune does not exist')
