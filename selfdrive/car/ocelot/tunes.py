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
    tune.kpBP = [0., 5., 35.]
    tune.kpV = [1., 0.8, 0.5]

    #Longitudinal Integral Values
    tune.kiBP = [0., 55.]
    tune.kiV = [0.3, 0.2]
  else:
    raise NotImplementedError('This longitudinal tune does not exist')


###### LAT ######
def set_lat_tune(tune, name):
  if name == LatTunes.SMART_PID:
    tune.init('pid')
    tune.pid.kiBP, tune.pid.kpBP = [[0.], [0.]]
    tune.pid.kpV, tune.pid.kiV = [[0.05], [0.008]]
    tune.pid.kf = 0.00007818594
  else:
    raise NotImplementedError('This lateral tune does not exist')
