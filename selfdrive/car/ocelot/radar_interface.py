#!/usr/bin/env python3
import time
from selfdrive.car.interfaces import RadarInterfaceBase

class class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    self.pts = {}
    self.delay = 0

  def update(self, can_strings):
    ret = car.RadarData.new_message()
    time.sleep(0.02)  # radard runs on RI updates
    return ret
