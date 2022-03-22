#!/usr/bin/env python3
import time
from selfdrive.car.interfaces import RadarInterfaceBase

class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    self.pts = {}
    self.delay = 0

  def update(self, can_strings):
    return super().update(None)
