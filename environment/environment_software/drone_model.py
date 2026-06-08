import numpy as np

class Drone:
  
  def __init__(self, pos=np.zeros(2, dtype=float), vel=np.zeros(2, dtype=float), radians=0.0, radians_rate=0.0, m=1.0, I=1.0, width=2.0):
    self.pos = pos
    self.vel = vel
    self.radians = radians
    self.radians_rate = radians_rate
    self.m = m
    self.I = I
    self.width = width
    self.fan1_thrust = 0
    self.fan2_thrust = 0
    
  def getDCMtoBody(self):
    return np.array([
      [np.cos(self.radians), np.sin(self.radians)],
      [-np.sin(self.radians), np.cos(self.radians)]
    ])
    
  def setFanThrust(self, fan1: float, fan2: float):
    self.fan1_thrust = max(0.0, fan1)
    self.fan2_thrust = max(0.0, fan2)
    
  def update(self, dt):
    force = self.getDCMtoBody().T @ np.array([0, self.fan1_thrust + self.fan2_thrust]) + np.array([0, -9.8 * self.m])
    torque = 0.5 * self.width * (self.fan2_thrust - self.fan1_thrust)
    
    self.vel += dt * force / self.m
    self.pos += dt * self.vel
    
    self.radians_rate += dt * torque / self.I
    self.radians += dt * self.radians_rate