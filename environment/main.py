from time import time
import numpy as np

from signal import Signaler

class Square:
  
  def __init__(self, m=1.0, I=1.0, pos=np.zeros(2, dtype=float), vel=np.zeros(2, dtype=float), radians=0.0, radians_rate=0.0):
    self.m = m
    self.I = I
    self.pos = pos
    self.vel = vel
    self.radians = radians
    self.radians_rate = radians_rate
    
    self.force = np.zeros(2, dtype=float)
    self.torque = 0.0
    
  def getDCMtoBody(self):
    return np.array([
      [np.cos(self.radians), np.sin(self.radians)],
      [-np.sin(self.radians), np.cos(self.radians)]
    ])
    
  def applyForce(self, force):
    self.force += force
    
  def applyTorque(self, torque):
    self.torque += torque
  
  def update(self, dt):
    self.vel += dt * self.force / self.m
    self.pos += dt * self.vel
    
    self.radians_rate += dt * self.torque / self.I
    self.radians += dt * self.radians_rate
    
    self.force = np.zeros(2, dtype=float)
    self.torque = 0.0
    
    

signaler = Signaler()


square = Square(pos=np.array([0, 0], dtype=float))
target_pos = np.array([90.0, 45.0])

dt = 0
t = 0
running = True
res_force = signaler.updateComputer(0, 0, 0, 0)
while running:
  start_time = time()
  
  res_force = signaler.updateComputer(
      target_pos[0] - square.pos[0],
      target_pos[1] - square.pos[1],
      square.vel[0],
      square.vel[1]
  )
  print(f"(PHYSICS): Received control maneuver @ t={round(t,3)}s")
  
  square.applyForce(np.array([
    res_force[0],
    res_force[1]
  ]))
  square.update(dt)
  
  dt = time() - start_time
  t += dt
  