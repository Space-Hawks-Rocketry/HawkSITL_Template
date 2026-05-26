from time import time, sleep
import numpy as np
import json

from framework.IPC.IPC_computer import IPC_Computer

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
    
    

ipc = IPC_Computer()
sleep(2)
ipc.start(3563)


square = Square(pos=np.array([0, 0], dtype=float))
target_pos = np.array([90.0, 45.0])

dt = 0
t = 0
running = True

ipc.sendJSON({
  "posx": 0.0,
  "posy": 0.0,
  "velx": 0.0,
  "vely": 0.0
})
res_json = ipc.recvJSON()
res_force = np.array([res_json["Fx"], res_json["Fy"]])

while running:
  start_time = time()
  
  ipc.sendJSON({
    "posx": target_pos[0] - square.pos[0],
    "posy": target_pos[1] - square.pos[1],
    "velx": square.vel[0],
    "vely": square.vel[1]
  })
  res_json = ipc.recvJSON()
  res_force = np.array([res_json["Fx"], res_json["Fy"]])
  print(f"(PHYSICS): Received control maneuver @ t={round(t,3)}s")
  
  square.applyForce(np.array([
    res_force[0],
    res_force[1]
  ]))
  square.update(dt)
  
  dt = time() - start_time
  t += dt
  