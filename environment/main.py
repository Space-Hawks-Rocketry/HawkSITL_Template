from time import time, sleep
import numpy as np
import json
import matplotlib.pyplot as plt

from framework.IPC.IPC_computer import IPC_Computer

# class Square:
  
#   def __init__(self, m=1.0, I=1.0, pos=np.zeros(2, dtype=float), vel=np.zeros(2, dtype=float), radians=0.0, radians_rate=0.0):
#     self.m = m
#     self.I = I
#     self.pos = pos
#     self.vel = vel
#     self.radians = radians
#     self.radians_rate = radians_rate
    
#     self.force = np.zeros(2, dtype=float)
#     self.torque = 0.0
    
#   def getDCMtoBody(self):
#     return np.array([
#       [np.cos(self.radians), np.sin(self.radians)],
#       [-np.sin(self.radians), np.cos(self.radians)]
#     ])
  
#   def applyForce(self, force):
#     self.force += force
    
#   def applyTorque(self, torque):
#     self.torque += torque
  
#   def update(self, dt):
#     self.vel += dt * self.force / self.m
#     self.pos += dt * self.vel
    
#     self.radians_rate += dt * self.torque / self.I
#     self.radians += dt * self.radians_rate
    
#     self.force = np.zeros(2, dtype=float)
#     self.torque = 0.0
    

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
    self.fan1_thrust = max(0, fan1)
    self.fan2_thrust = max(0, fan2)
    
  def update(self, dt):
    force = self.getDCMtoBody().T @ np.array([0, 2 * min(self.fan1_thrust, self.fan2_thrust)]) + np.array([0, -9.8*self.m])
    torque = 0.5 * self.width * (self.fan2_thrust - self.fan1_thrust)
    
    self.vel += dt * force / self.m
    self.pos += dt * self.vel
    
    self.radians_rate += dt * torque / self.I
    self.radians += dt * self.radians_rate
    

ipc = IPC_Computer()
ipc.start(3563)


# square = Square(pos=np.array([0, 0], dtype=float))
drone = Drone()
target_pos = np.array([0.1, 5.0])


dt = 0.01
t = 0
sim_time = 10
running = True

x_data = []
y_data = []

ipc.sendJSON({
  "posx": 0.0,
  "posy": 0.0,
  "velx": 0.0,
  "vely": 0.0,
  "radians": 0.0,
  "radians_rate": 0.0
})
res_json = ipc.recvJSON()
# res_force = np.array([res_json["F1"], res_json["F2"]])

while running:
  start_time = time()
  
  ipc.sendJSON({
    "posx": target_pos[0] - drone.pos[0],
    "posy": target_pos[1] - drone.pos[1],
    "velx": 0 - drone.vel[0],
    "vely": 0 - drone.vel[1],
    "radians": drone.radians,
    "radians_rate": drone.radians_rate
  })
  res_json = ipc.recvJSON()
  commands = res_json["commands"]
  if ("F1" in commands) and ("F2" in commands):
    drone.setFanThrust(commands["F1"], commands["F2"])
  
  # print(commands)
  # res_force = np.array([res_json["Fx"], res_json["Fy"]])
  # print(f"(PHYSICS): Received control maneuver @ t={round(t,3)}s")
  
  # square.applyForce(np.array([
  #   res_force[0],
  #   res_force[1]
  # ]))
  # square.update(dt)
  # drone.setFanThrust()
  
  drone.update(dt)
  
  x_data.append(drone.pos[0])
  y_data.append(drone.pos[1])
  
  if t >= sim_time:
    break
  
  # dt = time() - start_time
  t += dt

plt.plot(x_data, y_data)
plt.plot(target_pos[0], target_pos[1], "ro")
plt.savefig("myfig.png")