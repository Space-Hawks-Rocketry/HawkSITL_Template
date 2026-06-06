from time import time, sleep
import numpy as np
import json
import matplotlib.pyplot as plt
import csv

from framework.core.IPC_computer import IPC_Computer

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
    

ipc = IPC_Computer()
ipc.start(3563)


drone = Drone()
target_pos = np.array([1.0, -8.0])


file = open("sim_data.csv", "w", newline="")
writer = csv.writer(file)
writer.writerow(["t", "x", "y", "radians"])


dt = 0.01
t = 0
sim_time = 20
running = True

x_data = []
y_data = []

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
  
  
  drone.update(dt)
  
  x_data.append(drone.pos[0])
  y_data.append(drone.pos[1])
  writer.writerow([t, drone.pos[0], drone.pos[1], drone.radians])
  
  if t >= sim_time:
    break

  t += dt

plt.plot(x_data, y_data)
plt.plot(target_pos[0], target_pos[1], "ro")
plt.savefig("myfig.png")

file.close()