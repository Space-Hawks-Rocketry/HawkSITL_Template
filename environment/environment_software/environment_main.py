import matplotlib.pyplot as plt
import numpy as np
import csv

from .drone_model import Drone


drone = Drone()
target_pos = np.array([1.0, -8.0])

file = open("sim_data.csv", "w", newline="")
writer = csv.writer(file)
writer.writerow(["t", "x", "y", "radians"])
x_data = []
y_data = []


def SITL_setup():
  '''Required SITL Func: Called before starting simulation.'''
  pass

def SITL_physicsUpdate(t, dt):
  '''Required SITL Func: Called at each time step. Integrate physics forward by dt.'''
  drone.update(dt)
  
  x_data.append(drone.pos[0])
  y_data.append(drone.pos[1])
  writer.writerow([t, drone.pos[0], drone.pos[1], drone.radians])

def SITL_controlUpdate(control_msg: dict):
  '''Required SITL Func: Called when a control message is received by flight computer. 
    The control_msg structure is defined by flight software.'''
  if (type(control_msg) == dict) and ("F1" in control_msg) and ("F2" in control_msg):
    drone.setFanThrust(control_msg["F1"], control_msg["F2"])

def SITL_pollSensorData() -> dict:
  '''Required SITL Func: Must return sensor data to be parsed by flight computer.
    Returned dict structure is defined here and utilized by flight software.'''
  return {
    "posx": target_pos[0] - drone.pos[0],
    "posy": target_pos[1] - drone.pos[1],
    "velx": 0 - drone.vel[0],
    "vely": 0 - drone.vel[1],
    "radians": drone.radians,
    "radians_rate": drone.radians_rate
  }
  
def SITL_finish():
  '''Required SITL Func: Called upon finishing the simulation.'''
  plt.plot(x_data, y_data)
  plt.plot(target_pos[0], target_pos[1], "ro")
  plt.savefig("myfig.png")

  file.close()