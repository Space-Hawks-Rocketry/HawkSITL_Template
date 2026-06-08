from time import time, sleep
import numpy as np
from environment_software.environment_main import SITL_controlUpdate, SITL_physicsUpdate, SITL_pollSensorData, SITL_finish, SITL_setup
import matplotlib.pyplot as plt

from framework.core.IPC_computer import IPC_Computer
    

ipc = IPC_Computer()
ipc.start(3563)


t = 0
sim_time = 20
running = True

SITL_setup()

while running:
  
  sensor_data = SITL_pollSensorData()
  ipc.sendJSON(sensor_data)
  computer_response = ipc.recvJSON()
  
  control_msg = computer_response["control_msg"]
  dt = computer_response["dt"]
  
  SITL_controlUpdate(control_msg)
  SITL_physicsUpdate(t, dt)
  
  if t >= sim_time:
    break

  t += dt

SITL_finish()