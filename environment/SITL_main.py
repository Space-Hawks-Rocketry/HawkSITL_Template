from time import time, sleep
import numpy as np
from simulation.environment_main import SITL_controlUpdate, SITL_physicsUpdate, SITL_pollSensorData, SITL_finish, SITL_setup
import matplotlib.pyplot as plt

from framework.core.IPC_computer import IPC_Computer
    

ipc = IPC_Computer()
ipc.start(3563)


t = 0
sim_time = 20
running = True

target_dt = 0.01 # If a computer control step is quicker than this, dt will be smaller

SITL_setup()

while running:

  sensor_data = SITL_pollSensorData()
  ipc.sendJSON(sensor_data)
  computer_response = ipc.recvJSON()

  control_msg = computer_response["control_msg"] or {}
  computer_dt = computer_response["dt"]

  SITL_controlUpdate(control_msg)

  remaining_dt = computer_dt
  while remaining_dt > target_dt:
    SITL_physicsUpdate(t, target_dt)
    remaining_dt -= target_dt
  SITL_physicsUpdate(t, remaining_dt)

  t += computer_dt

  if t >= sim_time:
    break

SITL_finish()