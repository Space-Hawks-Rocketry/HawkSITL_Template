from simulation.environment_main import SITL_controlUpdate, SITL_physicsUpdate, SITL_pollSensorData, SITL_finish, SITL_setup
from framework.core.IPC_computer import IPC_Computer

import tomllib

# Load configuration from sim-config.toml
with open("sim-config.toml", "rb") as f:
    sim_config = tomllib.load(f)

# Start environment<-->computer IPC communication
ipc = IPC_Computer()
ipc.start(sim_config.get("IPC_port", 5400))

# Define simulation parameters
sim_stop_time = sim_config.get("sim_stop_time", 60)
target_dt = sim_config.get("target_dt", 0.01)  # If a computer control step is quicker than this, dt will be smaller

SITL_setup() # User-defined setup

t = 0
running = True
# Begin main simulation loop
while running:

  sensor_data = SITL_pollSensorData() # User-defined sensor data
  ipc.sendJSON(sensor_data)
  computer_response = ipc.recvJSON() # User-defined computer control response

  control_msg = computer_response["control_msg"] or {}
  computer_dt = computer_response["dt"]

  SITL_controlUpdate(control_msg) # User-defined control update handler

  remaining_dt = computer_dt
  while remaining_dt > target_dt:
    SITL_physicsUpdate(t, target_dt) # User-defined physics update
    remaining_dt -= target_dt
  SITL_physicsUpdate(t, remaining_dt) # User-defined physics update

  t += computer_dt

  if t >= sim_stop_time:
    break

SITL_finish() # User-defined finish handler