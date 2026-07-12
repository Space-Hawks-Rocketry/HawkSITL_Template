from framework.core.SITL_handle import SITLHandle
## ------------------------------------------
## Entry point to the user-defined environment simulation.
## ------------------------------------------
import matplotlib.pyplot as plt
from random import randint
from .models.flying_cube import FlyingCube
from .models.altimeter import Altimeter

## Define initial sim parameters
CUBE_START_HEIGHT = 20  # in meters
is_cube_suspended = True   # Suspend cube for a bit while computer is calibrating

## Create the cube!!
cube = FlyingCube(initial_height=CUBE_START_HEIGHT)
## Create the sensors
altimeter1 = Altimeter(sample_rate=30, white_noise_std=0.1, instability_rate_std=0.01)
altimeter2 = Altimeter(sample_rate=5)
altimeter1_status = True  # True means working
altimeter2_status = True  # True means working

## Sim data to be plotted
t_data = []
cube_height_data = []
cube_altimeter_data = []
alt_est_data = []
alt_t_data = []


def injectThrusterFailure():
  '''Fail random thruster out of the 4.'''
  thruster_fail_index = randint(0, 3)
  cube.setThrusterStatus(thruster_fail_index, False)
  
def injectAltimeterFailure():
  '''Fail altimeter 1 (the better one).'''
  global altimeter1_status
  altimeter1_status = False


def SITL_setup(sitl: SITLHandle):
  '''Called before starting simulation.'''
  ## Schedule events
  sitl.scheduleAtTime(20, injectThrusterFailure)
  sitl.scheduleAtTime(40, injectAltimeterFailure)


def SITL_physicsUpdate(sitl: SITLHandle, t: float, dt: float):
  '''Called at each time step. Integrate physics forward by dt.'''
  global cube, t_data, cube_height_data, altimeter1_status, is_cube_suspended
  
  ## Update models
  altimeter1.update(cube.height, dt)
  altimeter2.update(cube.height, dt)
  cube.update(dt)
  
  ## Suspend cube until it's finished calibrating
  if is_cube_suspended:
    cube.height = CUBE_START_HEIGHT
    cube.vel = 0
  
  ## Keep track of sim data
  t_data.append(t)
  cube_height_data.append(cube.height)
  
  ## Stop the sim if the cube hits the ground (height=0)
  if cube.height <= 0:
    print("CONTROL FAILURE: Hit the ground, ur dead!")
    sitl.stop()


def SITL_controlUpdate(sitl: SITLHandle, control_msg: dict):
  '''Called when a control message is received from the simulated flight computer. 
    The control_msg contents is defined by flight_main.cpp.'''
  global is_cube_suspended
  
  ## Listen to flight status signals
  if "flight_status" in control_msg:  
    status = control_msg["flight_status"]
    if status == "READY":
      # Release cube once calibration is complete
      is_cube_suspended = False
  
  ## Update thruster throttles if the flight computer commands it
  if "thruster_throttles" in control_msg:
    cube.setThrottles(control_msg["thruster_throttles"])
    alt_est_data.append(control_msg["alt_estimate"])
    alt_t_data.append(sitl.t)


def SITL_createSensorData(sitl: SITLHandle) -> dict:
  '''Must return sensor data to be parsed by the simulated flight computer.
    Returned dict structure is defined here and utilized by flight_main.cpp.'''
  data =  {
    "altimeter_status": [altimeter1_status, altimeter2_status],
    "thruster_status": cube.thruster_status
  }
  
  ## Add sensor data if new measurements are available
  if altimeter1.isReady():
    data["altitude1"] = altimeter1.measure()
  if altimeter2.isReady():
    data["altitude2"] = altimeter2.measure()
  
  return data
  
  
def SITL_finish(sitl: SITLHandle):
  '''Called immediately before stopping the simulation.'''
  ## Plot sim data for visualization
  plt.plot(t_data, cube_height_data)
  # plt.plot(alt_t_data, alt_est_data)
  
  plt.title("Flying Cube")
  plt.xlabel("Time (s)")
  plt.ylabel("Cube Height (m)")
  plt.show()