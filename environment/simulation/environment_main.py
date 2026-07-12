from framework.core.SITL_handle import SITLHandle
## ------------------------------------------
## Entry point to the user-defined environment simulation.
## ------------------------------------------
import matplotlib.pyplot as plt
from .models.flying_cube import FlyingCube
from .models.altimeter import Altimeter

# Create the cube!!
cube = FlyingCube(initial_height=20)
altimeter1 = Altimeter(sample_rate=5)
altimeter2 = Altimeter(sample_rate=5)
altimeter1_status = True  # True means working
altimeter2_status = True  # True means working

## Sim data to be propogated over time and then plotted
t_data = []
cube_height_data = []
cube_altimeter_data = []


def SITL_setup(sitl: SITLHandle):
  '''Called before starting simulation.'''
  pass

def SITL_physicsUpdate(sitl: SITLHandle, t: float, dt: float):
  '''Called at each time step. Integrate physics forward by dt.'''
  global cube, t_data, cube_height_data
  
  ## Update sensors
  altimeter1.update(cube.height, dt)
  altimeter2.update(cube.height, dt)
  
  ## Keep track of sim data
  t_data.append(t)
  cube_height_data.append(cube.height)
  cube_altimeter_data.append(altimeter1.measure())
  
  ## Stop the sim if the cube hits the ground (height=0)
  if cube.height <= 0:
    sitl.stop()
  
  cube.update(dt) # Progress the cube forward in time by dt

def SITL_controlUpdate(sitl: SITLHandle, control_msg: dict):
  '''Called when a control message is received from the simulated flight computer. 
    The control_msg contents is defined by flight_main.cpp.'''
  
  ## Update thruster throttles if the flight computer commands it
  if "thruster_throttles" in control_msg:
    cube.setThrottles(control_msg["thruster_throttles"])

def SITL_createSensorData(sitl: SITLHandle) -> dict:
  '''Must return sensor data to be parsed by the simulated flight computer.
    Returned dict structure is defined here and utilized by flight_main.cpp.'''
  return {
    "altitude1": altimeter1.measure(),
    "altitude2": altimeter2.measure(),
    "altimeter_status": [altimeter1_status, altimeter2_status],
    "thruster_status": cube.thruster_status
  }
  
def SITL_finish(sitl: SITLHandle):
  '''Called immediately before stopping the simulation.'''
  ## Plot sim data for visualization
  plt.plot(t_data, cube_height_data)
  plt.plot(t_data, cube_altimeter_data)
  
  plt.title("Flying Cube")
  plt.xlabel("Time (s)")
  plt.ylabel("Cube Height (m)")
  plt.show()