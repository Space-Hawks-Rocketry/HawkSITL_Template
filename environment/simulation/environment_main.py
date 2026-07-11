from framework.core.SITL_handle import SITLHandle
## ------------------------------------------
## Entry point to the user-defined environment simulation.
## ------------------------------------------

def SITL_setup(sitl: SITLHandle):
  '''Called before starting simulation.'''
  pass

def SITL_physicsUpdate(sitl: SITLHandle, t: float, dt: float):
  '''Called at each time step. Integrate physics forward by dt.'''
  pass

def SITL_controlUpdate(sitl: SITLHandle, control_msg: dict):
  '''Called when a control message is received from the simulated flight computer. 
    The control_msg contents is defined by flight_main.cpp.'''
  pass

def SITL_pollSensorData(sitl: SITLHandle) -> dict:
  '''Must return sensor data to be parsed by the simulated flight computer.
    Returned dict structure is defined here and utilized by flight_main.cpp.'''
  return {}
  
def SITL_finish(sitl: SITLHandle):
  '''Called immediately before stopping the simulation.'''
  pass