## ------------------------------------------
## Entry point to the environment simulation.
## ------------------------------------------

def SITL_setup():
  '''Required SITL Func: Called before starting simulation.'''
  pass

def SITL_physicsUpdate(t, dt):
  '''Required SITL Func: Called at each time step. Integrate physics forward by dt.'''
  pass

def SITL_controlUpdate(control_msg: dict):
  '''Required SITL Func: Called when a control message is received by flight computer. 
    The control_msg structure is defined by flight software.'''
  pass

def SITL_pollSensorData() -> dict:
  '''Required SITL Func: Must return sensor data to be parsed by flight computer.
    Returned dict structure is defined here and utilized by flight software.'''
  return {}
  
def SITL_finish():
  '''Required SITL Func: Called upon finishing the simulation.'''
  pass