import numpy as np

class FlyingCube:
    
    def __init__(self, m=1, initial_height=0, initial_velocity=0, max_thruster_force_newtons=15):
        self.m = m
        self.height = initial_height
        self.vel = initial_velocity
        self.max_thruster_force = max_thruster_force_newtons

        # The throttles (between 0 and 1) of thrusters 1-4.
        # If the first number is 1, it means T1 is at max throttle.
        self.thruster_throttles = np.array([0.0, 0.0, 0.0, 0.0])
        # "False" status means thruster has failed and cannot be used
        self.thruster_status = [True, True, True, True]

    def setThrusterStatus(self, thruster_index: int, status: bool):
        '''Disable/enable a thruster. First thruster has index of 0.'''
        self.thruster_status[thruster_index] = status

    def setThrottles(self, thruster_throttles: list):
        '''Set throttles (each between 0 and 1) of each thruster. Use array with length 4.'''
        self.thruster_throttles = np.clip(thruster_throttles, 0.0, 1.0)
        
    def update(self, dt):
        '''Integrate model flying cub emodel forward in time by dt seconds.'''
        # Throttles after taking into account thruster status
        active_throttles = self.thruster_throttles * self.thruster_status
        
        # Detect imbalance (and report)
        if (active_throttles[0] != active_throttles[2]) or (active_throttles[1] != active_throttles[3]):
            print("[CUBE SIM WARNING]: Flying cube is unstable. Simulation values no longer reflect real behavior.")
            
        thruster_forces = active_throttles * self.max_thruster_force
        net_thrust_force = np.sum(thruster_forces)
        net_force = self.m * -9.8 + net_thrust_force   # Total force on cube including gravity
        
        # Euler's integration (basic but good enough)
        accel = net_force / self.m
        self.vel += accel*dt
        self.height += self.vel*dt
        


