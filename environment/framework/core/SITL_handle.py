class SITLHandle:
    '''A handle for the SITL state to be controlled by user-defined functions.'''
    
    def __init__(self, stop_delegate, t0=0):
        self._stop_delegate = stop_delegate
        self.t = t0
      
    def stop(self):
        '''Call this to stop the simulation.'''
        self._stop_delegate()