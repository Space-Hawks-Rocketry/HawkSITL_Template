class SITLHandle:
    '''A handle for the SITL state to be controlled by user-defined functions.'''
    
    def __init__(self, stop_delegate):
        self.stop_delegate = stop_delegate
      
    def stop(self):
        '''Call this to stop the simulation.'''
        self.stop_delegate()