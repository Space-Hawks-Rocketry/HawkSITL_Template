import numpy as np

class Altimeter:
    
    def __init__(self, sample_rate: float, white_noise_std=1, instability_rate_std=0.05):
        self.sample_rate = sample_rate
        self.white_noise_std = white_noise_std
        self.instability_rate_std = instability_rate_std
        
        self.rng = np.random.default_rng()
        self.bias_instability = 0.0
        self.time_until_next_measurement = 0.0
        
        self.altitude_measurement = 0.0
        
    def update(self, true_altitude, dt):
        ## Introduce random walking bias (bias instability)
        instability_rate = self.rng.normal(loc=0, scale=self.instability_rate_std)
        self.bias_instability += instability_rate * dt
        
        ## Make measurement every now and then
        if self.time_until_next_measurement <= 0:
            white_noise = self.rng.normal(loc=0, scale=self.white_noise_std)
            self.altitude_measurement = true_altitude + white_noise + self.bias_instability
            self.time_until_next_measurement = 1.0 / self.sample_rate
            
        self.time_until_next_measurement -= dt
        
    def measure(self) -> float:
        '''Measure flawed altimeter altitude based on sim-truth altitude.'''
        return self.altitude_measurement
        