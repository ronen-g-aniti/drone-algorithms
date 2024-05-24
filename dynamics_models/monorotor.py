import numpy as np

class Monorotor:
    
    def __init__(self, m=1.0):
            self.m = m
            self.g = 9.81
            
            # State variables (z, z_dot)
            self.X = np.array([0.0, 0.0])
        
    def advance_state(self, u, dt):
        """
        Advances the state of the drone forward by dt seconds given the control input u.
        The control input, u, represents an applied thrust acting on the drone.
        """
        X_dot = np.array([self.X[1], (self.m * self.g - u) / self.m])
        self.X = self.X + X_dot * dt
        return self.X