import numpy as np



class Birotor: 
    def __init__(self, m=1.0):
        self.m = m
        self.g = 9.81
        self.I = np.array([0.1, 0.1, 0.2])
        
        # State variables (y, z, phi, y_dot, z_dot, phi_dot)
        self.X = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
    def advance_state(self, f1, f2, dt):
        """
        Advances the state of the drone forward by dt seconds given the control inputs u1 and u2.
        u1 is a collective thrust acting on the drone, and u2 is a torque acting on the drone.
        """