import time
import numpy as np
from numpy import pi

class PID:
    def __init__(self, k, angle=False, windup=None):
        self.k = k # tuning params
        self.angle = angle # if this is a PID controller for angles

        # setup PID information
        self.windup = windup
        self.e0 = 0.
        self.ei = 0.
        self.t0 = time.time()
        
    def input(self, x, x_des):
        dt = time.time() - self.t0

        # compute error terms
        e = x_des - x

        # limit the error if angle tracking
        if self.angle: e = e - 2*pi(e > pi) + 2*pi(e < -pi)

        # compute remaining terms
        ed = (e - self.e0)/dt
        ei = ei + e*dt

        # prevent windup
        if windup is not None:
            ei = max(min(ei, abs(windup)), -abs(windup))
            
        # compute the input
        u = self.k @ np.array([e, ei, ed])

        # store terms for next iteration
        self.e0 = e
        self.t0 = time.time()

        return u
