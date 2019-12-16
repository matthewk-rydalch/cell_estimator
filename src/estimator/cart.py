#cart functions
#####################

import numpy as np
import math
from importlib import reload, import_module
from IPython.core.debugger import set_trace


utils = reload(import_module("utils"))

class Cart:
    def __init__(self, sig_accel, sig_gyro, sig_gps):
        self.sig_accel = sig_accel
        self.sig_gyro = sig_gyro
        self.sig_gps = sig_gps
        self.vt = np.array([[0.0],[0.0],[0.0]]) #this needs to be remembered to calculate the next time step.  It is changed in get_vel
        self.first_accel = np.array([[0.0], [0.0], [0,0]])

    def dyn_2d(self, Xp, Ut, dt):

        Np = Xp[0][0]
        Ep = Xp[1][0]
        Dp = Xp[2][0]
        Nv = Ut[0]
        Ev = Ut[1]
        Dv = Ut[2]

        Nt = Np+Nv*dt
        Et = Ep+Ev*dt
        Dt = Dp+Dv*dt

        Xt = np.array([Nt, Et, Dt])
        return Xt

    def get_vel(self, accel, omega, dt, time):

        if time < 1.0:
            set_trace()
            self.first_accel = accel
        accel = accel-self.first_accel
        self.vt = self.vt + accel*dt

        Ut = self.vt

        return Ut

    def model_sensor(self, Xt, noise = 1):

        zt = Xt

        return(zt)
