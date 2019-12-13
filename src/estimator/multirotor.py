#multirotor functions
#####################

import numpy as np
import math
from importlib import reload, import_module
from IPython.core.debugger import set_trace


utils = reload(import_module("utils"))

class Multirotor:
    def __init__(self, sig_accel, sig_gyro, sig_gps):
        self.sig_accel = sig_accel
        self.sig_gyro = sig_gyro
        self.sig_gps = sig_gps
        self.vt = np.array([[0],[0],[0]]) #this needs to be remembered to calculate the next time step.  It is changed in get_vel

    def dyn_2d(self, Xp, Ut, dt):

        Np = Xp[0][0]
        Ep = Xp[1][0]
        thp = Xp[2][0]
        vt = Ut[0]
        wt = Ut[1]

        Nt = Np+(vt*math.cos(thp))*dt
        Et = Ep+(vt*math.sin(thp))*dt
        tht = thp+wt*dt

        Xt = np.array([Nt, Et, tht])
        return Xt

    def get_vel(self, accel, omega, dt, noise = 1):

        self.vt = self.vt + accel*dt
        vt_mag = np.sqrt(self.vt[0]**2+self.vt[1]**2)
        om_mag = omega[2]
        # self.v_t = a_t*dt+self.v_prev
        # self.omega_t = alpha_t*dt+self.omega_prev

        # epv = noise*np.random.normal(0, self.sig_v)
        # epw = noise*np.random.normal(0, self.sig_w)
        #
        # vt = vc+epv
        # wt = wc+epw

        Ut = np.array([vt_mag, om_mag])

        return Ut

    def model_sensor(self, Xt, noise = 1):

        zt = Xt

        return(zt)
