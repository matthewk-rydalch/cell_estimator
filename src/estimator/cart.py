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
        # print('a_n = ', accel[0])
        print('a_e = ', accel[2])
        self.vt = self.vt + accel*dt
        # print('v_n = ', self.vt[0])
        # print('v_e = ', self.vt[1])
        # vt_mag = self.vt[0]/10
        # vt_mag = np.sqrt(self.vt[0]**2+self.vt[1]**2)
        # print('vt_mag = ', vt_mag)
        om_mag = omega[2]
        vt_mag = np.array([1.6])
        Ut = np.array([vt_mag, om_mag])

        return Ut

    def model_sensor(self, Xt, noise = 1):

        zt = Xt

        return(zt)
