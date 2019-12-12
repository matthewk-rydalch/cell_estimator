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
        self.t_prev = 0 #this is updated in get_vel#used to calculate dt
        self.vt = np.array([[0],[0],[0]]) #this needs to be remembered to calculate the next time step.  It is changed in get_vel

    def dyn_2d(self, Xp, Ut, time):

        Np = Xp[0][0]
        Ep = Xp[1][0]
        thp = Xp[2][0]
        vt = Ut[0]
        wt = Ut[1]

        xt = xp+(vt*math.cos(thp))*self.dt
        yt = yp+(vt*math.sin(thp))*self.dt
        tht = thp+wt*self.dt

        Xt = np.array([[xt], [yt], [tht]])

        return Xt

    def get_vel(self, accel, omega, time, noise = 1):

        dt = np.array([[time-self.t_prev]])
        self.vt = self.vt + accel*dt
        self.t_prev = time
        # self.v_t = a_t*dt+self.v_prev
        # self.omega_t = alpha_t*dt+self.omega_prev

        # epv = noise*np.random.normal(0, self.sig_v)
        # epw = noise*np.random.normal(0, self.sig_w)
        #
        # vt = vc+epv
        # wt = wc+epw

        Ut = np.array([self.vt, omega])

        return Ut

    def model_sensor(self, Xt, noise = 1):

        #states
        xt = Xt[0,:]
        yt = Xt[1,:]
        tht = Xt[2,:]

        #range components w/o sensor noise
        difx = self.M[0]-xt
        dify = self.M[1]-yt

        #range and bearing w/o sensor noise/truth
        zr_tru = np.sqrt(difx**2+dify**2)
        zb_tru = np.arctan2(dify,difx)

        #add in sensor noise if noise is not specified as 0
        zr = zr_tru + noise*np.random.normal(0, self.sig_r)
        zb = utils.wrap(zb_tru - tht + noise*np.random.normal(0, self.sig_phi))

        z = np.array([zr,zb])

        return(z)
