#multirotor functions
#####################

import numpy as np
import math
from importlib import reload, import_module
from IPython.core.debugger import set_trace


utils = reload(import_module("utils"))

class Multirotor:
    def __init__(self, sig_v, sig_w, sig_r, sig_phi, dt, M):
        self.sig_v = sig_v
        self.sig_w = sig_w
        self.sig_r = sig_r
        self.sig_phi = sig_phi
        self.dt = dt
        self.M = M

    def dyn_2d(self, Xp, Ut):

        xp = Xp[0][0]
        yp = Xp[1][0]
        thp = Xp[2][0]
        vt = Ut[0]
        wt = Ut[1]

        xt = xp+(vt*math.cos(thp))*self.dt
        yt = yp+(vt*math.sin(thp))*self.dt
        tht = thp+wt*self.dt

        Xt = np.array([[xt], [yt], [tht]])

        return Xt

    def get_vel(self, Uc, noise = 1):

        vc = Uc[0]
        wc = Uc[1]

        epv = noise*np.random.normal(0, self.sig_v)
        epw = noise*np.random.normal(0, self.sig_w)

        vt = vc+epv
        wt = wc+epw

        Ut = np.array([vt, wt])

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