#Extended information Filter algorithm
###########################

import numpy as np
import math
from numpy.linalg import inv
from IPython.core.debugger import set_trace

import utils


class Eif():
    def __init__(self, dyn_2d, model_sensor, dt, sig_v, sig_w, sig_r, sig_phi, M):
        self.dyn_2d = dyn_2d
        self.model_sensor = model_sensor
        self.dt = dt
        self.sig_v = sig_v
        self.sig_w = sig_w
        self.sig_r = sig_r
        self.sig_phi = sig_phi
        self.M = M

    def ext_info_filter(self, Ksp, Omp, Ut, Zt):

        #prediction step
        Mup = inv(Omp)@Ksp
        # Mup[2] = utils.wrap(Mup[2]) #could be wrapped, but to match true theta, don't
        thp = Mup[2]
        Gt, Rt= self.propogation_matrices(Ut, thp)
        Omg_bar = inv(Gt@inv(Omp)@Gt.T+Rt)
        g_function = self.dyn_2d(Mup, Ut) #g is needed for both prediction and measurement
        Ks_bar = Omg_bar@g_function

        #measurement step for each marker
        mu_bar = g_function
        no_noise = 0
        h_function = self.model_sensor(mu_bar, no_noise)
        for i in range(len(self.M[0])):
            Ht, Qt = self.measurement_matrices(self.M[:,i], mu_bar)
            #reassign Omg to Omg_bar until all markers are accounted for
            Omg_bar = Omg_bar+Ht.T@inv(Qt)@Ht
            #reassign Ks to Ks_bar until all markers are accounted for
            Ks_bar = Ks_bar+Ht.T@inv(Qt)@(np.array([Zt[:,i]]).T-np.array([h_function[:,i]]).T+Ht@mu_bar)

        Ks = Ks_bar
        Omg = Omg_bar

        return Ks, Omg

    def propogation_matrices(self, Ut, thp):

        vc = Ut[0]
        wc = Ut[1]
        Gt = np.squeeze(np.array([[1.0, 0.0, -vc*math.sin(thp)*self.dt],\
                                  [0.0, 1.0, vc*math.cos(thp)*self.dt],\
                                  [0.0, 0.0, 1.0]]))

        Vt = np.squeeze(np.array([[math.cos(thp)*self.dt, 0.0],\
                                  [math.sin(thp)*self.dt, 0.0],\
                                  [0.0, self.dt]]))

        Mt = np.array([[self.sig_v**2, 0.0],\
                       [0.0, self.sig_w**2]])

        Rt = Vt@Mt@Vt.T

        return Gt, Rt

    def measurement_matrices(self, m, mu_bar):
        mx = m[0]
        my = m[1]
        mub_x = mu_bar[0][0]
        mub_y = mu_bar[1][0]
        mub_th = mu_bar[2][0]

        q = (mx-mub_x)**2+(my-mub_y)**2
        Ht = np.array([[(mub_x-mx)/math.sqrt(q), (mub_y-my)/math.sqrt(q), 0.0],\
                       [(my-mub_y)/q, (mub_x-mx)/q, -1.0]])

        Qt = np.array([[self.sig_r**2, 0.0],\
                       [0.0, self.sig_phi**2]])

        return Ht, Qt