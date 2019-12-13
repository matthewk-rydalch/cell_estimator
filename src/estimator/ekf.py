#Extended information Filter algorithm
###########################

import numpy as np
import math
from numpy.linalg import inv
from IPython.core.debugger import set_trace

import utils


class Ekf():
    def __init__(self, dyn_2d, model_sensor, prediction_jacobian, measurement_jacobian, sig_accel, sig_gyro, sig_gps):
        self.dyn_2d = dyn_2d
        self.model_sensor = model_sensor
        self.prediction_jacobian = prediction_jacobian
        self.measurement_jacobian = measurement_jacobian
        self.t_prev = 0
        self.sig_accel = sig_accel
        self.sig_gyro = sig_gyro
        self.sig_gps = sig_gps

    #this function is only used if using a driver that doesn't seperate prediction and measurement steps
    def ext_info_filter(self, Ksp, Omp, Ut, Zt):

        #ToDo: fix these
        self.prediction(Ut, Omp, Ksp)
        self.measure()

    def prediction(self, Ut, Mu, Sig, dt):

        #prediction step
        Gt= self.prediction_jacobian(Ut, Mu, dt)
        Rt= self.propogation_matrices(Mu, dt)
        Sig_bar = Gt@Sig@Gt.T+Rt
        Mu_bar = self.dyn_2d(Mu, Ut, dt)

        return Mu_bar, Sig_bar

    def measure(self, Mu_bar, Sig_bar, Zt):
        Ht= self.measurement_jacobian(Mu_bar, Zt)
        Qt = self.measurement_matrices()
        St = Ht@Sig_bar[0:2,0:2]@Ht.T+Qt
        Kt = Sig_bar[0:2,0:2]@Ht.T@inv(St)
        # Kt = np.eye(2)
        Mu_bar[0:2] = Mu_bar[0:2]+Kt@(Zt-Mu_bar[0:2])
        # set_trace()
        Sig_bar[0:2,0:2] = (np.eye(2)-Kt@Ht)@Sig_bar[0:2,0:2]

        Sig = Sig_bar
        Mu = Mu_bar

        return Mu, Sig

    def propogation_matrices(self, Mu, dt):

        thp = Mu[2]

        Vt = np.squeeze(np.array([[math.cos(thp)*dt, 0.0],\
                                  [math.sin(thp)*dt, 0.0],\
                                  [0.0, dt]]))

        Mt = np.array([[self.sig_accel**2, 0.0],\
                       [0.0, self.sig_gyro**2]])
        Rt = Vt@Mt@Vt.T

        return Rt

    def measurement_matrices(self):

        Qt = np.array([[self.sig_gps**2, 0.0],\
                       [0.0, self.sig_gps**2]])

        return Qt