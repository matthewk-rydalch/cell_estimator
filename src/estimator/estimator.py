import rospy
import numpy as np
from importlib import reload, import_module
import math
from numpy.linalg import inv

multirotor = reload(import_module("multirotor"))
from multirotor import Multirotor
visualizer = reload(import_module("visualizer"))
from visualizer import Visualizer
eif = reload(import_module("eif"))
from eif import Eif
utils = reload(import_module("utils"))


DEBUG = False
def printer(statement):
    if DEBUG:
        printer(statement)

class Estimator():
    def __init__(self):

        #parameters
        xlim = 30.0 #m
        ylim = 30.0 #m
        sig_gps = 3 #m
        sig_accel = 1 #m/s^2
        sig_gyro = 1 #m^2/s^2
        # sig_r = 0.2 #m
        # sig_phi = 0.1 #rad
        # sig_v = 0.15 #m/s
        # sig_w = 0.1 #rad/s
        N0 = 0.0 #m
        E0 = 0.0 #m
        #alitude does not change

    	#my specified parameters and variables
        Sig = np.array([[1.0, 0.0],
                        [0.0, 1.0]])

        Mu = np.array([[N0],[E0]])

        Sig_hist = []
        Mu_hist = []
        Ks_hist = []
        Z_hist = []

        #instantiate classes
        mr = Multirotor(sig_accel, sig_gyro, sig_gps)
        viz = Visualizer(xlim, ylim)
        eif = Eif(mr.dyn_2d, mr.model_sensor, sig_accel, sig_gyro, sig_gps)

        #convert to information form
        Om = inv(Sig)
        Ks = Om@Mu

    def imu_callback(self, data):
        # Ut = get_vel(data, time)
        # self.Ks, self.Om = propagate(Ut)
        printer('got imu')

    def ned_callback(self, data):
        # ned = np.zeros((3,1))
        # ned[0] = NED_data.relPosNED[0]
        # ned[1] = NED_data.relPosNED[1]
        # ned[2] = NED_data.relPosNED[2]
        # self.Ks, self.Om = measurement(ned)
        printer('got ned')

    def lla_callback(self, data):
        #this line is simply to verify that messages are being subscribed to during development
        # printer("lla_callback \n")
        printer('got lla')

    def rover_RelPos_callback(self, data):
        #this line is simply to verify that messages are being subscribed to during development
        printer("rover_RelPos_callback \n")

    # def get_vel(self):

    # def propagate(self, Ut):
    #     #prediction step
    #     Mup = inv(Omp)@Ksp
    #     # Mup[2] = utils.wrap(Mup[2]) #could be wrapped, but to match true theta, don't
    #     thp = Mup[2]
    #     Gt, Rt= self.mr.propogation_matrices(Ut, thp)
    #     Omg_bar = inv(Gt@inv(Omp)@Gt.T+Rt)
    #     g_function = self.dyn_2d(Mup, Ut) #g is needed for both prediction and measurement
    #     Ks_bar = Omg_bar@g_function

    #     Ks = Ks_bar
    #     Omg = Omg_bar

    #     return Ks, Omg

    # def measurement(self, ned):
    #     #measurement step for each marker
    #     mu_bar = g_function
    #     no_noise = 0
    #     h_function = self.model_sensor(mu_bar, no_noise)
    #     for i in range(len(self.M[0])):
    #         Ht, Qt = self.measurement_matrices(self.M[:,i], mu_bar)
    #         #reassign Omg to Omg_bar until all markers are accounted for
    #         Omg_bar = Omg_bar+Ht.T@inv(Qt)@Ht
    #         #reassign Ks to Ks_bar until all markers are accounted for
    #         Ks_bar = Ks_bar+Ht.T@inv(Qt)@(utils.wrap(np.array([Zt[:,i]]).T-np.array([h_function[:,i]]).T)+Ht@mu_bar)
    #     Ks = Ks_bar
    #     Omg = Omg_bar

    #     return Ks, Omg
