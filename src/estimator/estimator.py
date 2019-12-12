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
        self.mr = mr
        self.viz = viz
        self.eif = eif

        #convert to information form
        self.Om = inv(Sig)
        self.Ks = self.Om@Mu

    def imu_callback(self, data):
        accel_x = data.linear_acceleration.x
        accel_y = data.linear_acceleration.y
        accel_z = data.linear_acceleration.z
        accel = np.array([[accel_x],[accel_y],[accel_z]])
        omega_x = data.angular_velocity.x
        omega_y = data.angular_velocity.y
        omega_z = data.angular_velocity.z
        omega = np.array([[omega_x],[omega_y],[omega_z]])
        time = data.header.stamp.secs+data.header.stamp.nsecs*1E-9
        Ut = self.mr.get_vel(accel, omega, time)
        self.Ks, self.Om = self.eif.prediction(Ut)
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
