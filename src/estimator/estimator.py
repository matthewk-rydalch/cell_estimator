import rospy
import numpy as np
from importlib import reload, import_module
import math
from numpy.linalg import inv
from IPython.core.debugger import set_trace


cart = reload(import_module("cart"))
from cart import Cart
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
        N0 = 0.0 #m
        E0 = 0.0 #m
        th0 = 0.0 #rad
        #alitude does not change
        self.t_prev_imu = 0 #this is updated in imu callback #used to calculate dt

    	#my specified parameters and variables
        self.Sig = np.array([[1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, 0.0, 1.0]])

        self.Mu = np.array([[N0],[E0],[th0]])

        Sig_hist = []
        Mu_hist = []
        Ks_hist = []
        Z_hist = []

        #instantiate classes
        cart = Cart(sig_accel, sig_gyro, sig_gps)
        viz = Visualizer(xlim, ylim)
        Filter = Eif(mr.dyn_2d, mr.model_sensor, self.prediction_jacobians, self.measurement_jacobians, sig_accel, sig_gyro, sig_gps)
        self.mr = mr
        self.viz = viz
        self.Filter = Filter

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
        dt = time-self.t_prev_imu
        Ut = self.mr.get_vel(accel, omega, dt)
        self.Mu, self.Sig = self.Filter.prediction(Ut, self.Mu, self.Sig, dt)
        printer('got imu')

    def ned_callback(self, data):
        Zt = np.zeros((2,1))
        Zt[0] = data.relPosNED[0]
        Zt[1] = data.relPosNED[1]
        self.Mu, self.Sig = self.Filter.measure(self.Mu, self.Sig, Zt)
        printer('got ned')

    def lla_callback(self, data):
        #this line is simply to verify that messages are being subscribed to during development
        # printer("lla_callback \n")
        printer('got lla')

    def rover_RelPos_callback(self, data):
        #this line is simply to verify that messages are being subscribed to during development
        printer("rover_RelPos_callback \n")

    def prediction_jacobians(self, Ut, Mu, dt):

        vt = Ut[0]
        thp = Mu[2]
        jacob13 = -vt*math.sin(thp)*dt
        jacob12 = vt*math.cos(thp)*dt
        Gt = np.squeeze(np.array([[1.0, 0.0, jacob13[0]],\
                                [0.0, 1.0, jacob12[0]],\
                                [0.0, 0.0, 1.0]]))
        return Gt

    def measurement_jacobians(self, Mu_bar, Zt):
        #our model is linear
        Ht = np.array([[1.0, 0.0],\
                       [0.0, 1.0]])

        return Ht