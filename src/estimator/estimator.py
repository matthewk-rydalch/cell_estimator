import rospy
from cell_estimator.msg import RelPos
from cell_estimator.msg import imu
import numpy as np
from importlib import reload, import_module
import math
from numpy.linalg import inv
from IPython.core.debugger import set_trace
import val
import time


cart = reload(import_module("cart"))
from cart import Cart
visualizer = reload(import_module("visualizer"))
from visualizer import Visualizer
ekf = reload(import_module("ekf"))
from ekf import Ekf


DEBUG = False
def printer(statement, statement2=''):
    if DEBUG:
        printer(statement, statement2)

class Estimator():
    def __init__(self):

        #parameters
        xlim = 30.0 #m
        ylim = 30.0 #m
        sig_gps = 4.0 #m #sensor values are rough estimates from https://www.ncbi.nlm.nih.gov/pmc/articles/PMC5017405/
        sig_gps_heading = 0.01 #m #sensor values are rough estimates from https://www.ncbi.nlm.nih.gov/pmc/articles/PMC5017405/
        sig_accel = 5.0 #m/s^2
        sig_gyro = 10.0 * val.d2r

        self.omega_bound = 10.0
        self.gyro_high = sig_gyro
        self.gyro_low = sig_gyro
        self.gps_heading_high = sig_gps_heading
        self.gps_heading_low = sig_gps_heading
        self.gps_high = sig_gps
        self.gps_low = sig_gps
        # self.omega_bound = 0.02
        # self.gyro_high = 1.0 * val.d2r
        # self.gyro_low = 10.0 * val.d2r
        # self.gps_heading_high = 5.0
        # self.gps_heading_low = 20.0
        # self.gps_high = sig_gps-0.05
        # self.gps_low = sig_gps


        N0 = 0.0 #m
        E0 = 0.0 #m
        self.th0 = -np.pi/2 #rad
        #alitude does not change
        self.t_prev_imu = 0 #this is updated in imu callback #used to calculate dt
        self.t_start = 0
        self.prop_first = True
        self.mu_first = True

    	#my specified parameters and variables
        self.Sig = np.diag([1.0,1.0,1.0])*1e2
        # self.Sig = np.array([[1.0, 0.0, 0.0],
        #                     [0.0, 1.0, 0.0],
        #                     [0.0, 0.0, 1.0]])

        self.Mu = np.array([[N0],[E0],[self.th0]])

        self.Sig_hist = []
        self.Mu_hist = []
        self.cell_time_hist = []
        self.rover_time_hist = []
        self.rover_pos_hist = []

        #instantiate classes
        cart = Cart(sig_accel, sig_gyro, sig_gps)
        viz = Visualizer(xlim, ylim)
        Filter = Ekf(cart.dyn_2d, cart.model_sensor, self.prediction_jacobians, self.measurement_jacobians, sig_accel, sig_gyro, sig_gps, sig_gps_heading)
        self.cart = cart
        self.viz = viz
        self.Filter = Filter

        # ROS stuff (sorry I did't want to try and figure out a better way to do this)
        self.pub_Mu = rospy.Publisher('Mu', RelPos, queue_size=1024)
        self.pub_SIG = rospy.Publisher('SIG', imu, queue_size=1024)


    def imu_callback(self, data):
        accel_x = data.linear_acceleration.x
        accel_y = data.linear_acceleration.y
        accel_z = data.linear_acceleration.z
        accel = np.array([[accel_x],[accel_y],[accel_z]])
        omega_x = data.angular_velocity.x
        omega_y = data.angular_velocity.y
        omega_z = -data.angular_velocity.z
        omega = np.array([[omega_x],[omega_y],[omega_z]])
        if np.abs(omega_z)>self.omega_bound:
            self.Filter.sig_gyro = self.gyro_high
            self.Filter.sig_gps_heading = self.gps_heading_low
            self.Filter.sig_gps = self.gps_low
        else:
            self.Filter.sig_gyro = self.gyro_low
            self.Filter.sig_gps_heading = self.gps_heading_high
            self.Filter.sig_gps = self.gps_high
        # set_trace()
        time = data.header.stamp.secs+data.header.stamp.nsecs*1E-9
        if self.prop_first:
            self.t_start = time
            self.prop_first = False
        if time - self.t_start > 7.0:
            if self.t_prev_imu != 0.0:
                dt = (time-self.t_prev_imu)
            else:
                dt = 0.003
            self.t_prev_imu = time
            Ut = self.cart.get_vel(accel, omega, dt)
            self.Mu, self.Sig = self.Filter.prediction(Ut, self.Mu, self.Sig, dt)
            self.Mu_hist.append(self.Mu)
            self.Sig_hist.append(self.Sig)
            self.cell_time_hist.append(time)
            self.mu_publisher()
            self.SIG_publisher()
            #visualization()
            printer('got imu')

    def ned_callback(self, data):
        Zt = np.zeros((3,1))
        Zt[0] = data.relPosNED[0]
        Zt[1] = data.relPosNED[1]
        if self.mu_first:
            Zt[2] = -np.pi/2.0
            self.Mu = Zt
            self.mu_first = False
        time = data.header.stamp.secs+data.header.stamp.nsecs*1E-9
        self.Mu, self.Sig = self.Filter.measure(self.Mu, self.Sig, Zt)
        self.Sig_hist.append(self.Sig)
        self.cell_time_hist.append(time)
        self.mu_publisher()
        self.SIG_publisher()

    def lla_callback(self, data):
        #this line is simply to verify that messages are being subscribed to during development
        # printer("lla_callback \n")
        printer('got lla')

    def rover_RelPos_callback(self, data):
        time = data.header.stamp.secs+data.header.stamp.nsecs*1E-9
        self.rover_time_hist.append(time)
        Pt = [0]*2
        Pt[0] = data.relPosNED[0]
        Pt[1] = data.relPosNED[1]
        self.rover_pos_hist.append(Pt)
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
        Ht = np.eye(3)
        # Ht = np.array([[1.0, 0.0],\
        #                [0.0, 1.0]])

        return Ht

    def mu_publisher(self):
        MU = RelPos()
        MU.relPosNED[0] = self.Mu[0]
        MU.relPosNED[1] = self.Mu[1]
        MU.relPosNED[2] = self.Mu[2]
        self.pub_Mu.publish(MU)

    def SIG_publisher(self):
        sig = imu()
        sig.orientation_covariance = self.Sig.flatten()
        self.pub_SIG.publish(sig)
