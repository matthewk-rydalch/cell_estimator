import rospy
from cell_estimator.msg import RelPos
import numpy as np
from importlib import reload, import_module
import math
from numpy.linalg import inv
from IPython.core.debugger import set_trace


cart = reload(import_module("cart"))
from cart import Cart
visualizer = reload(import_module("visualizer"))
from visualizer import Visualizer
ekf = reload(import_module("ekf"))
from ekf import Ekf


DEBUG = False
def printer(statement):
    if DEBUG:
        printer(statement)

class Estimator():
    def __init__(self):

        #parameters
        xlim = 30.0 #m
        ylim = 30.0 #m
        sig_gps = 0.5 #m #sensor values are rough estimates from https://www.ncbi.nlm.nih.gov/pmc/articles/PMC5017405/
        sig_accel = 1.0#0.4 #m/s^2
        sig_gyro = 1.0#1.0 #deg/s^2
        sig_gyro = sig_gyro*np.pi/180 #rad/s^2
        N0 = 0.0 #m
        E0 = 0.0 #m
        self.th0 = -np.pi/2 #rad
        #alitude does not change
        self.t_prev_imu = 0 #this is updated in imu callback #used to calculate dt
        self.t_start = 0
        self.prop_first = True
        self.mu_first = True

    	#my specified parameters and variables
        self.Sig = np.array([[1.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0],
                            [0.0, 0.0, 1.0]])

        self.Mu = np.array([[N0],[E0],[self.th0]])

        self.Sig_hist = []
        self.Mu_hist = []
        self.cell_time_hist = []
        self.rover_time_hist = []
        self.rover_pos_hist = []

        #instantiate classes
        cart = Cart(sig_accel, sig_gyro, sig_gps)
        viz = Visualizer(xlim, ylim)
        Filter = Ekf(cart.dyn_2d, cart.model_sensor, self.prediction_jacobians, self.measurement_jacobians, sig_accel, sig_gyro, sig_gps)
        self.cart = cart
        self.viz = viz
        self.Filter = Filter

        # ROS stuff (sorry I did't want to try and figure out a better way to do this)
        self.pub_Mu = rospy.Publisher('Mu', RelPos, queue_size=1024)
        # self.pub_NED = rospy.Publisher('NED', RelPos, queue_size=1024)


    def imu_callback(self, data):
        accel_x = data.linear_acceleration.x
        accel_y = data.linear_acceleration.y
        accel_z = data.linear_acceleration.z
        accel = np.array([[accel_x],[accel_y],[accel_z]])
        omega_x = data.angular_velocity.x
        omega_y = data.angular_velocity.y
        omega_z = -data.angular_velocity.z
        omega = np.array([[omega_x],[omega_y],[omega_z]])
        # set_trace()
        time = data.header.stamp.secs+data.header.stamp.nsecs*1E-9
        if self.prop_first:
            self.t_start = time
            self.prop_first = False
        if time - self.t_start > 6.0:
            if self.t_prev_imu != 0.0:
                dt = (time-self.t_prev_imu)
            else:
                dt = 0.003
            self.t_prev_imu = time
            Ut = self.cart.get_vel(accel, omega, dt)
            # print("MU propagation before:", self.Mu)
            self.Mu, self.Sig = self.Filter.prediction(Ut, self.Mu, self.Sig, dt)
            # print("MU propagation after:", self.Mu)
            self.Mu_hist.append(self.Mu)
            self.Sig_hist.append(self.Sig)
            self.cell_time_hist.append(time)
            self.mu_publisher()
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
        print("MU measurement before:", self.Mu)
        self.Mu, self.Sig = self.Filter.measure(self.Mu, self.Sig, Zt)
        print("MU measurement after:", self.Mu)
        self.Mu_hist.append(self.Mu)
        self.Sig_hist.append(self.Sig)
        self.cell_time_hist.append(time)
        self.mu_publisher()
        printer('got ned')

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
        # jacob13 = -vt*math.sin(thp)*dt
        # jacob12 = vt*math.cos(thp)*dt
        Gt = np.squeeze(np.array([[1.0, 0.0, 0.0],\
                                [0.0, 1.0, 0.0],\
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

