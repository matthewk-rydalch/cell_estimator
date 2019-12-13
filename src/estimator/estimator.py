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
        sig_gps = 3.0 #m #sensor values are rough estimates from https://www.ncbi.nlm.nih.gov/pmc/articles/PMC5017405/
        sig_accel = 0.4 #m/s^2
        sig_gyro = 1.0 #deg/s^2
        sig_gyro = sig_gyro*np.pi/180 #rad/s^2
        N0 = 0.0 #m
        E0 = 0.0 #m
        th0 = -np.pi/2 #rad
        #alitude does not change
        self.t_prev_imu = 0 #this is updated in imu callback #used to calculate dt

    	#my specified parameters and variables
        self.Sig = np.array([[1.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0],
                            [0.0, 0.0, 1.0]])

        self.Mu = np.array([[N0],[E0],[th0]])

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
        # print("MU propagation:", self.Mu)
        self.mu_publisher()
        #visualization()
        printer('got imu')

    def ned_callback(self, data):
        Zt = np.zeros((2,1))
        Zt[0] = data.relPosNED[0]
        Zt[1] = data.relPosNED[1]
        time = data.header.stamp.secs+data.header.stamp.nsecs*1E-9
        # print('sigma = ', self.Sig)
        self.Mu, self.Sig = self.Filter.measure(self.Mu, self.Sig, Zt)
        # print("MU measurement:", self.Mu)
        # print('sigma post= ', self.Sig)
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

    def mu_publisher(self):
        MU = RelPos()
        MU.relPosNED[0] = self.Mu[0]
        MU.relPosNED[1] = self.Mu[1]
        MU.relPosNED[2] = self.Mu[2]
        self.pub_Mu.publish(MU)

