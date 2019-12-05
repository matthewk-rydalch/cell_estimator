#!/usr/bin/env python3

#Driver for eif
#################################
#header
from IPython.core.debugger import set_trace
import numpy as np
import os
import scipy
import scipy.io as sio
from importlib import reload, import_module
import math
from numpy.linalg import inv
import rospy

multirotor = reload(import_module("multirotor"))
from multirotor import Multirotor
visualizer = reload(import_module("visualizer"))
from visualizer import Visualizer
eif = reload(import_module("eif"))
from eif import Eif
utils = reload(import_module("utils"))

from cell_estimator.msg import imu
from cell_estimator.msg import RelPos
from cell_estimator.msg import PositionVelocityTime
from ublox.msg import PositionVelocityTime
from ublox.msg import RelPos
from estimator import Estimator

#################################
def driver():

	#parameters
	xlim = 30.0 #m
	ylim = 30.0 #m
	sig_r = 0.2 #m
	sig_phi = 0.1 #rad
	sig_v = 0.15 #m/s
	sig_w = 0.1 #rad/s
	x0 = -5.0 #m
	y0 = 0.0 #m
	th0 = 90.0 #deg!!!!
	th0 = th0*math.pi/180
	#alitude does not change

	####need to get these parameters instead of using the parameters below.
	# Xtr = self.Xtr #from rover/RelPos
	# Rtr = self.Rtr #different measurement model.  Will come from cell/NED.  Make this N? range truth
	# Phi_tr = self.Phi_tr #different measurement model.  Will come from cell/NED.  Make this E? bearing truth
	# M = self.M #not needed for NE measurements.  Could use the base?
	# V = self.V #velocity comes from integrating accel from imu
	# vc = self.vc
	# W = self.W #angular velocity comes from integrating angular accel from imu
	# wc = self.wc
	# Uc = self.Uc #just a combination of the two.
	# Time = self.Time #comes from ROS?

	#read in given data
	path = rospy.get_param('/estimator/path')
	given = sio.loadmat(path + '/src/estimator/midterm_data.mat')
	Xtr = given['X_tr']
	Rtr = given['range_tr']
	Phi_tr = given['bearing_tr']
	M = given['m'].astype('float')
	V = given['v']
	vc = given['v_c']
	W = given['om']
	wc = given['om_c']
	Uc = np.squeeze(np.array([vc,wc]))
	Time = given['t']
	dt = 0.1 #seconds, this correlates with given Time
	tf = 30.0 #seconds, this correlates with given Time

	#my specified parameters and variables
	Sig = np.array([[1.0, 0.0, 0.0],\
					[0.0, 1.0, 0.0],\
					[0.0, 0.0, 1.0]])

	Mu = np.array([[x0],[y0],[th0]])

	Sig_hist = []
	Mu_hist = []
	Ks_hist = []
	Z_hist = []

	#instantiate classes
	mr = Multirotor(sig_v, sig_w, sig_r, sig_phi, dt, M)
	viz = Visualizer(xlim, ylim)
	eif = Eif(mr.dyn_2d, mr.model_sensor, dt, sig_v, sig_w, sig_r, sig_phi, M)

	#loop through algorithm for each time step
	for i in range(len(Time[0])):
		Om = inv(Sig)
		Ks = Om@Mu
		# Ut = mr.get_vel(Uc[:,i]) #use the given velocities
		# Zt = mr.model_sensor(Mu) #use the given measurements
		Ut = np.array([vc[0][i],wc[0][i]])
		Zt = np.array([Rtr[i],Phi_tr[i]])
		Ks, Om = eif.ext_info_filter(Ks, Om, Ut, Zt)
		Sig = inv(Om)
		Mu = Sig@Ks
		# Mu[2] = utils.wrap(Mu[2]) #could be wrapped, but to match true theta, don't
		Ks_hist.append(Ks)
		Sig_hist.append(Sig)
		Mu_hist.append(Mu)
		Z_hist.append(Zt)

	Ks_hat = np.squeeze(np.array(Ks_hist))
	Mu_hat = np.squeeze(np.array(Mu_hist))
	Sig_hat = np.squeeze(np.array(Sig_hist))
	Z_hat = np.squeeze(np.array(Z_hist))

	error = Xtr.T - Mu_hat
	error[:,2] = utils.wrap(error[:,2])

	viz.plotter(Ks_hat, Mu_hat, Xtr, error, Sig_hat, Z_hat, Rtr, Phi_tr, Time)
	viz.animator(Mu_hat, Xtr, M)

def ros_estimator():
	rospy.init_node('estimator', anonymous=True)
	rospy.Subscriber('cell/imu', imu, est.imu_callback)
	rospy.Subscriber('cell/NED', RelPos, est.ned_callback)
	rospy.Subscriber('cell/lla', PositionVelocityTime, est.lla_callback)
	rospy.Subscriber('rover/RelPos', RelPos, est.rover_RelPos_callback)

	rospy.spin()
#

if __name__ == '__main__':

	# driver()
	est = Estimator()
	ros_estimator()
#
