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
plotter = reload(import_module("plotter"))
from plotter import Plotter

from cell_estimator.msg import imu
from cell_estimator.msg import RelPos
from cell_estimator.msg import PositionVelocityTime
from ublox.msg import PositionVelocityTime
from ublox.msg import RelPos
from estimator import Estimator

def ros_estimator():
	rospy.init_node('estimator', anonymous=True)
	rospy.Subscriber('imu', imu, est.imu_callback)
	rospy.Subscriber('NED', RelPos, est.ned_callback)
	rospy.Subscriber('lla', PositionVelocityTime, est.lla_callback)
	rospy.Subscriber('rover/RelPos', RelPos, est.rover_RelPos_callback)

	rospy.spin()
	plt.plotter()
#

if __name__ == '__main__':

	# driver()
	plt = Plotter()
	est = Estimator()
	ros_estimator()
#
