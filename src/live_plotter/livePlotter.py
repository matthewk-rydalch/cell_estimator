#!/usr/bin/env python3

import rospy
# from builtins import range
import numpy as np
import collections as cl
from cell_estimator.msg import RelPos
import matplotlib
matplotlib.use('TKAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation


class plotter():

	def __init__(self):
		# get parameters
		# try:
		# 	param_namespace = '/latis'
		# 	self.param = rospy.get_param(param_namespace)
		# except KeyError:
		# 	rospy.logfatal('Parameters not set in ~/latis namespace')
		# 	rospy.signal_shutdown('Parameters not set')

		print('\nStarting the plotter node\n')

		#Booleans to determine what the plot will show
		self.history = False#self.param['plot_history']
		self.tail = False#self.param['tail']
		self.tail_length_r = 5#self.param['tail_length']
		self.tail_length_m = 5#self.tail_length_r
		self.units = 1#self.param['radar_units']

		# Stuff for plotting covariance ellipses around RRANSAC tracks
		self.Plot_Ellipses = False#self.param['plot_ellipses']
		if self.Plot_Ellipses:
			circle = np.arange(0., 2 * np.pi, 0.1)
			self.x_points_cir = np.cos(circle)
			self.y_points_cir = np.sin(circle)
			self.cir_points = np.vstack([self.x_points_cir, self.y_points_cir])
			self.ellipse = self.cir_points
			# self.positions = []
			# self.positions.append(self.param['r0_location'])
			# self.positions.append(self.param['r1_location'])
			# self.positions.append(self.param['r2_location'])
			# self.positions.append(self.param['r3_location'])
			# self.radar_num = 0


		# Set our plotting variables
		self.r_x = []
		self.r_y = []
		self.m_x = []
		self.m_y = []

		# Make queues for the tails to plot if self.tail is true
		if self.tail:
			self.qr_x = cl.deque(maxlen=self.tail_length_r)
			self.qr_y = cl.deque(maxlen=self.tail_length_r)
			self.qm_x = cl.deque(maxlen=self.tail_length_m)
			self.qm_y = cl.deque(maxlen=self.tail_length_m)


		# Define the limits for the plot
		self.xmin = -30#self.param['plotter_xmin']
		self.xmax = 10#self.param['plotter_xmax']
		self.ymin = -30#self.param['plotter_ymin']
		self.ymax = 10#self.param['plotter_ymax']

		# Make the figure and axis for animation
		self.plot_update_interval = 100#self.param['plot_update_interval']
		self.fig = plt.figure()
		self.ax1 = self.fig.add_subplot(1,1,1)
		self.ani = animation.FuncAnimation(self.fig, self.plot_data, interval=self.plot_update_interval)

		# Define our subscribers
		rospy.Subscriber('NED', RelPos, self.ned_callback, queue_size=10)
		# self.rransac_data_sub_ = rospy.Subscriber('/Radarsim0/rransac_model_states', model_array, self.rransac_states_update_callback0, queue_size=10)

		# Start the Plotting
		plt.show()

		#self.tracks_pub_ = rospy.Publisher('rransac_tracks', tracks, queue_size=1024)
		while not rospy.is_shutdown():
			# wait for new messages and call the callback when they arrive
			rospy.spin()


	def plot_data(self, i):

		# if self.BASE_FOUND:
		if self.history == False:
			self.ax1.clear()

		if self.tail:
			self.ax1.plot(list(self.qr_x), list(self.qr_y), 'r*')
			self.ax1.plot(list(self.qm_x), list(self.qm_y), 'b.')
		else:
			self.ax1.plot(self.r_x, self.r_y, 'r*')
			self.ax1.plot(self.m_x, self.m_y, 'b.')
			############################################################################
			# Stuff for plotting ellipses
			if self.Plot_Ellipses:

				self.ax1.plot(self.ellipse[0], self.ellipse[1], color='green')

		self.ax1.plot(0, 0, 'k+')

		self.ax1.set_ylim(self.ymin, self.ymax)
		self.ax1.set_xlim(self.xmin, self.xmax)
		self.ax1.set_ylabel('y position wrt base station')
		self.ax1.set_xlabel('x position wrt base station')


	def ned_callback(self, msg):
		first = True
		print("msg", msg.relPosNED)
		# pos = np.matrix([0, 0, 0])
		# data = msg.data
		# for NN in range(len(data)):
		# 	p = np.matrix([data[NN].N, data[NN].E, data[NN].D])
		# 	if first:
		# 		pos = p
		# 		first = False
		# 	else:
		# 		pos = np.vstack([pos,p])
		if self.tail:
			ys = msg.relPosNED[0]
			xs = msg.relPosNED[1]
			# for NN in range(len(xs)):
			self.qr_x.append(xs)#.item(NN))
			self.qr_y.append(ys)#.item(NN))
		else:
			self.r_y = msg.relPosNED[0]
			self.r_x = msg.relPosNED[1]
		# if self.tail == False:
		# 	self.r_x = []
		# 	self.r_y = []


	# def rransac_states_update_callback0(self, msg):
	# 		xs = []
	# 		ys = []
	#
	# 		# Iterate through and save the x and y positions for the models
	# 		for NN in range(0, msg.models):
	# 			###########################################################################3
	# 			ys = np.append(ys, msg.modelArray[NN].position[1])
	# 			xs = np.append(xs, msg.modelArray[NN].position[0])
	# 			############################################################################
	# 			# Stuff for plotting ellipses
	# 			if self.Plot_Ellipses:
	# 				P = np.reshape(msg.modelArray[NN].P,(8,8))
	# 				print("P:", P)
	# 				u, s, v = np.linalg.svd(P[0:2, 0:2])
	# 				c = u * 2 *np.sqrt(s)
	# 				R = np.array([[np.cos(self.positions[0][3]), -np.sin(self.positions[0][3])],
	# 				[np.sin(self.positions[0][3]), np.cos(self.positions[0][3])]])
	# 				print("R:", R)
	# 				self.ellipse = R @ c @ self.cir_points
	# 				self.ellipse[0]+=xs
	# 				self.ellipse[1]-=ys
	# 			############################################################################
	#
	# 		# Save the x and y values according to the plotting criteria
	# 		if self.tail:
	# 			for NN in range(len(xs)):
	# 				self.qm_x.append(xs[NN])
	# 				self.qm_y.append(-1*ys[NN])
	# 			self.m_x = list(self.qm_x)
	# 			self.m_y = list(self.qm_y)
	# 		else:
	# 			self.m_x = xs
	# 			self.m_y = -1*ys
	#
	#
	# def rransac_states_update_callback1(self, msg):
	# 		xs = []
	# 		ys = []
	#
	# 		# Iterate through and save the x and y positions for the models
	# 		for NN in range(0, msg.models):
	# 			###########################################################################3
	# 			ys = np.append(ys, msg.modelArray[NN].position[1])
	# 			xs = np.append(xs, msg.modelArray[NN].position[0])
	# 			############################################################################
	# 			# Stuff for plotting ellipses
	# 			if self.Plot_Ellipses:
	# 				P = np.reshape(msg.modelArray[NN].P,(8,8))
	# 				print("P:", P)
	# 				u, s, v = np.linalg.svd(P[0:2, 0:2])
	# 				c = u * 2 *np.sqrt(s)
	# 				R = np.array([[np.cos(self.positions[1][3]), -np.sin(self.positions[1][3])],
	# 				[np.sin(self.positions[1][3]), np.cos(self.positions[1][3])]])
	# 				print("R:", R)
	# 				self.ellipse = R @ c @ self.cir_points
	# 				self.ellipse[0]+=xs
	# 				self.ellipse[1]-=ys
	# 			############################################################################
	#
	# 		# Save the x and y values according to the plotting criteria
	# 		if self.tail:
	# 			for NN in range(len(xs)):
	# 				self.qm_x.append(xs[NN])
	# 				self.qm_y.append(-1*ys[NN])
	# 			self.m_x = list(self.qm_x)
	# 			self.m_y = list(self.qm_y)
	# 		else:
	# 			self.m_x = xs
	# 			self.m_y = -1*ys
	#

if __name__ == '__main__':
	rospy.init_node('plotter0', anonymous=True)
	myPlotter = plotter()
