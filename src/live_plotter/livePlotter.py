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
        print('\nStarting the plotter node\n')

        #Booleans to determine what the plot will show
        self.history = True#self.param['plot_history']
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

        # Set our plotting variables
        self.ned_x = []
        self.ned_y = []
        self.Mu_x = []
        self.Mu_y = []
        self.rover_x = []
        self.rover_y = []

        # Define the limits for the plot
        pos = 100
        neg = 100
        self.xmin = -neg#self.param['plotter_xmin']
        self.xmax = pos#self.param['plotter_xmax']
        self.ymin = -neg#self.param['plotter_ymin']
        self.ymax = pos#self.param['plotter_ymax']

        # Make the figure and axis for animation
        self.plot_update_interval = 100#self.param['plot_update_interval']
        self.fig = plt.figure()
        self.ax1 = self.fig.add_subplot(1,1,1)
        self.ani = animation.FuncAnimation(self.fig, self.plot_data, interval=self.plot_update_interval)

        # Define our subscribers
        rospy.Subscriber('NED', RelPos, self.ned_callback, queue_size=10)
        rospy.Subscriber('Mu', RelPos, self.Mu_callback, queue_size=10)
        rospy.Subscriber('/rover/RelPos', RelPos, self.rover_callback, queue_size=10)
        

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

        self.ax1.plot(self.ned_x, self.ned_y, 'r.')
        self.ax1.plot(self.Mu_x, self.Mu_y, 'b.')
        self.ax1.plot(self.rover_x, self.rover_y, 'g.')
        
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
        self.ned_y = msg.relPosNED[0]
        self.ned_x = msg.relPosNED[1]

    def Mu_callback(self, msg):
        self.Mu_y = msg.relPosNED[0]
        self.Mu_x = msg.relPosNED[1]

    def rover_callback(self, msg):
        self.rover_y = msg.relPosNED[0]
        self.rover_x = msg.relPosNED[1]


if __name__ == '__main__':
    rospy.init_node('plotter', anonymous=True)
    myPlotter = plotter()
