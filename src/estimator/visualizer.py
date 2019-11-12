#plots and animations
####################

from IPython.core.debugger import set_trace
from importlib import reload
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation

class Visualizer:
    def __init__(self, xlim, ylim):
        self.xlim = xlim
        self.ylim = ylim

    def animator(self, Mu_hat, Xtr, M):

        #global variables
        self.xtr = Xtr[0]
        self.ytr = Xtr[1]
        self.thtr = Xtr[2]

        xhat = Mu_hat[:,0]
        yhat = Mu_hat[:,1]
        thhat = Mu_hat[:,2]

        #gather landmark locations
        mx1, my1, mx2, my2, mx3, my3, mx4, my4, mx5, my5, mx6, my6 = \
            M[0][0], M[1][0], M[0][1], M[1][1], M[0][2], M[1][2], \
            M[0][3], M[1][3], M[0][4], M[1][4], M[0][5], M[1][5]

        #initialize lists
        xtru, ytru, xdata, ydata, thdata, xpoint, ypoint = [], [], [], [], [], [], []

        #initialize animations
        fig, ax = plt.subplots()
        plt.axes(xlim=(-self.xlim/2,self.xlim/2), ylim=(-self.ylim/2,self.ylim/2))

        #static plots
        plt.plot(mx1,my1,'g^') #plot markers
        plt.plot(mx2,my2,'g^')
        plt.plot(mx3,my3,'g^')
        plt.plot(mx4,my4,'g^')
        plt.plot(mx5,my5,'g^')
        plt.plot(mx6,my6,'g^')
        plt.plot(xhat,yhat,'y') #plot estimated path
        plt.plot(self.xtr,self.ytr,'b') #plot true path

        #initialize animations elements
        robot, = plt.plot([], [], 'ro', markersize=12, animated=True)
        arrow, = plt.plot([], [], 'k*', markersize=3, animated=True)
        f = np.linspace(-3, 3, 200)

        def init():

            robot.set_data(xdata,ydata)
            arrow.set_data(xpoint, ypoint)

            return robot, arrow

        def update(frame):

            #gather the right elements
            i = int(frame)
            xdata = self.xtr[i]
            ydata = self.ytr[i]
            thdata = self.thtr[i]
            xpoint = xdata + np.cos(self.thtr[i])
            ypoint = ydata + np.sin(self.thtr[i])

            #set the animation elements
            robot.set_data(xdata, ydata)
            arrow.set_data(xpoint, ypoint)

            return robot, arrow

        #run the animation
        ani = animation.FuncAnimation(fig, update,
                            init_func=init, frames = 201, interval = 20, blit=True)
        plt.show()

    def plotter(self, Ks_hat, Mu_hat, Xtr, error, Sig_hat, Z_hat, Rtr, Phi_tr, t):

        #unpackage arrays
        ksx = Ks_hat[:,0]
        ksy = Ks_hat[:,1]
        ksth = Ks_hat[:,2]
        xhat = Mu_hat[:,0]
        yhat = Mu_hat[:,1]
        thhat = Mu_hat[:,2]
        xtr = Xtr[0]
        ytr = Xtr[1]
        thtr = Xtr[2]
        erx = error[:,0]
        ery = error[:,1]
        erth = error[:,2]
        Rhat = Z_hat[:,0]
        Phi_hat = Z_hat[:,1]
        t = t[0]

        #calculate upper and lower bounds for covariance plot
        sigx_hi = 2*np.sqrt(Sig_hat[:,0,0])
        sigx_lo = -2*np.sqrt(Sig_hat[:,0,0])
        sigy_hi = 2*np.sqrt(Sig_hat[:,1,1])
        sigy_lo = -2*np.sqrt(Sig_hat[:,1,1])
        sigth_hi = 2*np.sqrt(Sig_hat[:,2,2])
        sigth_lo = -2*np.sqrt(Sig_hat[:,2,2])

        #subplots for states vs. time
        fig1, aXk = plt.subplots(3)
        fig1.suptitle("Information Vector Components vs. Time")
        aXk[0].plot(t, ksx, label = "info x")
        aXk[0].legend(loc = "upper right")
        aXk[1].plot(t, ksy, label="info y")
        aXk[1].legend(loc = "upper right")
        aXk[2].plot(t, ksth, label = "info theta")
        aXk[2].legend(loc = "upper right")
        aXk[2].set_xlabel('time [s]')
        fig1.show()

        #subplots for states vs. time
        fig2, aXk = plt.subplots(3)
        fig2.suptitle("x, y, and theta vs. Time")
        aXk[0].plot(t, xhat, label = "est. x [m]")
        aXk[0].plot(t, xtr, label = "true x [m]")
        aXk[0].legend(loc = "upper right")
        aXk[1].plot(t, yhat, label="est. y [m]")
        aXk[1].plot(t, ytr, label="true y [m]")
        aXk[1].legend(loc = "upper right")
        aXk[2].plot(t, thhat, label = "est. theta [rad]")
        aXk[2].plot(t, thtr, label = "true theta [rad]")
        aXk[2].legend(loc = "upper right")
        aXk[2].set_xlabel('time [s]')
        fig2.show()

        fig3, aXk = plt.subplots(3)
        fig3.suptitle("Covariance & Error vs. Time")
        aXk[0].plot(t,erx, label="x error [m]")
        aXk[0].plot(t,sigx_hi, label="upper covariance")
        aXk[0].plot(t,sigx_lo, label="lower covariance")
        aXk[0].legend(loc = "upper right")
        aXk[1].plot(t,ery, label="y error [m]")
        aXk[1].plot(t,sigy_hi, label="upper covariance")
        aXk[1].plot(t,sigy_lo, label="lower covariance")
        aXk[1].legend(loc = "upper right")
        aXk[2].plot(t,erth, label="theta error [rad]")
        aXk[2].plot(t,sigth_hi, label="upper covariance")
        aXk[2].plot(t,sigth_lo, label="lower covariance")
        aXk[2].legend(loc = "upper right")
        aXk[2].set_xlabel('time [s]')
        fig3.show()

        ##only needed if generating measurment data
        # fig4, aXk = plt.subplots(2)
        # fig4.suptitle("Range and Bearing modeled and truth vs. Time")
        # aXk[0].plot(t, Rhat[:,0], label="Est. Range [m]")
        # aXk[0].plot(t, Rtr[:,0], label="True Range [m]")
        # aXk[0].legend(loc = "upper right")
        # aXk[1].plot(t, Phi_hat[:,0], label="Est. Bearing [rad]")
        # aXk[1].plot(t, Phi_tr[:,0], label="True Bearing [rad]")
        # aXk[1].legend(loc = "upper right")
        # aXk[1].set_xlabel('time [s]')
        # fig4.show()
    #