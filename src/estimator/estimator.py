import rospy
DEBUG = False
def printer(statement):
    if DEBUG:
        printer(statement)

class Estimator():
    # def __init__():

    #     #parameters
    #     self.xlim = 30.0 #m
    #     self.ylim = 30.0 #m
    #     self.sig_r = 0.2 #m
    #     self.sig_phi = 0.1 #rad
    #     self.sig_v = 0.15 #m/s
    #     self.sig_w = 0.1 #rad/s
    #     self.N0 = 0.0 #m
    #     self.E0 = 0.0 #m
    #     #alitude does not change

    # 	#my specified parameters and variables
    #     self.Sig = np.array([[1.0, 0.0, 0.0],
    #                         [0.0, 1.0, 0.0],
    #                         [0.0, 0.0, 1.0]])

    #     self.Mu = np.array([[self.x0],[self.y0],[self.th0]])

    #     self.Sig_hist = []
    #     self.Mu_hist = []
    #     self.Ks_hist = []
    #     self.Z_hist = []

    #     #instantiate classes
    #     self.mr = Multirotor(sig_v, sig_w, sig_r, sig_phi, dt, M)
    #     self.viz = Visualizer(xlim, ylim)
    #     self.eif = Eif(mr.dyn_2d, mr.model_sensor, dt, sig_v, sig_w, sig_r, sig_phi, M)

    #     #convert to information form
    #     self.Om = inv(Sig)
    #     self.Ks = Om@Mu

    def imu_callback(self, data):
        # Ut = get_vel(data, time)
        # self.Ks, self.Om = propagate(Ut)
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

    # def propagate(self, Ut):
    #     #prediction step
    #     Mup = inv(Omp)@Ksp
    #     # Mup[2] = utils.wrap(Mup[2]) #could be wrapped, but to match true theta, don't
    #     thp = Mup[2]
    #     Gt, Rt= self.mr.propogation_matrices(Ut, thp)
    #     Omg_bar = inv(Gt@inv(Omp)@Gt.T+Rt)
    #     g_function = self.dyn_2d(Mup, Ut) #g is needed for both prediction and measurement
    #     Ks_bar = Omg_bar@g_function

    #     Ks = Ks_bar
    #     Omg = Omg_bar

    #     return Ks, Omg

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
