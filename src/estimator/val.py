import numpy as np

r2d = 180.0 / np.pi
d2r = np.pi / 180.0

e_smaj = 6378137.0      # length of Earth's semi-major axis in meters
e_smin = 6356752.314245 # length of Earth's semi-minor axis in meters

e_frst_num_ecc = 1.0 - pow((e_smin / e_smaj), 2)    # first numerical eccentricity

def RI2B(state):
    # returns the transformation inertial to body frame
    # ======================================
    # first check if state was given in quaternion values
    # use appropriate i2b calculation

    if np.size(state,0) == 4:
        return RI2Bq(state)
    elif np.size(state,0) == 3:
        return RI2Be(state)
    else:
        print("Size error. Size not equal to 4 or 3.\nOr possibly a shape error.")

# ======================================

def RI2Be(stateEul):
    # hold current phi, theta, and psi
    phi     = stateEul[0,0]
    theta   = stateEul[1,0]
    psi     = stateEul[2,0]

    # ======================================

    # # trig abbreviations
    cph = np.cos(phi)
    sph = np.sin(phi)
    # tph = tan(phi)

    cth = np.cos(theta)
    sth = np.sin(theta)
    # tth = tan(theta);

    cps = np.cos(psi)
    sps = np.sin(psi)
    # tps = tan(psi);

    ri2bE   = np.matrix([[cth*cps, cth*sps, -sth],[sph*sth*cps - cph*sps, sph*sth*sps + cph*cps, sph*cth],[cph*sth*cps + sph*sps, cph*sth*sps - sph*cps, cph*cth]])

    return ri2bE
    # # tb2i = rpyI2B'

# ======================================

def RI2Bq(stateQuat):
    print("Error. Size equal to 4.")
    # q0 = stateQuat(4, 1)
    # q1 = stateQuat(5, 1)
    # q2 = stateQuat(6, 1)
    # q3 = stateQuat(7, 1)
    #
    # # ======================================
    # ri2bQ   = [ q0^2+q1^2-q2^2-q3^2,    2*(q1*q2+q0*q3),        2*(q1*q3-q0*q2);
    #             2*(q1*q2-q0*q3),        q0^2-q1^2+q2^2-q3^2,    2*(q2*q3+q0*q1);
    #             2*(q1*q3+q0*q2),        2*(q2*q3-q0*q1),        q0^2-q1^2-q2^2+q3^2 ];
    # #
