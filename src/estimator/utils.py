#Utilities
##################

import numpy as np
from IPython.core.debugger import set_trace


r2d = 180.0 / np.pi
d2r = np.pi / 180.0
inv_360 = 1.0 / 360.0
inv_180 = 1.0 / 180.0
inv_pi = 1.0 / np.pi
inv_2pi = 0.5 / np.pi

def deg_wrap_180( angle ):
    """wrap an angle in degrees, -180 <= theta < 180"""
    angle -= 360.0 * np.floor((angle + 180.) * inv_360)
    return angle
#
def deg_wrap_360( angle ):
    """wrap an angle in degrees, 0 <= theta < 360"""
    angle -= 360.0 * np.floor(angle * inv_360)
    return angle
#

# def rad_wrap_pi( angle ):
def wrap( angle ):
    # """wrap an angle in rads, -pi <= theta < pi"""
    # set_trace()
    angle -= 2*np.pi * np.floor((angle + np.pi) * inv_2pi)
    return angle
def wrapf( angle ):
    #just using this to find bugs
    return angle
#
def rad_wrap_2pi( angle ):
    """wrap an angle in rads, 0 <= theta < 2*pi"""
    angle -= 2*np.pi * np.floor(angle * inv_2pi)
    return angle
#

def low_var_sampler(xt, wt, n): #n is the number of states
    M = xt.shape[1]
    xbar = np.zeros((M,3)) #phi in algorithm
    r = np.random.uniform(0, 1.0/M, size=None) #starting point for comb
    c = wt[0]
    i = 0
    for m in range(M):
        U = r+m/M #comb brush
        while U>c:
            i = i+1
            c = c+wt[i] #go to next weight
        xbar[m] = xt[:,i]

    # #combating particle deprivation
    # P = np.cov(xbar) # covariance of prior
    # dupl = 0
    # for i in range(1,M):
    #     if xbar[i] != xbar[i-1]:
    #         dupl = dupl+1
    # uniq = M - dupl # number of unique particle in resampled cloud
    # if uniq/M < 0.5: # if there is a lot of duplication
    #     Q = P/((M*uniq)**(1/n)); # add noise to the samples
    #     xbar = xbar + Q*randn(size(x));

    return(xbar.T)