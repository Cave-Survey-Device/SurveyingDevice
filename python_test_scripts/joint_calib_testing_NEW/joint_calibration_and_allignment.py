import numpy as np
from numpy.linalg import inv
from numpy import pi
import utils
import matplotlib.pyplot as plt
import jcaa_funcs as jcaa
import mag_funcs2 as mag

# using "Accelerometer and Magnetometer Joint Calibration and Axes Alignment"

if __name__ == "__main__":
    """ --------------------- GENERATE INITIAL DATA ---------------------"""
    # Tsf = np.diag([-0.1,0.1,0]) # Scale factor error
    # Tcc = np.array([
    #     [0.05,0.1,0.15],
    #     [0.05,0.1,0.15],
    #     [0.05,0.1,0.15]
    # ]) # Cross coupling error
    # Ta = np.identity(3) + Tsf + Tcc
    # ha = np.array([[0.01],[-0.02],[0.03]])

    # Tsf = np.diag([0.9,1.1,0]) # Scale factor error
    # Tsi = np.identity(3) + np.array([
    #     [0.05,0.1,0.15],
    #     [0.05,0.1,0.15],
    #     [0.05,0.1,0.15]
    # ]) # Soft iron distortion
    # hhi = np.array([[0.3],[-0.2],[0.1]]) # Hard iron bias
    # hb = np.array([[-0.05],[0.25],[-0.16]]) # Offset
    # Tm = Tsf @ Tcc @ Tsi
    # hm = Tsf @ Tcc @ hhi + hb
    # print(Tm) 

    Ta = np.array([
        [9.7710,0.0018,-0.0030],
        [0.0019,9.7032,-0.0011],
        [-0.0087,-0.0013,9.6927]])
    ha = np.array([[-0.1472],[-0.0011],[0.1274]])

    # Tm = np.array([
    #     [0.4620,-0.0293,-0.0370],
    #     [0.0686,0.4379,0.0303],
    #     [0.0427,-0.0336,0.4369]])
    # hm = np.array([[-0.1760],[0.2214],[0.0398]])

    g_true, m_true = utils.generate_true_data()
    g_samples= utils.generate_sample_data(g_true,Ta,ha,0,0.000001)
    m_samples= utils.generate_sample_data(m_true,Tm,hm,0,0.000001)



    """ --------------------- PLOT INITIAL DATA ---------------------"""
    # Show generated g data
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    xs1 = g_true[0,:]
    xs2 = g_samples[0,:]
    ys1 = g_true[1,:]
    ys2 = g_samples[1,:]
    zs1 = g_true[2,:]
    zs2 = g_samples[2,:]

    ax.scatter(xs1, ys1, zs1, color="g")
    ax.scatter(xs2, ys2, zs2, color="r")
    plt.show()

    """ --------------------- BEGIN CALIBRATION AND ALLIGNMENT ---------------------"""
    # Generate initial guesses
    ym = m_samples
    M0 = ym
    ya = g_samples
    F0 = ya
    K = g_samples.shape[1]

    # Axis allignment matrix
    R0 = np.identity(3)
    d0 = np.deg2rad(68) # angle in Lancaster

    # Accelerometer calibration matrix
    Ha0 = np.identity(3)
    va0 = np.zeros((3,1))

    # Magnetometer calibration matrix
    Hm0, h0 = jcaa.claibrate_magnetometer(ym)
    #R0, h0 = mag.calibrate_mag(m_samples) # R = T^-1 = H, 
    vm0 = Hm0 @ h0

    print(inv(Tm))
    print(Hm0)

    """---------------------------------------------------"""

    """

    x = jcaa.composeX(Ha0,va0,F0,Hm0,vm0,M0,R0,d0)
    alpha = 0.95
    t  = 10
    error_arr = []
    while jcaa.J(x,ya,ym,K) > 0.01:
        error_arr.append(jcaa.J(x,ya,ym,K))
        grad = jcaa.grad_J(x,ya,ym,K)
        while jcaa.J(x,ya,ym,K) < jcaa.J(x+t*grad,ya,ym,K):
            t = t * alpha

        x = x - t * grad
        error_arr.append(jcaa.J(x,ya,ym,K))
    """

