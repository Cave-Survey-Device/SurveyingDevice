import numpy as np
from numpy.linalg import inv
from numpy import pi
import utils
import matplotlib.pyplot as plt

if __name__ == "__main__":
    """ --------------------- GENERATE INITIAL DATA ---------------------"""
    Tsf = np.diag([-0.1,0.1,0]) # Scale factor error
    Tcc = np.array([
        [0.05,0.1,0.15],
        [0.05,0.1,0.15],
        [0.05,0.1,0.15]
    ]) # Cross coupling error
    Ta = np.identity(3) + Tsf @ Tcc
    print(Ta)
    
    ha = np.array([[0.01],[-0.02],[0.03]])


    Tsi = np.identity(3) + np.array([
        [0.05,0.1,0.15],
        [0.05,0.1,0.15],
        [0.05,0.1,0.15]
    ]) # Soft iron distortion
    hhi = np.array([[0.3],[-0.2],[0.1]]) # Hard iron bias
    hb = np.array([[-0.05],[0.25],[-0.16]]) # Offset
    Tm = Tsf @ Tcc @ Tsi
    hm = Tsf @ Tcc @ hhi + hb
    print(Tm)

    g_true, m_true = utils.generate_true_data()
    g_samples= utils.generate_sample_data(g_true,Ta,ha,0,0.000001)
    m_samples= utils.generate_sample_data(g_true,Tm,hm,0,0.000001)



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


   