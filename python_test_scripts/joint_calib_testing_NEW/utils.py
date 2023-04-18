import numpy as np
from numpy import cos, sin, tan, pi
import matplotlib.pyplot as plt
def x_rotation(ang):
    R = np.array([
        [1,0,0],
        [0, cos(ang), -sin(ang)],
        [0, sin(ang), cos(ang)]])
    return R

def y_rotation(ang):
    R = np.array([
        [cos(ang),0,sin(ang)],
        [0 ,1, 0],
        [-sin(ang), 0, cos(ang)]])
    return R

def z_rotation(ang):
    R = np.array([
        [cos(ang),-sin(ang), 0],
        [sin(ang) ,cos(ang), 0],
        [0, 0, 1]])
    return R


def generate_true_data():
    """Generates the initial data following a 12 point measurement

    :return: Returns the tur g and m vectors
    :rtype: `tuple[np.ndarray, np.ndarray]`
    """
    g_samples = np.ndarray((3,12))
    m_samples = np.ndarray((3,12))
    g_vec = np.array([[0],[0],[1]])
    m_vec = np.array([[1],[0],[0]])
    x_rots = [0, 0 , 180, 180, 270, 270, 270, 90 , 0  , 0  , 0  , 0  ]
    y_rots = [0, 0 , 90 , 90 , 0  , 0  , 180, 0  , 270, 270, 90 , 90 ]
    z_rots = [0, 90, 0  , 45 , 270, 0  , 0  , 225, 90 , 180, 180, 225]
    for n in range(12):
        Rx = x_rotation(np.deg2rad(x_rots[n]))
        Ry = y_rotation(np.deg2rad(y_rots[n]))
        Rz = z_rotation(np.deg2rad(z_rots[n]))
        R = Rx @ Ry @ Rz
        g_samples[:,n] = (R @ g_vec)[:,0]
        m_samples[:,n] = (R @ m_vec)[:,0]

    # Show generated g data
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    # xs1 = g_samples[0,:]
    # ys1 = g_samples[1,:]
    # zs1 = g_samples[2,:]
    # ax.scatter(xs1, ys1, zs1, color="r")
    xs1 = m_samples[0,:]
    ys1 = m_samples[1,:]
    zs1 = m_samples[2,:]
    ax.scatter(xs1, ys1, zs1, color="b")
    plt.show()

    return g_samples, m_samples

def generate_sample_data(true_vec: np.ndarray, T: np.ndarray, h:np.ndarray, mean:float, var:float) -> np.ndarray:
    """Generates a set of realisitc data using an error matrix, T, a bias vector, h, and the mean and variance of the gaussian noise

    :param true_vec: Input data
    :type true_vec: np.ndarray
    :param T: 3x3 error matrix
    :type T: np.ndarray
    :param h: 3x1 bias vector
    :type h: np.ndarray
    :param mean: mean of gaussian white noise
    :type mean: float
    :param var: variance of gaussian white noise
    :type var: float
    :return: Output array
    :rtype: np.ndarray
    """
    N_NOISE = 5
    samples = np.ndarray((3,12*N_NOISE))
    for n in range(12):
        for m in range(N_NOISE):
            samples[:,n*N_NOISE + m] = ((T @ true_vec[:,n] + h.T).T + np.random.normal(mean,var,(3,1)))[:,0]
    return samples


