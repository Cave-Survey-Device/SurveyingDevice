import numpy as np
from numpy import kron
from numpy.linalg import inv, cholesky, norm
import utils
import matplotlib.pyplot as plt
import mag_funcs2 as mag

def generate_data(Tm: np.ndarray, hm: np.ndarray):
    m_true = np.array([[1],[0],[0]])

    n=0
    n_y = 10
    n_z = 20
    true = np.ndarray((3,n_y*n_z))
    samples = np.ndarray((3,n_y*n_z))
    for y_ang in range(0,360,int(360/n_y)):
        for z_ang in range(0,360,int(360/n_z)):
            T = utils.y_rotation(np.deg2rad(y_ang)) @ utils.z_rotation(np.deg2rad(z_ang))
            true[:,n] = T @ m_true[:,0]
            samples[:,n] = (Tm @ true[:,n] + hm.T).T[:,0]
            n+=1

    return true, samples

def calculate_Cr(m_samples:np.ndarray):
    K = m_samples.shape[1]
    Y = m_samples

    k=0
    x = Y[0,k]
    y = Y[1,k]
    z = Y[2,k]
    D = np.array([x**2,y**2,z**2,x*y,y*z,x*z,x,y,z,1])
    for k in range(1,K):
        x = Y[0,k]
        y = Y[1,k]
        z = Y[2,k]
        concat = np.array([x**2,y**2,z**2,x*y,y*z,x*z,x,y,z,1])
        D = np.block([[D],[concat]])

    #w, v = np.linalg.eigh(D.T@D)
    #a = v[:,0]
    a = np.linalg.lstsq(D,np.zeros(K))
    print(a[0])
    a = a[0].flatten()
    A = np.array([[a[0],a[3]/2,a[4]/2],
                [a[3]/2,a[1],a[5]/2],
                [a[4]/2,a[5]/2,a[2]]])
    print(A)
    C = inv(cholesky(A).T)
    b = -inv(A)@np.array([[a[6]],[a[7]],[a[8]]])
    return C, b

if __name__ == "__main__":
    S = np.array([
        [0.7,-0.8,0.4],
        [1.1,0.3,-0.1],
        [-0.3,0.6,0.7]
    ])
    h = np.array([[0.1],[0.2],[-0.3]])
    m_true, m_samples = generate_data(S,h)

    # Show generated data
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    xs1 = m_true[0,:]
    xs2 = m_samples[0,:]
    ys1 = m_true[1,:]
    ys2 = m_samples[1,:]
    zs1 = m_true[2,:]
    zs2 = m_samples[2,:]

    ax.scatter(xs1, ys1, zs1, color="g")
    ax.scatter(xs2, ys2, zs2, color="r")
    plt.show()

    T, b = mag.calibrate_mag(m_samples)
    print(T)
    print(h)

    m_corrected = inv(T) @ (m_samples.T - h.T).T

    # Show generated data
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    xs1 = m_true[0,:]
    xs2 = m_samples[0,:]
    xs3 = m_corrected[0,:]

    ys1 = m_true[1,:]
    ys2 = m_samples[1,:]
    ys3 = m_corrected[1,:]

    zs1 = m_true[2,:]
    zs2 = m_samples[2,:]
    zs3 = m_corrected[2,:]

    ax.scatter(xs1, ys1, zs1, color="g")
    ax.scatter(xs2, ys2, zs2, color="r")
    ax.scatter(xs3, ys3, zs3, color="b")
    plt.show()


    


