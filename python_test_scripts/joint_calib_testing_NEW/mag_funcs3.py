# using https://iopscience.iop.org/article/10.1088/1755-1315/237/3/032015/pdf
import numpy as np
from numpy import kron
from numpy.linalg import inv, cholesky, norm


def calibrate_mag(m_samples: np.ndarray):
    K = m_samples.shape[1]
    Y = m_samples

    # Form D to solve
    k = 0
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

    eigen_vals, eigen_vecs = np.linalg.eig(D.T@D)
    a = eigen_vecs[:,-1]


    A = np.array([[a[0],a[3]/2,a[4]/2],
                  [a[3]/2,a[1],a[5]/2],
                  [a[4]/2,a[5]/2,a[2]]])
    
    
    R = cholesky(A)
    b = -inv(A) @ np.array([[a[6]],[a[7]],[a[8]]])
    c = a[9]
    return R, b
