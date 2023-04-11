import numpy as np
from numpy import kron
from numpy.linalg import inv, cholesky


def calibrate_mag(m_samples: np.ndarray):
    """Generates an initial estimate for magnetometer calibration parameters

    :param m_samples: (3,k) ndarray of magnetometer samples
    :type m_samples: np.ndarray
    :return: T0, h0
    :rtype: `(np.ndarray, np.ndarray)`
    """
    y = m_samples
    K = y.shape[1]

    # Formulate Y vector
    k = 0
    Y = np.block([kron(y[:,k].T,y[:,k].T).T, y[:,k].T, 1])
    Y = Y[:, np.newaxis]

    for k in range(1,K):
        concat = np.block([kron(y[:,k].T,y[:,k].T).T, y[:,k].T, 1])
        Y = np.block([Y,concat[:,np.newaxis]])
    Y = Y.T

    k = 0
    Y = np.block([  y[0,k]*y[0,k].T,
                  2*y[0,k]*y[1,k].T,
                  2*y[0,k]*y[2,k].T,
                    y[1,k]*y[1,k].T,
                  2*y[1,k]*y[2,k].T,
                    y[2,k]*y[2,k].T,
                    y[:,k].T,
                    1])
    Y = Y[:, np.newaxis]
    for k in range(1,K):
        concat = np.block([  y[0,k]*y[0,k].T,
                  2*y[0,k]*y[1,k].T,
                  2*y[0,k]*y[2,k].T,
                    y[1,k]*y[1,k].T,
                  2*y[1,k]*y[2,k].T,
                    y[2,k]*y[2,k].T,
                    y[:,k].T,
                    1])
        Y = np.block([Y,concat[:,np.newaxis]])
    Y = Y.T

        
    # Solve least squares
    eigen_vals, eigen_vecs = np.linalg.eigh(Y.T@Y)
    ze = eigen_vecs[:,0]

    # Find alpha
    Ae = np.block([[ze[0],ze[1]/2,ze[2]/2],
                     [ze[1]/2,ze[3],ze[4]/2],
                     [ze[2]/2,ze[4]/2,ze[5]]])
    be = ze[5:8]
    ce = ze[9]
    alpha = 4/(be.T@inv(Ae)@be-4*ce)

    # Calculate solution
    z = alpha * ze
    A = np.block([[z[0],z[1]/2,z[2]/2],
                  [z[1]/2,z[3],z[4]/2],
                  [z[2]/2,z[4]/2,z[5]]])
    b = z[5:8]
    c = z[9]

    h0 = -inv(A) @ b * 0.5
    T0 = inv(cholesky(A).T)

    return T0, h0