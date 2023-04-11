import numpy as np
from numpy import kron
from numpy.linalg import inv, cholesky


def calibrate_mag(m_samples: np.ndarray):
    """Generates an initial estimate for magnetometer calibration parameters

    :param m_samples: (3,k) ndarray of magnetometer samples
    :type m_samples: np.ndarray
    :return: R0, h0
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
        
    # Solve least squares
    eigen_vals, eigen_vecs = np.linalg.eig(Y.T@Y)
    ze = eigen_vecs[-1]

    # Find alpha
    Ae = ze[0:9].reshape((3,3))
    be = ze[9:12]
    ce = ze[12]
    alpha = 4/(be.T@inv(Ae)@be-4*ce)

    # Calculate solution
    z = alpha * ze
    A = z[0:9].reshape((3,3))
    b = z[9:12]
    c = z[12]

    h0 = -inv(A) @ b * 0.5
    R0 = cholesky(A)

    return R0, h0
