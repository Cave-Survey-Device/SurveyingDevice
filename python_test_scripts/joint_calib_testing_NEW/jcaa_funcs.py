import numpy as np
from numpy import kron, sin, cos, tan
from numpy.linalg import norm, inv, det

""" GLOBALS """
lambda_a = 1
lambda_m = 1
lambda_an = 1
lambda_mn = 1
lambda_al = 1
lambda_R = 1

def correct_hard(mat):
    return np.mean(mat,axis=1)

def correct_soft(hard_corrected_arr):
    cov_mat = np.cov(hard_corrected_arr)
    eigen_vals, eigen_vecs = np.linalg.eig(cov_mat)
    t_mat = np.matmul(eigen_vecs,np.diag(np.sqrt(eigen_vals)))
    return inv(t_mat)

def claibrate_magnetometer(ym):
    hm = correct_hard(ym)
    Tm = correct_soft((ym.T - hm).T)
    return inv(Tm), hm

def composeX(Ha: np.ndarray, va: np.ndarray, F: np.ndarray, Hm: np.ndarray, vm: np.ndarray, M: np.ndarray, R: np.ndarray, d: float):
    out =  np.block([Ha.flatten(), va.flatten(), F.flatten('F'),
              Hm.flatten(), vm.flatten(), M.flatten('F'),
              R.flatten(), d])
    if out.size() != (9+3+3*K+9+3+3*K+9+1):
        raise("composeX: X has incorrect size!")
    return out

def decomposeX(x: np.ndarray, K):
    Ha = x[0:9].reshape((3,3))
    np.delete(x,[range(9)])

    va = x[0:3].reshape((3,1))
    np.delete(x,[range(3)])

    F = x[0:3*K].reshape((3,K))
    np.delete(x,[range(3*K)])

    Hm = x[0:9].reshape((3,3))
    np.delete(x,[range(9)])

    vm = x[0:3].reshape((3,1))
    np.delete(x,[range(3)])

    M = x[0:3*K].reshape((3,K))
    np.delete(x,[range(3*K)])

    R = x[0:9].reshape((3,3))
    np.delete(x,[range(9)])

    d = x[0]
    np.delete(x,1)

    if x.size > 0:
        raise("decomposeX: X has not been fully emptied!")
    return Ha, va, F, Hm, vm, M, R, d

def J(x,ya,ym,K):
    """Cost function - evaluate for the error

    :param x: input vecor
    :type x: np.ndarray
    :return: Cost for given input vector
    :rtype: float
    """
    
    Ha, va, m, Hm, vm, f, R, d = decomposeX(x,K)
    tot1 = norm(f[:,k] - Ha@ya[:,k] + va)**2
    tot2 = norm(m[:,k] - Hm@ym[:,k] + vm)**2
    tot3 = (d - f[:,k].T@R@m[:,k])**2
    for k in range(1,K):
        tot1 += norm(f[:,k] - Ha@ya[:,k] + va)**2
        tot2 += norm(m[:,k] - Hm@ym[:,k] + vm)**2
        tot3 += (d - f[:,k].T@R@m[:,k])**2
    return lambda_a * tot1 + lambda_m * tot2 + lambda_al * tot3

def dJ_vecHa(x,ya,ym,K):
    Ha, va, m, Hm, vm, f, R, d = decomposeX(x,K)
    k = 0
    tot = kron(ya[:,k], Ha@ya[:,k] - f[:,k] - va)
    for k in range(1,K):
        tot += kron(ya[:,k], Ha@ya[:,k] - f[:,k] - va)
    return 2*lambda_a * tot

def dJ_va(x,ya,ym,K):
    Ha, va, m, Hm, vm, f, R, d = decomposeX(x,K)
    k=0
    tot = -Ha@ya[:,k] + va + f[:,k]
    for k in range(1,K):
        tot += -Ha@ya[:,k] + va + f[:,k]
    return 2 * lambda_a * tot

def dJ_f(x,p,ya,ym,K):
    Ha, va, m, Hm, vm, f, R, d = decomposeX(x,K)
    out = -2*lambda_a*(Ha@ya[:,p]+f[:,p]-va)
    out += 4*lambda_an*f[:,p]*norm(f[:,p])**2
    out += -lambda_al* (2*(d-f[:,p].T@R@m[:,p])@m[:,p].T@R.T).T
    return out

def dJ_F(x,ya,ym,K):
    li = [dJ_f(x,p,ya,ym,K) for p in range(K)]
    return np.concatenate( li, axis=0 )

def dJ_vecHm(x,ya,ym,K):
    Ha, va, m, Hm, vm, f, R, d = decomposeX(x,K)
    k = 0
    tot = kron(ym[:,k], Hm@ym[:,k] - m[:,k] - vm)
    for k in range(1,K):
        tot += kron(ym[:,k], Hm@ym[:,k] - m[:,k] - vm)
    return 2*lambda_m * tot

def dJ_vm(x,ya,ym,K):
    Ha, va, m, Hm, vm, f, R, d = decomposeX(x,K)
    k=0
    tot = -Hm@ym[:,k] + vm + m[:,k]
    for k in range(1,K):
        tot += -Hm@ym[:,k] + vm + m[:,k]
    return 2 * lambda_m * tot

def dJ_m(x,p,ya,ym,K):
    Ha, va, m, Hm, vm, f, R, d = decomposeX(x,K)
    out = -2*lambda_m*(Hm@ym[:,p]+m[:,p]-vm)
    out += 4*lambda_mn*m[:,p]*norm(m[:,p])**2
    out += -lambda_al* (2*(d-f[:,p].T@R@m[:,p])@f[:,p].T@R.T).T
    return out

def dJ_M(x,ya,ym,K):
    li = [dJ_m(x,p,ya,ym,K) for p in range(K)]
    return np.concatenate( li, axis=0 )

def dJ_vecR(x,ya,ym,K):
    Ha, va, m, Hm, vm, f, R, d = decomposeX(x,K)
    k=0
    tot = (d-f[:,k].T@R@m[:,k]) * kron(m[:,k],f[:,k])
    for k in range(1,K):
        tot += (d-f[:,k].T@R@m[:,k]) * kron(m[:,k],f[:,k])
    out = -2*lambda_al*tot
    out += 4*lambda_R*(R@R.T@R-R).flatten()
    out += 2*lambda_R*(det(R) -1) * (R.H.T)
    return out

def dJ_d(x,ya,ym,K):
    Ha, va, m, Hm, vm, f, R, d = decomposeX(x,K)
    k=0
    tot = (d-f[:,k].T@R@m[:,k])
    for k in range(1,K):
        tot += (d-f[:,k].T@R@m[:,k])
    return 2*lambda_al*tot

def grad_J(x,ya,ym,K):
    out = np.block([dJ_vecHa(x,ya,ym,K).T,
              dJ_va(x,ya,ym,K).T,
              dJ_F(x,ya,ym,K).T,
              dJ_vecHm(x,ya,ym,K).T,
              dJ_vm(x,ya,ym,K).T,
              dJ_M(x,ya,ym,K).T,
              dJ_vecR(x,ya,ym,K).T,
              dJ_d(x,ya,ym,K).T]).T
    return out