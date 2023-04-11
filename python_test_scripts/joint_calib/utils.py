import numpy as np
from numpy import sin, cos
from numpy.linalg import inv, norm, lstsq, det

import numpy.matrix.flatten as vec

def matrix_cofactor(matrix):
 
    try:
        determinant = np.linalg.det(matrix)
        if(determinant!=0):
            cofactor = None
            cofactor = np.linalg.inv(matrix).T * determinant
            # return cofactor matrix of the given matrix
            return cofactor
        else:
            raise Exception("singular matrix")
    except Exception as e:
        print("could not find cofactor matrix due to",e)


def adj(matrix):
    return matrix_cofactor(matrix).T

def generate_test_data():
    T_a = np.array(
    [[1.15,0.0602,-0.401],
    [0,1.0985,0.0575],
    [0,0,1.0978]])
    T_a = np.array(
    [[1.15,0.0602,-0.401],
    [0,1.0985,0.0575],
    [0,0,1.0978]])

    b_a = np.array([[0.1],[0.15],[0.2]])
    b_m = np.array([[0.1],[0.15],[0.2]])
    

    g_vec = np.array([[0],[0],[1]])

    yrots = 4
    xrots = 2
    N = 30

    ym_size = yrots*xrots*N
    ym = np.zeros((3,ym_size))

    g_vec = np.array([0,0,1])
    m_vec = np.array([1,0,0])

    for x in range (xrots):
        x_ang = x*np.pi/xrots
        xrotation_mat =  np.array([[1, 0, 0],
                        [0, np.cos(x_ang), -np.sin(x_ang)],
                        [0, np.sin(x_ang), np.cos(x_ang)]])
        
        for y in range(yrots):
            y_ang = y*2*np.pi/yrots
            yrotation_mat = [[np.cos(y_ang), 0, np.sin(y_ang)],
                        [0, 1 ,0],
                        [-np.sin(y_ang), 0, np.cos(y_ang)]]
            for i in range(N):
                ya[:,x*yrots*N+y*N+i] = (T_m @ xrotation_mat @ yrotation_mat @ m_vec + b_a)[:,0] + np.random.normal(0,0.05,(3))
                ym[:,x*yrots*N+y*N+i] = (T_a @ xrotation_mat @ yrotation_mat @ g_vec + b_a)[:,0] + np.random.normal(0,0.05,(3))

def getJacobian():
    J1 = 2*la * np.sum(np.kron(yak,Ha*yak-fk-va),axis=0)
    J2 = 2*la * np.sum(-Ha*yak+va+fk,axis=0)
    J3 = -2*la*(Ha*yap+fp-va) + 4*lan*fp*norm(fp)**2 - lal(2*(sin(delta) - fp.T*R*mp)*mp.T*R.T ).T
    
    J4 = 2*lm*np.sum(np.kron(ymk,(Hm*ymk-mk-vm)))
    J5 = 2*lm * np.sum(-Hm*ymk+vm+mk,axis=0)
    J6 = -2*lm*(Hm*ymp+mp-vm) + 4*lmn*mp*norm(mp)**2 - lal(2*(sin(delta) - fp.T*R*mp)*mp.T*R.T ).T

    J7 = -2*lal*np.sum([(sin(delta)-f[k].T*R*m[k])*(np.kron(m[k],f[k])) for k in range(N)],axis=0) + 4*lr*vec(R*R.T*R-R) + 2*lr*(det(R)-1)*vec(adj(R).T)
    
    J8 = 2*lal*np.sum([sin(delta)-f[k].T*R*m[k] for k in range(N)])