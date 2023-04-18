import numpy as np
from numpy.linalg import inv
from numpy import pi
import utils
import matplotlib.pyplot as plt

def xtosR(x):
    # Decompose x
    vecR = x[:-1].T
    s = x[-1]
    # Convert vec(R) to 3x3 R
    R = np.block([[vecR[:3]],[vecR[3:6]],[vecR[6:9]]])
    return s, R

def sRtoX(s, R):
    x = np.block([R.flatten().T,s]).T
    return x

def J(x, m_samples, g_samples):
    s, R = xtosR(x)
    print("s:", s)
    print("R", R)
    k = 0
    tot = (s-g_samples[:,k].T@R@m_samples[:,k])**2
    print("K=0 value:", tot)
    for k in range(1,K):
        tot += (s-g_samples[:,k].T@R@m_samples[:,k])**2

    J = np.linalg.norm(R@R.T - np.identity(3))**2 + tot #np.sum([(s-g_samples[:,k].T@R@m_samples[:,k])**2] for k in range(K))
    print(J)
    print()
    return J

def grad_J(x,m_samples,g_samples):
    s, R = xtosR(x)

    """-------------------- dJ_R --------------------"""
    #dJ_R = - 2 * np.sum([(s-g_samples[:,k].T@R@m_samples[:,k]) * np.kron(m_samples[:,k],g_samples[:,k]) for k in range(K)])
    k = 0
    dJ_R = (s-g_samples[:,k].T@R@m_samples[:,k]) * np.kron(m_samples[:,k],g_samples[:,k])
    for k in range(1,K):
        dJ_R += (s-g_samples[:,k].T@R@m_samples[:,k]) * np.kron(m_samples[:,k],g_samples[:,k])
    dJ_R = -2 * dJ_R + 4*(R@R.T@R-R).flatten()

    """-------------------- dJ_s --------------------"""
    # dJ_s =   2 * np.sum([(s-g_samples[:,k].T@R@m_samples[:,k])] for k in range(K))
    k = 0
    dJ_s = (s-g_samples[:,k].T@R@m_samples[:,k])
    for k in range(1,K):
        dJ_s += (s-g_samples[:,k].T@R@m_samples[:,k])
    dJ_s = 2 * dJ_s

    """-------------------- grad_J --------------------"""
    grad_J = np.block([dJ_R.T, dJ_s]).T

    return grad_J

def laplacian_J(x,m_samples,g_samples, A):
    s, R = xtosR(x)
    # A = np.block([e1,e4,e7,e2,e5,e8,e3,e6,e9]).T
        # np.sum([np.kron(m_samples[:,k],g_samples[:,k])*np.kron(m_samples[:,k],g_samples[:,k]).T] for k in range(K))
    """-------------------- D2J_DRDT --------------------"""
    k = 0
    tot = np.kron(m_samples[:,k],g_samples[:,k])*np.kron(m_samples[:,k],g_samples[:,k]).T
    for k in range(1,K):
        tot += np.kron(m_samples[:,k],g_samples[:,k])*np.kron(m_samples[:,k],g_samples[:,k]).T

    d2J_dRdRT = np.kron((R.T@R),np.identity(3))
    + np.kron(np.identity(3),R@R.T)
    + np.kron(R.T,R) @ A
    - 4 * np.identity(9)
    + 2 * tot

    """-------------------- D2J_dRds --------------------"""
    #d2J_dRds  = -2 * np.sum([np.kron(m_samples[:,k],g_samples[:,k])] for k in range(K))
    k = 0
    d2J_dRds = np.kron(m_samples[:,k],g_samples[:,k])
    for k in range(1,K):
        d2J_dRds += np.kron(m_samples[:,k],g_samples[:,k])
    d2J_dRds = -2* d2J_dRds

    """-------------------- D2J_dsdRT --------------------"""
    #d2J_dsdRT = -2 * np.sum([np.kron(m_samples[:,k],g_samples[:,k])] for k in range(K)).T
    k = 0
    d2J_dsdRT = np.kron(m_samples[:,k],g_samples[:,k])
    for k in range(1,K):
        d2J_dsdRT += np.kron(m_samples[:,k],g_samples[:,k])
    d2J_dsdRT = -2 * d2J_dsdRT

    """-------------------- d2J_ds2 --------------------"""
    d2J_ds2 = 2*K

    """-------------------- laplacian_J --------------------"""
    laplacian_J = np.block([[d2J_dRdRT, d2J_dRds],[d2J_dsdRT, d2J_ds2]])
    print("laplacian:\n", laplacian_J, "\n")

    return laplacian_J


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


    # Tsi = np.identity(3) + np.array([
    #     [0.05,0.1,0.15],
    #     [0.05,0.1,0.15],
    #     [0.05,0.1,0.15]])
    Tsi = np.identity(3)

    hhi = np.ones((3,1)) #np.array([[0.3],[-0.2],[0.1]]) # Hard iron bias
    hb = np.array([[-0.05],[0.25],[-0.16]]) # Offset
    Tm = Tsf @ Tcc @ Tsi
    hm = Tsf @ Tcc @ hhi + hb
    Tm = Tcc
    print(Tm)

    """ Alignment only:"""
    ha = np.array([[0],[0],[0]])
    hm = np.array([[0],[0],[0]])
    Ta = np.identity(3)
    Tm = utils.x_rotation(pi/12) @ utils.y_rotation(pi/10) 

    g_true, m_true = utils.generate_true_data()
    g_samples= utils.generate_sample_data(g_true,Ta,ha,0,0.001)
    m_samples= utils.generate_sample_data(g_true,Tm,hm,0,0.001)



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
    # plt.show()


    """ --------------------- RUN ALLIGNMENT AND CALIBRATION ---------------------"""
    convergence_figure = plt.figure()
    conv_ax = convergence_figure.add_subplot()
    conv_ax.set_xlabel("Solution Space")
    conv_ax.set_xticks(range(2), ['Initial\n Guess','Phase 1\n Output'])
    conv_ax.set_ylabel("Alignment Error - J(R,s)")
    """ ---------- PHASE 1 ---------- """
    # Define consts
    K = g_samples.shape[1]

    # Step 1: form Kx9 matrix A
    A = np.kron(m_samples[:,0],g_samples[:,0]).T
    for k in range(1,K):
        concat = np.kron(m_samples[:,k],g_samples[:,k]).T
        A = np.block([[A],[concat]])

    # Step 2: for 3x3 matrix H
    H = inv(A.T@A) @ A.T @ np.ones((K,1))
    h1 = H[0:3,:]
    h2 = H[3:6,:]
    h3 = H[6:9,:]
    H = np.block([h1,h2,h3])

    # Step 3: Assume SVD of H
    u, s, v = np.linalg.svd(H)
    U_hat = np.sign(np.linalg.det(H)) * u

    # Step 4: Calculate R_hat
    R_hat = U_hat @ v.T

    # Step 5: Calculate s_hat
    s_hat = 0
    for k in range(K):
        s_hat += 1/K * g_samples[:,k].T @ R_hat @ m_samples[:,k]

    conv_ax.scatter(0,J(sRtoX(0,np.identity(3)),m_samples,g_samples))   
    conv_ax.scatter(1,J(sRtoX(s_hat,R_hat),m_samples,g_samples))   
    plt.show()


    """ ---------- PHASE 2 ----------- """
    # Step 6: Utilise Newton-Raphson Method
    # Step 6.1:
    R = R_hat #np.identity(3)
    s = s_hat #0
    x = sRtoX(s,R) #np.block([R.flatten().T,s]).T

    # Step 6.2:
    t = 10
    a = 1
    beta = 0.975

    # Steps 6.3-6.6:
    # A = 
    x_arr = []
    dx_arr = []
    J_arr = [1000]
    while J_arr[-1] > 0.001:
        print("New Iter")
        # Step 6.3:
        #dx = -inv(laplacian_J(x,m_samples,g_samples,A)) @ grad_J(x,m_samples,g_samples).T
        dx = -grad_J(x,m_samples,g_samples) # Gradient descent
        print(dx)
        dx_arr.append(dx)

        # Step 6.4:
        while np.linalg.norm(J(x+t*dx,m_samples,g_samples)) > np.linalg.norm(J(x,m_samples,g_samples)+a*t*grad_J(x,m_samples,g_samples).T@dx):
            t = beta*t
            print(t)

        # Step 6.5:
        x = x+t*dx
        x_arr.append(x)

        # Step 6.6:
        J_arr.append(J(x,m_samples,g_samples))

    print("DONE")




    # """ --------------------- PLOT ALLIGNMENT AND CALIBRATION DATA ---------------------"""
    
    # corrected_accel_data = inv(T.T[:,0:3].T) @ (Y[0:3,:] - H.T[:,0:3].T) 
    # corrected_mag_data = inv(T.T[:,3:6].T) @ (Y[3:6,:] - H.T[:,3:6].T) 

    # fig = plt.figure()
    # ax = fig.add_subplot()
    # ax.plot(range(m_samples.shape[1]),np.linalg.norm(m_samples,axis=0) , color = "r", label="Sample data")
    # ax.plot(range(corrected_mag_data.shape[1]),np.linalg.norm(corrected_mag_data,axis=0) , color="b",label="Corrected data")
    # ax.set_title("Magnitude of magnetometer data")
    # ax.legend()

    # fig = plt.figure()
    # ax2 = fig.add_subplot()
    # ax2.plot(range(len(cost)),cost)
    # ax2.set_title("Cost VS iterations")
    # ax2.set_xlabel("Iteration")
    # ax2.set_ylabel("Cost")

    # # Show generated g data
    # fig = plt.figure()
    # ax = fig.add_subplot(projection='3d')

    # xs1 = g_true[0,:]
    # xs2 = g_samples[0,:]
    # ys1 = g_true[1,:]
    # ys2 = g_samples[1,:]
    # zs1 = g_true[2,:]
    # zs2 = g_samples[2,:]

    # xs3 = corrected_accel_data[0,:]
    # ys3 = corrected_accel_data[1,:]
    # zs3 = corrected_accel_data[2,:]

    # ax.scatter(xs1, ys1, zs1, color="g")
    # ax.scatter(xs2, ys2, zs2, color="r")
    # ax.scatter(xs3, ys3, zs3, color="b")
    
    # plt.show()