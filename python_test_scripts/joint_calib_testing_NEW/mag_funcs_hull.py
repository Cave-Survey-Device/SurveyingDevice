   
def calibrate_mag(s):
     # D (samples)
    D = np.array([s[0]**2., s[1]**2., s[2]**2.,
                2.*s[1]*s[2], 2.*s[0]*s[2], 2.*s[0]*s[1],
                2.*s[0], 2.*s[1], 2.*s[2], np.ones_like(s[0])])

    # S, S_11, S_12, S_21, S_22 (eq. 11)
    S = np.dot(D, D.T)
    S_11 = S[:6,:6]
    S_12 = S[:6,6:]
    S_21 = S[6:,:6]
    S_22 = S[6:,6:]

    # C (Eq. 8, k=4)
    C = np.array([[-1,  1,  1,  0,  0,  0],
                [ 1, -1,  1,  0,  0,  0],
                [ 1,  1, -1,  0,  0,  0],
                [ 0,  0,  0, -4,  0,  0],
                [ 0,  0,  0,  0, -4,  0],
                [ 0,  0,  0,  0,  0, -4]])

    # v_1 (eq. 15, solution)
    E = np.dot(linalg.inv(C),
            S_11 - np.dot(S_12, np.dot(linalg.inv(S_22), S_21)))

    E_w, E_v = np.linalg.eig(E)

    v_1 = E_v[:, np.argmax(E_w)]
    if v_1[0] < 0: v_1 = -v_1

    # v_2 (eq. 13, solution)
    v_2 = np.dot(np.dot(-np.linalg.inv(S_22), S_21), v_1)

    # quadric-form parameters
    M = np.array([[v_1[0], v_1[3], v_1[4]],
                [v_1[3], v_1[1], v_1[5]],
                [v_1[4], v_1[5], v_1[2]]])
    n = np.array([[v_2[0]],
                [v_2[1]],
                [v_2[2]]])
    d = v_2[3]

    return M, n, d