import numpy as np
import scipy
import matplotlib.pyplot as plt
N = 6
b_a = np.array([[0.1],[0.15],[0.2]])

# Cb_n = np.array([
#     [1,0,0],
#     [0,1,0],
#     [0,0,1]])

# S_x = 0
# S_y = 0
# S_z = 0
# C_s = np.array([
#     [1+S_x, 0, 0],
#     [0, 1+S_y, 0],
#     [0, 0, 1+S_z]])

# a = 0
# b = 0
# c = 0
# C_n = np.array([
#     [1, np.sin(a), -np.sin(b)],
#     [0, np.cos(a), np.sin(c)*np.cos(b)],
#     [0, 0, np.cos(c)*np.cos(b)]]);

# T_a = C_s @ C_n
T_a = np.array(
    [[1.15,0.0602,-0.401],
    [0,1.0985,0.0575],
    [0,0,1.0978]])

g_vec = np.array([[0],[0],[1]])

ym = np.zeros((3,6*5))
for x in range (4):
    x_ang = x*np.pi/2
    rotation_mat =  np.array([[1, 0, 0],
                    [0, np.cos(x_ang), -np.sin(x_ang)],
                    [0, np.sin(x_ang), np.cos(x_ang)]])
    for i in range(5):
        ym[:,x*5+i] = (T_a @ rotation_mat @ g_vec + b_a)[:,0] + np.random.normal(0,0.1,(3))


for y in range(2):
    y_ang = y*np.pi+np.pi/2;
    rotation_mat = [[np.cos(y_ang), 0, np.sin(y_ang)],
                    [0, 1 ,0],
                    [-np.sin(y_ang), 0, np.cos(y_ang)]]
    for i in range(5):
        ym[:,20+y*5+i] = (T_a @ rotation_mat @ g_vec + b_a)[:,0] + np.random.normal(0,0.1,(3))




# ym = np.array([
#     [-0.301000, 0.039800, 0.501000, 0.160200, 1.250000, -1.050000],
#     [0.207500, -0.948500, 0.092500, 1.248500, 0.150000, 0.150000],
#     [1.297800, 0.200000, -0.897800, 0.200000, 0.200000, 0.200000]])

print("ym: \n", ym)
fig = plt.figure()
ax = fig.add_subplot(projection='3d')

xs = ym[0]
ys = ym[1]
zs = ym[2]
ax.scatter(xs, ys, zs, color="r")

A = np.ones((6*5,10)); 
for i in range(6*5):
    ym_i = ym[:,i]
    A[i,:] = [
        ym_i[0]**2, ym_i[1]**2, ym_i[2]**2,
        2*ym_i[0]*ym_i[1], 2*ym_i[0]*ym_i[2], 2*ym_i[1]*ym_i[2],
        2*ym_i[0], 2*ym_i[1], 2*ym_i[2],
        1]
print("A:\n", A)

# A2 = np.array([
#     [0.090601, 0.043056, 1.684285, -0.124915, -0.781276, 0.538587, -0.602000, 0.415000, 2.595600, 1.000000],
#     [0.001584, 0.899652, 0.040000, -0.075501, 0.015920, -0.379400, 0.079600, -1.897000, 0.400000, 1.000000],
#     [0.251001, 0.008556, 0.806045, 0.092685, -0.899596, -0.166093, 1.002000, 0.185000, -1.795600, 1.000000],
#     [0.025664, 1.558752, 0.040000, 0.400019, 0.064080, 0.499400, 0.320400, 2.497000, 0.400000, 1.000000],
#     [1.562500, 0.022500, 0.040000, 0.375000, 0.500000, 0.060000, 2.500000, 0.300000, 0.400000, 1.000000],
#     [1.102500, 0.022500, 0.040000, -0.315000, -0.420000, 0.060000, -2.100000, 0.300000, 0.400000, 1.000000]])

b = np.ones(6*5)

def f(x):
    m = np.array([
        ym_i[0]**2, ym_i[1]**2, ym_i[2]**2,2*ym_i[0]*ym_i[1], 2*ym_i[0]*ym_i[2], 2*ym_i[1]*ym_i[2], 2*ym_i[0], 2*ym_i[1], 2*ym_i[2],1])
    return 1-m@x

#x = scipy.optimize.least_squares(f,np.zeros((10))).x
x = np.linalg.lstsq(A,b)[0]
#x = np.linalg.inv(A.T@A)@A.T@b

print(x)
E_targ = np.linalg.inv(T_a).T@np.linalg.inv(T_a)
F_targ = -E_targ @ b_a
X_targ = np.array([E_targ[0,0],E_targ[1,1],E_targ[2,2],E_targ[0,1],E_targ[0,2],E_targ[1,2],F_targ[0,0],F_targ[1,0],F_targ[2,0],0])

print()
print("X_TARGET\n",X_targ)
print("x\n",x)
print()

E = np.array([
    [x[0], x[3], x[4]],
    [x[3], x[1], x[5]],
    [x[4], x[5], x[2]]])


F = np.array([x[6], x[7], x[8]])

G = x[9]-1

print("E_TARGET:\n",E_targ)
print("E: \n", E)


R0_a = np.linalg.cholesky(E).T
b0_a = -np.linalg.inv(E) @ F




print("T_a: \n",np.linalg.inv(R0_a))
print("R0_a:\n",R0_a)
b0_a = b0_a[:, np.newaxis]
print("b0_a: \n", b0_a)


ym1 = np.linalg.inv(T_a) @ (ym - b_a)
xs = ym1[0]
ys = ym1[1]
zs = ym1[2]
ax.scatter(xs, ys, zs, color="b")

ym2 = (ym-b0_a)
xs = ym2[0]
ys = ym2[1]
zs = ym2[2]
ax.scatter(xs, ys, zs, color="g")
plt.show()

