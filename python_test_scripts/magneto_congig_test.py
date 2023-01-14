import numpy as np
import matplotlib.pyplot as plt
import scipy
from scipy import optimize

def translate(mat,t_mat):
    return list(map(np.matmul,t_mat,mat))

def correct_hard(mat):
    return -(np.mean(mat,axis=0))

def error(a,b):
    return abs(a-b)

def add(a,b):
    return a+b

arr = np.array([[0],[0],[0],[0]])

for theta in range(0,180,20):
    for phi in range(0,360,10):
        x = np.sin(theta) * np.cos(phi)
        y = np.sin(theta) * np.sin(phi)
        z = np.cos(theta)
        arr = np.concatenate((arr,np.array([[x],[y],[z],[1]])), axis=1)

s_x, s_y, s_z = [1.1,0.5,0.7]
t_x, t_y, t_z = [4,4,4]

T_scale = np.array([[s_x,0,0,0],[0,s_y,0,0],[0,0,s_z,0],[0,0,0,1]])
T_translate = np.array([[1,0,0,t_x],[0,1,0,t_y],[0,0,1,t_z],[0,0,0,1]])


arr_translated = np.matmul(T_scale,arr)
arr_translated = np.matmul(T_translate,arr)


PERFECT_ARR = arr
HS_TRANSLATED_ARR = arr_translated
MEAN_CORRECTION = correct_hard(HS_TRANSLATED_ARR)
H_CORRECTED_ARR = HS_TRANSLATED_ARR + MEAN_CORRECTION


del(arr)
del(arr_translated)
del(s_x,s_y,s_z)
del(t_x,t_y,t_z)
del(theta)
del(phi)

cov_mat = np.cov(H_CORRECTED_ARR)
print("Covariance Matrix:", cov_mat)
#covar_mat = np.matmul(H_CORRECTED_ARR,np.transpose(H_CORRECTED_ARR))
eigen_vals, eigen_vecs = np.linalg.eig(cov_mat)
print("Eigenvalues:",eigen_vals)
print("Eigenvectors:",eigen_vecs)
print("Suare-root of eigenvalues:",np.sqrt(eigen_vals))
print("Diag of sqrt of eigenvalues:",np.diag(np.sqrt(eigen_vals)))
t_mat = np.matmul(eigen_vecs,np.diag(np.sqrt(eigen_vals)))
print("Transformation matrix:", t_mat)
print("")

HS_CORRECTED_ARR = np.matmul(np.linalg.inv(t_mat),H_CORRECTED_ARR)


#print(correct_hard(HS_TRANSLATED_ARR))
#print(correct_hard(H_CORRECTED_ARR))
#print(correct_hard(HS_CORRECTED_ARR))
#print(correct_hard(PERFECT_ARR))

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

# x_arr,y_arr,z_arr,NA = np.vsplit(HS_TRANSLATED_ARR,np.shape(HS_TRANSLATED_ARR)[0])
# ax.scatter(x_arr,y_arr,z_arr,s=2,color='red')

# x_arr,y_arr,z_arr,NA = np.vsplit(H_CORRECTED_ARR,np.shape(H_CORRECTED_ARR)[0])
# ax.scatter(x_arr,y_arr,z_arr,s=2,color='orange')

x_arr,y_arr,z_arr,NA = np.vsplit(HS_CORRECTED_ARR,np.shape(HS_CORRECTED_ARR)[0])
ax.scatter(x_arr,y_arr,z_arr,s=2,color='blue')

x_arr,y_arr,z_arr,NA = np.vsplit(PERFECT_ARR,np.shape(PERFECT_ARR)[0])
ax.scatter(x_arr,y_arr,z_arr,s=2,color='green')

plt.show() 


