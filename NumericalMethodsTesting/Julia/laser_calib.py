import numpy as np
import math as m
import transforms3d as t3d

arbrot = t3d.axangles.axangle2mat

# Define model parameters
target = np.array([1,1,1])
x_ax = np.array([1,0,0])
y_ax = np.array([0,1,0])
z_ax = np.array([0,0,1])

inclination_error = -np.deg2rad(5) #-rotation about y axis
heading_error = np.deg2rad(-2.5)
DISTO_LEN = 0.1
N_ROTS = 8

# Generate data parameters
laser_vec = np.array([1,0,0])
laser_vec = arbrot(y_ax,inclination_error) @ laser_vec
laser_vec = arbrot(z_ax,heading_error) @ laser_vec
combined_error = np.arccos(np.dot(laser_vec,x_ax))
initial_roll = -np.pi/2 - np.arctan2(laser_vec[2],laser_vec[1])
print("Initial roll:", np.rad2deg(initial_roll))

# Generate initial data
target_len = np.linalg.norm(target)
theta = combined_error
alpha = np.pi - theta
beta = np.arcsin(DISTO_LEN * np.sin(alpha)/target_len);

gamma = theta-beta
print("Gamma:", gamma)


initial_disto_tip = arbrot(np.cross(target,z_ax),gamma) @ target / (np.linalg.norm(target)) * DISTO_LEN


# # Find contributions of laser
# laser_vec = target - initial_disto_tip
# laser_vec = laser_vec/np.linalg.norm(laser_vec)
# laser_vec = np.array([np.dot(laser_vec,x_ax),np.dot(laser_vec,y_ax),np.dot(laser_vec,z_ax)])


print("Initial disto tip: ", initial_disto_tip)
print("Initial laser vec: ", laser_vec)
print("Initial laser location: ", x_ax*DISTO_LEN + laser_vec * np.linalg.norm(target - initial_disto_tip)) 
print()

# Generate rotated data
for i in range(0,N_ROTS):
    theta = i * np.pi/N_ROTS
    disto_tip = arbrot(target,theta) @ initial_disto_tip
    roll = theta + initial_roll

    # Calculate new axis
    x_ax_xfrm = disto_tip
    y_ax_xfrm = np.cross(disto_tip,z_ax)
    z_ax_xfrm = np.cross(x_ax_xfrm, y_ax_xfrm)
    x_ax_xfrm = x_ax_xfrm / np.linalg.norm(x_ax_xfrm)
    y_ax_xfrm = y_ax_xfrm / np.linalg.norm(y_ax_xfrm)
    z_ax_xfrm = z_ax_xfrm / np.linalg.norm(z_ax_xfrm)

    # Find laser in terms of each axis
    laser_vec = target - disto_tip
    x_cont = np.dot(laser_vec,x_ax_xfrm)
    y_cont = np.dot(laser_vec,y_ax_xfrm)
    z_cont = np.dot(laser_vec,z_ax_xfrm)


    # Find laser vec
    laser_vec_new = np.array([x_cont,y_cont,z_cont])
    laser_vec_rotated = arbrot(x_ax, -roll+np.pi) @ laser_vec_new

    print("New laser vec: ", laser_vec_new/np.linalg.norm(laser_vec_new))
    print("New rotated laser vec: ", laser_vec_rotated/np.linalg.norm(laser_vec_rotated))

    print()



# combined_error = (inclination_error**2 + heading_error**2)**0.5
# intial_disto_tip = t3d.axangles.axangle2mat()

# t3d.axangles.axangle2mat()