import numpy as np
import math as m
import transforms3d as t3d

arbrot = t3d.axangles.axangle2mat

# Define model parameters
target = np.array([1,0.5,1])
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
print("Initial disto tip:", initial_disto_tip)
print("Initial laser vec", laser_vec)
print()

# Generate rotated data
for i in range(0,1):
    theta = i * np.pi/N_ROTS
    disto_tip = arbrot(target,theta) @ initial_disto_tip
    roll = theta + initial_roll

    # Rotate disto into x-axis --- WORKS
    angle = np.arccos(np.dot(disto_tip,x_ax)/np.linalg.norm(disto_tip))
    rotated_target = arbrot(np.cross(disto_tip,x_ax),angle) @ target
    rotated_disto_tip = arbrot(np.cross(disto_tip,x_ax),angle) @ disto_tip
    roll_corrected_target = arbrot(x_ax,-roll) @ rotated_target

    laser_vec = rotated_target-rotated_disto_tip
    roll_corr_laser_vec = roll_corrected_target - rotated_disto_tip
    print("Rotated disto tip: ", rotated_disto_tip)
    print("Rotated target: ", rotated_target)
    print("Roll corrected target: ", roll_corrected_target)
    print("Laser vec: ", laser_vec/np.linalg.norm(laser_vec))
    print("Roll corrected laser vec: ", roll_corr_laser_vec/np.linalg.norm(roll_corr_laser_vec))
    print()



# combined_error = (inclination_error**2 + heading_error**2)**0.5
# intial_disto_tip = t3d.axangles.axangle2mat()

# t3d.axangles.axangle2mat()