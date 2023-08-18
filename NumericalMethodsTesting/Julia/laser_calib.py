import numpy as np
import math as m
import transforms3d as t3d
import matplotlib.pyplot as plt
from numpy.linalg import norm

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
initial_laser_vec = np.array([1,0,0])
initial_laser_vec = arbrot(y_ax,inclination_error) @ initial_laser_vec
initial_laser_vec = arbrot(z_ax,heading_error) @ initial_laser_vec
combined_error = np.arccos(np.dot(initial_laser_vec,x_ax))
initial_roll = -np.pi/2 - np.arctan2(initial_laser_vec[2],initial_laser_vec[1])

# Generate initial data
target_len = norm(target)
theta = combined_error
alpha = np.pi - theta
beta = np.arcsin(DISTO_LEN * np.sin(alpha)/target_len);
gamma = theta-beta

print("Laser vec: ", initial_laser_vec)
print("Initial roll: ", np.rad2deg(initial_roll))
print("Alpha: ", np.rad2deg(alpha))
print("Beta: ", np.rad2deg(beta))
print("Gamma: ", np.rad2deg(gamma))
print("Initial disto tip rotation: \n",  arbrot(np.cross(target,z_ax),gamma))
initial_disto_tip = arbrot(np.cross(target,z_ax),gamma) @ target / (norm(target)) * DISTO_LEN
laser_len = norm(initial_disto_tip-target)


# Generate rotated data
ax = plt.axes(projection="3d")
line = []
line.append([0,0,0])
line.append(target*0.1)
line = np.transpose(line)
ax.plot(*line)

for i in range(0,N_ROTS):
    theta = i * 2*np.pi/N_ROTS
    disto_tip = arbrot(target,theta) @ initial_disto_tip
    print("disto tip: ", disto_tip)
    roll = theta + initial_roll

    disto_tip_rot = arbrot(target, -roll+np.pi) @ disto_tip
    # print("Roll", np.rad2deg(roll), "\tRotated disto tip: ", disto_tip_rot)

    x_ax_xfrm = target
    y_ax_xfrm = np.cross(target,z_ax)
    z_ax_xfrm = np.cross(x_ax_xfrm, y_ax_xfrm)


    ref_point = (target/norm(target)) * DISTO_LEN * np.cos(gamma)

    disto_target_vec = disto_tip_rot-ref_point
    disto_target_vec = disto_target_vec / norm(disto_target_vec)
    ref_vec = z_ax_xfrm / norm(z_ax_xfrm)

    angle = np.dot(disto_target_vec,ref_vec)
    angle = angle/(norm(disto_target_vec) * norm(ref_vec))
    angle = np.arccos(angle)



    # print()
print("angle: ", np.rad2deg(angle))
disto_tip = np.array([DISTO_LEN,0,0])
target = arbrot(y_ax,gamma) @ np.array([target_len,0,0])
laser_vec = arbrot(x_ax,-angle) @ (target - disto_tip)
laser_vec = laser_vec/norm(laser_vec)
print("New laser vec: ", laser_vec)
print(np.rad2deg(np.arctan2(laser_vec[2],laser_vec[0])), np.rad2deg(np.arctan2(laser_vec[1],laser_vec[0])))
plt.show()

# combined_error = (inclination_error**2 + heading_error**2)**0.5
# intial_disto_tip = t3d.axangles.axangle2mat()

# t3d.axangles.axangle2mat()