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
print("Initial roll:", np.rad2deg(initial_roll))

# Generate initial data
target_len = norm(target)
theta = combined_error
alpha = np.pi - theta
beta = np.arcsin(DISTO_LEN * np.sin(alpha)/target_len);
gamma = theta-beta
print("Gamma:", gamma)


initial_disto_tip = arbrot(np.cross(target,z_ax),gamma) @ target / (norm(target)) * DISTO_LEN
laser_len = norm(initial_disto_tip-target)



# # Find contributions of laser
# laser_vec = target - initial_disto_tip
# laser_vec = laser_vec/norm(laser_vec)
# laser_vec = np.array([np.dot(laser_vec,x_ax),np.dot(laser_vec,y_ax),np.dot(laser_vec,z_ax)])

print("Initial roll applied to initial lase vec: ", arbrot(x_ax, initial_roll) @ initial_laser_vec )
print("Initial disto tip: ", initial_disto_tip)
print("Initial laser vec: ", initial_laser_vec)
# print("Initial laser location: ", x_ax*DISTO_LEN + laser_vec * norm(target - initial_disto_tip))


print()

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
    roll = theta + initial_roll

    # Calculate new axis
    x_ax_xfrm = disto_tip
    y_ax_xfrm = np.cross(disto_tip,z_ax)
    z_ax_xfrm = np.cross(x_ax_xfrm, y_ax_xfrm)

    x_ax_xfrm = x_ax_xfrm / norm(x_ax_xfrm)
    y_ax_xfrm = y_ax_xfrm / norm(y_ax_xfrm)
    z_ax_xfrm = z_ax_xfrm / norm(z_ax_xfrm)

    # Find laser in terms of each axis
    laser_vec = target - disto_tip
    # print("Test parms, disto length, laser length: ", norm(disto_tip), norm(laser_vec))
    x_cont = np.dot(laser_vec,x_ax_xfrm)
    y_cont = np.dot(laser_vec,y_ax_xfrm)
    z_cont = np.dot(laser_vec,z_ax_xfrm)


    # Find laser vec
    laser_vec_new = np.array([x_cont,y_cont,z_cont])
    laser_vec_rotated = arbrot(x_ax, -theta+np.pi) @ laser_vec_new

    # Theta working
    disto_tip_rot = arbrot(target, -roll+np.pi) @ disto_tip
    ax.scatter(*disto_tip_rot)

    # Calculate angle errors
    heading_error = np.arctan2(laser_vec_rotated[1],laser_vec_rotated[0])
    inclination_error = np.arctan2 (laser_vec_rotated[2],laser_vec_rotated[1])


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

    print("Angle between disto and ref: ", np.rad2deg(angle))

    # Construct laser vector


    # print("Disto tip: ", disto_tip)
    # print("New laser vec: ", laser_vec_new/norm(laser_vec_new))
    # print("New rotated laser vec: ", laser_vec_rotated/norm(laser_vec_rotated), np.rad2deg(theta))
    # print("Heading and Inclination errors: ", np.rad2deg(heading_error), np.rad2deg(inclination_error))

    # print()
disto_tip = np.array([DISTO_LEN,0,0])
target = arbrot(y_ax,gamma) @ np.array([target_len,0,0])
laser_vec = arbrot(x_ax,-angle) @ (target - disto_tip)
laser_vec = laser_vec/norm(laser_vec)
print("New laser vec: ", laser_vec)

plt.show()

# combined_error = (inclination_error**2 + heading_error**2)**0.5
# intial_disto_tip = t3d.axangles.axangle2mat()

# t3d.axangles.axangle2mat()