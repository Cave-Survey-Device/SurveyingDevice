import matplotlib.pyplot as plt
import numpy as np
import json
from matplotlib import cm
from numpy.linalg import norm

def angle(v1, v2):
    v1 = np.array(v1)
    v2 = np.array(v2)
    dot = np.dot(v1,v2)
    frac = dot / (norm(v1)*norm(v2))
    return abs(90-abs(np.rad2deg(np.arccos(frac))))

with open("./data.json", "r") as f: 
    data = json.load(f);
    

static_mag_samples = np.array(data["static_mag_samples"])
static_acc_samples = np.array(data["static_acc_samples"])

laser_mag_samples = np.array(data["laser_mag_samples"])
laser_acc_samples = np.array(data["laser_acc_samples"])

Ra_static = np.array(data["Ra_static"])
ba_static = np.array(data["ba_static"])
Rm_static = np.array(data["Rm_static"])
bm_static = np.array(data["bm_static"])

Ra_laser = np.array(data["Ra_laser"])
Rm_laser = np.array(data["Rm_laser"])

Rm_align = np.array(data["Rm_align"])


mag_corrections = Rm_static @ (static_mag_samples.T - bm_static).T
acc_corrections = Ra_static @ (static_acc_samples.T - ba_static).T


mag_samples_norm = norm(static_mag_samples,axis=0)
acc_samples_norm = norm(static_acc_samples,axis=0)
mag_corrections_norm = norm(mag_corrections,axis=0)
acc_corrections_norm = norm(acc_corrections,axis=0)


#------------------------------- Normal Plots -------------------------------
fig = plt.figure()
plt.plot(mag_samples_norm, color="r")
plt.plot(mag_corrections_norm, color="g")

fig2 = plt.figure()
plt.plot(acc_samples_norm, color="r")
plt.plot(acc_corrections_norm, color="g")

fig3 = plt.figure()
plt.plot(mag_corrections[0], color="r")
plt.plot(mag_corrections[1], color="g")
plt.plot(mag_corrections[2], color="b")


# ------------------------------- 3D Static Data plots -------------------------------
fig = plt.figure()
rx_mag_orig = static_mag_samples[0]
ry_mag_orig = static_mag_samples[1]
rz_mag_orig = static_mag_samples[2]

rx_acc_orig = static_acc_samples[0]
ry_acc_orig = static_acc_samples[1]
rz_acc_orig = static_acc_samples[2]


ax_mag_scatter = fig.add_subplot(projection='3d')
ax_mag_scatter.scatter(rx_mag_orig,ry_mag_orig,rz_mag_orig,color="steelblue")
ax_mag_scatter.scatter(rx_acc_orig,ry_acc_orig,rz_acc_orig,color="crimson")


# ------------------------------- 3D Laser Data plots -------------------------------
mag_las_align = Rm_laser @ Rm_static @ (laser_mag_samples.T - bm_static).T
acc_las_align = Ra_laser @ Ra_static @ (laser_acc_samples.T - ba_static).T

fig = plt.figure()
ax_mag_data = fig.add_subplot(1,2,1,projection='3d')
ax_mag_data.quiver(*np.zeros_like(laser_acc_samples),*laser_acc_samples, color='crimson')
ax_mag_data.set_xlim3d([-1.25, 1.25])
ax_mag_data.set_ylim3d([-1.25, 1.25])
ax_mag_data.set_zlim3d([-1.25, 1.25])

ax_acc_data = fig.add_subplot(1,2,2,projection='3d')
ax_acc_data.quiver(*np.zeros_like(acc_las_align),*acc_las_align, color='steelblue')
ax_acc_data.set_xlim3d([-1.25, 1.25])
ax_acc_data.set_ylim3d([-1.25, 1.25])
ax_acc_data.set_zlim3d([-1.25, 1.25])

fig = plt.figure()
ax_mag_data = fig.add_subplot(1,2,1,projection='3d')
ax_mag_data.quiver(*np.zeros_like(laser_mag_samples),*laser_mag_samples, color='crimson')
ax_mag_data.set_xlim3d([-1.25, 1.25])
ax_mag_data.set_ylim3d([-1.25, 1.25])
ax_mag_data.set_zlim3d([-1.25, 1.25])

ax_acc_data = fig.add_subplot(1,2,2,projection='3d')
ax_acc_data.quiver(*np.zeros_like(mag_las_align),*mag_las_align, color='steelblue')
ax_acc_data.set_xlim3d([-1.25, 1.25])
ax_acc_data.set_ylim3d([-1.25, 1.25])
ax_acc_data.set_zlim3d([-1.25, 1.25])

plt.show()
# mag_corrections = Ralign@mag_corrections

# s = np.shape(acc_corrections)
# sum = 0
# for i in range(s[1]):
#     sum += angle(acc_corrections[:,i],mag_corrections[:,i])
#     print(angle(acc_corrections[:,i],mag_corrections[:,i]))

# print("Avg angle:", sum/i)


mag_static_align = Rm_laser @ Rm_align @ Rm_static @ (static_mag_samples.T - bm_static).T
acc_static_align = Ra_laser @ Ra_static @ (static_acc_samples.T - ba_static).T
s = np.shape(mag_static_align)
sum=0
for i in range(s[1]):
    sum += angle(mag_static_align[:,i],acc_static_align[:,i])
    print(angle(mag_static_align[:,i],acc_static_align[:,i]))

print("Avg angle:", sum/i)

# print("Std deviation:", np.std(acc_corrections[1][0:20]), np.std(mag_corrections[1][0:20]))

# # print()
# # print(mag_corrections)
# # print()
# # print(acc_corrections)