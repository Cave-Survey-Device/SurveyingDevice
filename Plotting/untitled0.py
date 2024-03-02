# -*- coding: utf-8 -*-
"""
Created on Fri Feb 16 10:55:03 2024

@author: chris
"""
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


m1 = [-0.31, 0.17, -0.94]
m2 = [0.43, -0.11, -0.91]
m3 = [0.20, 0.31, -0.95]
m4 = [-0.10, -0.34, -0.95]


fig = plt.figure()
ax_true = fig.add_subplot(projection='3d')
ax_true.quiver(0,0,0,m1[0],m1[1],m1[2])
ax_true.quiver(0,0,0,m2[0],m2[1],m2[2])
ax_true.quiver(0,0,0,m3[0],m3[1],m3[2])
ax_true.quiver(0,0,0,m4[0],m4[1],m4[2])
ax_true.set_xlim3d([-0.75, 0.75])
ax_true.set_ylim3d([-0.75, 0.75])
ax_true.set_zlim3d([-0.75, 0.75])


print(angle(m1,m2))
print(angle(m2,m3))
print(angle(m3,m4))
print(angle(m4,m1))



plt.show()