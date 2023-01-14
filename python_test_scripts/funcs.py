import numpy as np

def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


def calculate_normal_vec(g_vec: np.ndarray):
    svd = np.linalg.svd(g_vec - np.mean(g_vec, axis=1, keepdims=True))
    left = svd[0]
    line = left[:, -1]
    print("SVD magnitude:",np.linalg.norm(line))
    return line

def calc_true_vec(normal_vec: np.ndarray, laser_distances: np.ndarray, disto_len: float):
    laser_length = np.mean(laser_distances)
    x_axis = np.array([1,0,0]).T

    b1 = normal_vec
    b2 = np.array([[1],[0],[0]])

    # Get angle between true direction of target and direction of device
    alpha = np.arccos(np.dot(b1,b2) / np.linalg.norm(b1)*np.linalg.norm(b2))
    if alpha > np.pi/2:
        alpha = np.pi-alpha
        b1 = -b1
    print("alpha:",np.rad2deg(alpha))
    print("normal_vec:", b1)


    l_0 = disto_len
    l_1 = laser_length
    l_2 = l_0 * np.sin(alpha)
    l_3 = l_0 * np.cos(alpha) + np.sqrt(l_1**2-l_2**2)

    true_vec = l_3 * x_axis + l_0 * normal_vec
    return true_vec

def get_base_change_mat(A, B):
    """Gets the basis change vector changinf from A to B"""

    # Find rotation of x-axis to new vector
    # i.e. basis matrix:
    # https://math.stackexchange.com/a/897677

    print(A, B)

    AdotB = (A.T @ B)[0][0]
    AcrossB = np.cross(A,B,axis=0)
    BcrossA = np.cross(B,A,axis=0)
    AcrossBnorm = np.linalg.norm(AcrossB)
    G = np.array([[AdotB      , -AcrossBnorm, 0],
        [AcrossBnorm, AdotB       , 0],
        [0          , 0           , 1]])

    numerator = B - AdotB * A
    denominator = np.linalg.norm(numerator)

    Finv = np.array([A.T[0] , (numerator/denominator).T[0], BcrossA.T[0]]).T
    F = np.linalg.inv(Finv)

    return F, Finv , G












import matplotlib.pyplot as plt
# Create graphs
fig = plt.figure(figsize = (8,8))
ax = plt.axes(projection='3d')
ax.grid()
ax.set_title('3D Parametric Plot')

# Set axes label
ax.set_xlabel('x', labelpad=20)
ax.set_ylabel('y', labelpad=20)
ax.set_zlabel('t', labelpad=20)

fig2 = plt.figure(figsize = (8,8))
ax2 = plt.axes(projection='3d')
ax2.grid()
ax2.set_title('3D Parametric Plot')

# Set axes label
ax.set_xlabel('x', labelpad=20)
ax.set_ylabel('y', labelpad=20)
ax.set_zlabel('t', labelpad=20)


x_axis = np.array([[1],[0],[0]])

# Set length of disto
disto_len = 0.1
"""Length of disto from base to tip"""

# Set laser misalignement vector
misalign_vec = np.array([[1],[0.5],[0.5]])
"""Direction of fire or laser where straight is [1,0,0]"""

# Turn into unit vec
misalign_vec = misalign_vec/np.linalg.norm(misalign_vec)


initial_norm = np.array([[0],[0],[0.2]])
device_len = 0.4
initial_device_vec = np.array([[device_len],[0],[0]])
target_vec = (initial_device_vec.T + misalign_vec.T * 0.6).T

initial_norm += initial_device_vec

F, Finv, G = get_base_change_mat(target_vec,x_axis)


# Finv @ G @ F @ vec @ A to rotate A to B

ax2.plot3D([0,initial_device_vec[0]],[0,initial_device_vec[1]],[0,initial_device_vec[2]],c="blue")
ax2.plot3D([initial_device_vec[0], target_vec[0]], [initial_device_vec[1], target_vec[1]], [initial_device_vec[2], target_vec[2]],c="red")
ax2.plot3D([0,target_vec[0]],[0,target_vec[1]],[0,target_vec[2]],c="black")
ax2.plot3D([initial_device_vec[0],initial_norm[0]],[initial_device_vec[1],initial_norm[1]],[initial_device_vec[2],initial_norm[2]],c="blue")


ax2.set_xlim(0,1)
ax2.set_ylim(0,0.5)
ax2.set_zlim(0,0.5)


# Take 5 shots, all pointing at the target_vec
for shot_num in range(10):
    theta = shot_num * (2*np.pi/10)
    # Calculate device_vector
    # Calculate rotation matrix, T. Where T is the rotation about the target vector
    x_axis_rotation = np.array([
    [1,0,0],
    [0,np.cos(theta),-np.sin(theta)],
    [0,np.sin(theta),np.cos(theta)]])

    device_vec = x_axis_rotation @ initial_device_vec
    normal_vec = x_axis_rotation @ initial_norm


    #ax.plot3D([0,device_vec[0],target_vec[0]],[0,device_vec[1],target_vec[1]],[0,device_vec[2],target_vec[2]])
    ax.plot3D([0,device_vec[0]],[0,device_vec[1]],[0,device_vec[2]],c="blue")
    ax.plot3D([device_vec[0],normal_vec[0]],[device_vec[1],normal_vec[1]],[device_vec[2],normal_vec[2]],c="blue")
    ax.plot3D([device_vec[0],target_vec[0]],[device_vec[1],target_vec[1]],[device_vec[2],target_vec[2]],c="red")


ax.plot3D([0,target_vec[0]],[0,target_vec[1]],[0,target_vec[2]],c="black")
ax.set_xlim(0,1)
ax.set_ylim(0,0.5)
ax.set_zlim(0,0.5)


plt.show()
