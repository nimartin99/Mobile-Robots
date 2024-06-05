import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def rotZ(angle):
    return np.array([[np.cos(angle), -1* np.sin(angle), 0],
                     [np.sin(angle), np.cos(angle), 0],
                     [0, 0, 1]])

def rotX(angle):
    return np.array([[1 , 0, 0],
                     [0, np.cos(angle), -1*np.sin(angle)],
                     [0, np.sin(angle), np.cos(angle)]])

W_R_S = rotZ(215/180 * np.pi) @ rotX(70/180 * np.pi)
R = W_R_S
print("Rot: \n", R)

origin = np.array([0, 0, 0])
x = np.array([1, 0, 0])
y = np.array([0, 1, 0])
z = np.array([0, 0, 1])

x_new = W_R_S @ x
y_new = W_R_S @ y
z_new = W_R_S @ z

origin2 = np.array([1, 2, 2])

def plotVect(ax, origin, direction, clr):
    ax.quiver(origin[0], origin[1], origin[2], direction[0], direction[1], direction[2], color = clr)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
plotVect(ax, origin, x, "red")
plotVect(ax, origin, y, "green")
plotVect(ax, origin, z, "blue")

plotVect(ax, origin2, x_new, "red")
plotVect(ax, origin2, y_new, "green")
plotVect(ax, origin2, z_new, "blue")

ax.scatter(4, -4, 0)
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_xlim([-3, 3])
ax.set_ylim([-3, 3])
ax.set_zlim([-3, 3])
plt.show()


# Transformation matrix
W_T_C = np.array([[R[0,0], R[0,1], R[0,2], 1], [R[1,0], R[1,1], R[1,2], 2], [R[2,0], R[2,1], R[2,2], 2], [0, 0, 0, 1]])
print("W_R_S: \n", W_T_C)
C_T_W = np.linalg.inv(W_T_C)
print("C_T_W: \n", C_T_W)

# --------------- b) ---------------
p_W = np.array([4, -4, 0, 1])
p_C = C_T_W @ p_W
print("p_C: ", p_C) 
print(np.linalg.norm(p_C))
