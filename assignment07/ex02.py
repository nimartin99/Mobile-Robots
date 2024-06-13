import numpy as np


### a)

t_x = 0.2
t_y = 0.1
t_z = 0
t_X = np.array([[0, -1*t_z, t_y], [t_z, 0, -1*t_x], [-1*t_y, t_x, 0]])

R = np.array([[np.cos(np.pi / 60), 0, np.sin(np.pi / 60)], [0, 1, 0], [-1*np.sin(np.pi / 60), 0, np.cos(np.pi / 60)]])
E = t_X @ R

L_T_R = np.array([[np.cos(np.pi / 60), 0, np.sin(np.pi / 60), 0.2], [0, 1, 0, 0.1], [-1*np.sin(np.pi / 60), 0, np.cos(np.pi / 60), 0], [0, 0, 0, 1]])

print("Essential Matrix: \n", E)


### b)
pl = np.array([30,50,1])
pl_homog = np.array([30, 50, 1, 1])
pr = np.linalg.inv(L_T_R) @ pl_homog

pr1 = np.array([11, 19, 1])
pr2 = np.array([11, 20, 1])
pr3 = np.array([12, 19, 1])
pr4 = np.array([12, 20, 1])


print("p_R1: ", pl @ (E @ pr1))
print("p_R2: ", pl @ E @ pr2)
print("p_R3: ", pl @ E @ pr3)
print("p_R4: ", pl @ E @ pr4)

print("pr: ", pr)
print("r_Rcomp: ", pl@E@pr[:3])


### c)
pr5 = np.array([9, 27, 1, 1])
pl5 = L_T_R @ pr5
print("Point in Left Cam-CS: ", pl5)