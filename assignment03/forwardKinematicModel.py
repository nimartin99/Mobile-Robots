# Mobile Robots, SoSe 2023
# Exercise 2
# student 1:
# student 2:


import numpy as np
import matplotlib.pyplot as plt


# read commands from file
f = open('commands.txt', 'r')
f.readline()
startPose = np.array([float(val) for val in f.readline().split(';')])
f.readline()
commands = []
for line in f:
    if (len(line)>2):    
        commands.append(np.array([float(val) for val in line.split(';')]))


# a) Pose update function ********************************
def f(x_, u, t):
    """
    Updates the robot's pose given the old pose 'x_' and a command 'u' for a certain duration 't'
    :param x_: old pose [x, y, theta]
    :param u: command [v, w]
    :param t: duration
    :return: new pose [x_new, y_new, theta_new]
    """
    x, y, theta = x_  # old pose
    v, w = u          # command

    """
    TODO: Update the robot's pose according to the kinematic model presented in the lecture
    """
    l = 0.5     #length parameter

    # compute the distance traveled by both wheels
    v_r = (2*v + l*w) / 2
    v_l = (2*v - l*w) / 2
    s_r = v_r * t
    s_l = v_l * t
 
    # compute delta_x, delta_y and delta_theta
    delta_x = 0
    delta_y = 0
    delta_theta = 0
    if s_r != s_l:
        # compute radius and contour of arc if robot drives a curve 
        r = (s_l + s_r)/(s_r - s_l) * l/2
        delta_theta = (s_r - s_l)/l
        delta_x = r*(np.sin(theta + delta_theta) - np.sin(theta))
        delta_y = r*(-np.cos(theta + delta_theta) + np.cos(theta))
    else:
        delta_x = s_r * np.cos(theta)
        delta_y = s_r * np.sin(theta)
        
    x_new = x + delta_x
    y_new = y + delta_y
    theta_new = theta + delta_theta

    return np.array([x_new, y_new, theta_new])


# b) generate trajectory ********************************
"""
TODO: Create a list of poses. One pose for every millisecond. 
"""
poses = []
current_pose = startPose

for command in commands:
    for t in range(0, int(command[-1]*1000)):
        next_pose = f(current_pose, command[:2], 0.001)
        current_pose = next_pose
        poses.append(next_pose)




# c) plot trajectory ********************************
plt.figure('robot\'s trajectory')
plt.title('robot\'s trajectory')
plt.xlabel('x')
plt.ylabel('y')

"""
TODO: Extract a list of x- and y-coordinates from your pose list. Plot those coordinates.
"""
poses = np.asarray(poses)
xs = poses[:, 0]
ys = poses[:, 1]

plt.plot(xs, ys)
plt.show()

