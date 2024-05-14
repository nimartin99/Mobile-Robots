import numpy as np
import matplotlib.pyplot as plt
import math


# example trajectory
def x0(t):
    """
    Example trajectory that doesn't move the robot
    Returns:
        Pose as triple (x,y,theta)
    """
    return (0,0,0)


"""
IMPLEMENT THE TRAJECTORY FUNCTIONS HERE
"""
# function x1
def x1(t):
    a = math.pi / 150
    b = math.sqrt(2) / (math.sin(a*t)**2 + 1)
    x = b * math.sin(a*t)
    y = b * math.sin(a*t) * math.cos(a*t)
    theta = a * t
    return (x, y, theta)

# function x2
def x2(t):
    a = math.pi / 150
    b = (math.exp(math.cos(a*t)) - 2*math.cos(4*a*t) - math.sin(a*t / 12)**5) / 2
    x = b * math.sin(a*t)
    y = b * math.cos(a*t)
    theta = 0
    return (x, y, theta)

# function x3
def x3(t):
    a = 3*math.pi / 150
    b = 2*math.pi / 150
    c = math.pi / 150
    s = 1
    x = s * math.cos(a*t)
    y = s * math.sin(b*t)
    theta = c * t
    return (x, y, theta)

# function x3 modified for theta
def x3_modified(t):
    a = 3*math.pi / 150
    b = 2*math.pi / 150
    c = math.pi / 150
    s = 1
    x = s * math.cos(a*t)
    y = s * math.sin(b*t)
    # set theta to angle of vector pointing from current to next point
    x_next = s * math.cos(a*(t+1))
    y_next = s * math.sin(b*(t+1)) 
    theta = math.atan2(y_next- y, x_next - x)
    return (x, y, theta)

def transformation_matrix(x, y, theta):
    """
    Returns Transformation matrix with rotation around angle theta and offset by x and y
    """
    return np.array([[math.cos(theta), -1 * math.sin(theta), x], [math.sin(theta), math.cos(theta), y], [0, 0, 1]])

def compute_velocities(poses):
    """
    Computes Velocities in Robot Frame based on poses
    """
    x, y, theta = poses
    vel_x = []
    vel_y = []
    omega = []
    for i in range(0, len(x)-1):
        # transformation matrix from world to robot frame
        w_R_r = transformation_matrix(x[i], y[i], theta[i])
        # inverse to transform point from W into R
        r_R_w = np.linalg.inv(w_R_r)
        # compute point of next position in Robot frame, the resulting vector is the velocity in robot frame, since Δt is 1 sec between two steps
        p_w = np.array([x[i+1], y[i+1], 1])
        vel_R = r_R_w @ p_w

        # also compute delta_theta (= omega)
        delta_theta = theta[i+1] - theta[i]
        omega_ = (delta_theta + math.pi) % (math.pi * 2) - math.pi

        vel_x.append(vel_R[0])
        vel_y.append(vel_R[1])
        omega.append(omega_)

    return (vel_x, vel_y, omega)

def compute_wheel_velocites(vel_R_Frame, l = 0.4):
    """
    Computes individual Wheel Velocities (v1, v2, v3) for Robot based on the velocity Vector in the R Frame and Omega
    Parameters:
        vel_R_Frame - tuple of list of velocities in Robot frame (vx[], vy[], omega[])
        l - Length of wheel axes, default value: 0.4 m
    """
    vel_x, vel_y, omega = vel_R_Frame
    
    # define matrix from inverse kinematics to compute v1, v2, v3
    inv_kin_matrix = np.array([[math.sqrt(3)/2, -1/2, -l], [0, 1, -l], [-math.sqrt(3)/2, -1/2, -l]])

    v1 = []
    v2 = []
    v3 = []

    for i in range(0, len(vel_x)):
        vel_vec_wheels = inv_kin_matrix @ np.array([vel_x[i], vel_y[i], omega[i]])
        v1.append(vel_vec_wheels[0])
        v2.append(vel_vec_wheels[1])
        v3.append(vel_vec_wheels[2])

    return (v1, v2, v3)

    
def plotWheelVelocities(name, time, wheel_velocity):
    """
    Plots Wheel Velocities over time t
    """
    v1, v2, v3 = wheel_velocity
    fig = plt.figure(layout='constrained')
    v1_plot = fig.add_subplot(311)
    v2_plot = fig.add_subplot(312)
    v3_plot = fig.add_subplot(313)
    v1_plot.plot(time[:-1], v1)
    v2_plot.plot(time[:-1], v2)
    v3_plot.plot(time[:-1], v3)

    v1_plot.set_title("Velocity Wheel 1 (v1): " + name)
    v2_plot.set_title("Velocity Wheel 2 (v2): " + name)
    v3_plot.set_title("Velocity Wheel 3 (v3): " + name)
    v1_plot.set_xlabel('x [sec]')
    v1_plot.set_ylabel('y [m/sec]')
    v2_plot.set_xlabel('x [sec]')
    v2_plot.set_ylabel('y [m/sec]')
    v3_plot.set_xlabel('x [sec]')
    v3_plot.set_ylabel('y [m/sec]')

def plotVelocity(name, time, velocity):
    """
    Plots Velocities and Omega in Robot frame over time t
    """
    x, y, omega = velocity
    fig = plt.figure(layout='constrained')
    vx_plot = fig.add_subplot(311)
    vy_plot = fig.add_subplot(312)
    omega_plot = fig.add_subplot(313)
    #fig.tight_layout(h_pad=4)
    vx_plot.plot(time[:-1], x)
    vy_plot.plot(time[:-1], y)
    omega_plot.plot(time[:-1], omega)

    vx_plot.set_title("v_x Robot Frame: " + name)
    vy_plot.set_title("v_y Robot Frame: " + name)
    omega_plot.set_title("Omega: " + name)
    vx_plot.set_xlabel('x [sec]')
    vx_plot.set_ylabel('y [m/sec]')
    vy_plot.set_xlabel('x [sec]')
    vy_plot.set_ylabel('y [m/sec]')
    omega_plot.set_xlabel('x [sec]')
    omega_plot.set_ylabel('y [rad/sec]')

def plotVelinGlobalFrame(name, time, x, y, theta, velocity, arrow_scale=5, n=6, ticks=25):
    """
    Plots trajectory and velocity vectors of robot
    Parameters:
        name - Used for the title of the plotting window
        time - List of all time points (seconds)
        x - List of all x coordinates
        y - List of all y coordinates
        theta - List of all orientations
        velocity - tuple of list of velocities in Robot frame (vx[], vy[], omega[])
        arrow_scale - Scale for the rendered coordinate frames
        n - Every nth pose will be rendered with a frame
        ticks - Every ticks-th pose will be labelled with the time point
    """
    vx, vy, omega = velocity
    # rotate vel from R-Frame to W-Frame
    xp = []
    yp = []
    for i in range(0, len(vx)):
        xp.append(math.cos(theta[i]) * vx[i] - math.sin(theta[i]) * vy[i])
        yp.append(math.sin(theta[i]) * vx[i] + math.cos(theta[i]) * vy[i])

    fig = plt.figure()
    fig.canvas.setWindowTitle("trajectory with velocity " + name)

    trajectory = fig.add_subplot(111)
    trajectory.set_title("trajectory with velocity: " + name)
    trajectory.set_xlabel('x [m]')
    trajectory.set_ylabel('y [m]')

    trajectory.plot(x, y)
    trajectory.plot(x[::ticks], y[::ticks], 'ro')
    for i in range(0, len(time), ticks):
        trajectory.text(x[i], y[i], "t=" + str(time[i]))

    # plot vel vector
    trajectory.quiver(x[::n], y[::n], xp[::n], yp[::n], angles='xy', scale_units='xy', units='dots', scale=0.5, color='g')

    miny, maxy = trajectory.get_ylim()
    minx, maxx = trajectory.get_xlim()
    trajectory.set_ylim((miny, maxy))
    trajectory.set_xlim((minx, maxx))
    trajectory.set_aspect(1)

# function to plot trajectory (does not need to be modified)
def plotTrajectory(name, time, x, y, theta, arrow_scale=0.3, n=6, ticks=25):
    """
    Plots a named trajectory
    Parameters:
        name - Used for the title of the plotting window
        time - List of all time points (seconds)
        x - List of all x coordinates
        y - List of all y coordinates
        theta - List of all orientations
        arrow_scale - Scale for the rendered coordinate frames
        n - Every nth pose will be rendered with a frame
        ticks - Every ticks-th pose will be labelled with the time point
    """

    # prepare plot
    fig = plt.figure()

    # had to change this from 'set_window_title' to 'setWindowTitle'
    #fig.canvas.set_window_title("trajectory " + name)
    fig.canvas.setWindowTitle("trajectory " + name)

    trajectory = fig.add_subplot(111)
    trajectory.set_title("trajectory and orientation: " + name)
    trajectory.set_xlabel('x [m]')
    trajectory.set_ylabel('y [m]')

    # plot trajectory
    trajectory.plot(x, y)

    # plot labels
    trajectory.plot(x[::ticks], y[::ticks], 'ro')
    for i in range(0, len(time), ticks):
        trajectory.text(x[i], y[i], "t=" + str(time[i]))

    # frame parameters
    xu = [arrow_scale * d for d in map(np.cos, theta)]
    xv = [arrow_scale * d for d in map(np.sin, theta)]
    yu = [-arrow_scale * d for d in map(np.sin, theta)]
    yv = [arrow_scale * d for d in map(np.cos, theta)]

    # plot looking direction
    # trajectory.quiver(x[::n],y[::n], xu[::n],xv[::n], scale=0.1,width=0.0001,color='black')

    # plot x axis
    trajectory.quiver(x[::n], y[::n], xu[::n], xv[::n], angles='xy', scale_units='xy', units='dots', scale=2, color='r')

    # plot y axis
    trajectory.quiver(x[::n], y[::n], yu[::n], yv[::n], angles='xy', scale_units='xy', units='dots', scale=2, color='g')

    # resize plot to generate some white space at the border
    miny, maxy = trajectory.get_ylim()
    minx, maxx = trajectory.get_xlim()
    trajectory.set_ylim((miny - arrow_scale, maxy + arrow_scale))
    trajectory.set_xlim((minx - arrow_scale, maxx + arrow_scale))
    trajectory.set_aspect(1)


"""
IMPLEMENT FUNCTION TO ANALYZE THE TRAJECTORY AND DRAW THE PLOTS
"""
# main function
def analyzeTrajectory(trajectory, t):
    """
    Analyzes a trajectory function
    Parameters:
        trajectory - a trajectory function that maps time to a pose
        t - number of time steps in seconds
    """

    """
    YOUR CODE GOES HERE
    """

    # a) visualize x_1, x_2 and x_3 trajectories

    # you can easily generate lists of coordinates for each component using zip
    # for example trajectory is evaluated at times 1,2,3
    #x, y, theta = zip(trajectory(1), trajectory(2), trajectory(3))
    # but now do this for t time steps
    x = []
    y = []
    theta = []
    for t_ in range(0, t):
        x.append(trajectory(t_)[0])
        y.append(trajectory(t_)[1])
        theta.append(trajectory(t_)[2])

    # visualize:
    plotTrajectory(trajectory.__name__,range(0,t), x, y, theta) 


    # b) visualize robots velocities (in robot frame)
    # - extract poses from trajectories
    # - use the difference in poses to estimate the world velocities
    # - transform world velocities into robot velocities
    # - visualize
    velocities = compute_velocities((x,y,theta))
    plotVelocity(trajectory.__name__, range(0,t), velocities)
    plotVelinGlobalFrame(trajectory.__name__, range(0,t), x, y, theta, velocities)

    # c) visualize wheel velocities
    # - transform robot velocities into wheel velocities
    # - visualize
    v_wheels = compute_wheel_velocites(velocities)
    plotWheelVelocities(trajectory.__name__, range(0,t), v_wheels)


# close all old plots
plt.close("all")

# plot everything for the given function for three time steps
analyzeTrajectory(x0, 3)
analyzeTrajectory(x1, 300)
analyzeTrajectory(x2, 300)
analyzeTrajectory(x3, 300)
analyzeTrajectory(x3_modified, 300)

# plot the trajectories from the exercise sheet for the given time steps
plt.show()
