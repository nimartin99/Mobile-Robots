import numpy as np
import matplotlib.pyplot as plt
import math
from matplotlib.patches import Ellipse

import utils_ekf as utekf


plt.close("all")


""" colors for plot """
robot_colors = ['r', 'g', 'b', 'y'] 

""" number of robots """
n = len(robot_colors)


""" read measurements and start poses """
noiseMeasurements = utekf.ReadOdometry('measurements.txt')
startPoses = utekf.ReadOdometry('start.txt')[0]

startPositions = []
for i in range(0,n):
    startPositions.append( np.array([startPoses[i*2],startPoses[i*2+1]]) )

""" start positions, one per row """
startPositions = np.array(startPositions)

""" arrays for plotting all measures """
npMeasures = np.array(noiseMeasurements)
npMeasures = npMeasures.reshape(len(noiseMeasurements)*n,2)


""" 
Plot functions 
"""
def PlotPositions(pos):            
    fig = plt.figure() 
    ax = fig.add_subplot(111)
    ax.set_aspect('equal', adjustable='box')
    x = [t[0] for t in pos ]
    y = [t[1] for t in pos ]
    plt.scatter(x,y, marker='o', color='k', s=5)
    plt.show()  

def PlotPositionsV2(pos):            
    x = [t[0] for t in pos ]
    y = [t[1] for t in pos ]
    plt.scatter(x,y, marker='o', color='k', s=5)

def PlotStates(states,covariances,col):
    pTra = [ t[:2] for t in states ]
    plt.plot(*zip(*pTra), marker='o', color=col, ls='solid', ms=1)
    for i in range(0,len(states)):
        t = states[i]
        cov = covariances[i]
        plt.plot([t[0],t[0]+np.cos(t[2])],[t[1],t[1]+np.sin(t[2])], color=col, ls='solid')
        utekf.PlotEllipse(cov[0:2,0:2],t, ec=col)
    


""" plot all measurements """
#PlotPositions(npMeasures)
    
    
"""  
a) Definition of At and Ct here
"""

""" At depends on the step size dt. """
def TransitionMatrix(dt):
    #TODO
    At = np.identity(6)
    At[0,2] = dt
    At[1,3] = dt
    At[2,4] = dt
    At[3,5] = dt

    return At

#TODO
Ct = np.zeros(shape=(2,6))
Ct[0,0] = 1
Ct[1,1] = 1

"""  
 Create process noise R, measurement noise Q and initial covariance matrices CI here
"""
R = np.identity(6)*10.0   #100.0
Q = np.identity(2)*400.0  #1000.0
CI = np.identity(6)*400.0 #10000.0


"""  
 Set up a Kalman filter class and create 3 instances of it
"""
class KalmanFilter(object):

    def __init__(self, state, covar):
        self.state = state
        self.covar = covar

    """
    b) Prediction step
    """
    def predict(self, dt):
        """predict for time step dt. """       
        At = TransitionMatrix(dt)
        self.state = At.dot(self.state)
        self.covar = At@self.covar@np.transpose(At) + R


    """
    d) Update step
    """
    def update(self, m):
        """ update for measurement m. """
        K = self.covar@np.transpose(Ct) @ np.linalg.pinv(Ct@self.covar@np.transpose(Ct)+Q)
        self.state = self.state + K@(m-Ct@self.state)
        self.covar = (np.identity(np.shape(K@Ct)[0]) - K@Ct) @ self.covar


    """
    c) Correspondence evaluation between predicted position and measured position m
    """
    def getProbability(self, m):
        """ calculate probability that a measurement m corresponds to this instance. """

        # measurement based on current state:
        m_predict = Ct@self.state
        # compute malahonobis distance (distance between point and distribution)
        covar_position = np.array([[self.covar[0, 0], self.covar[0, 1]], [self.covar[1,0], self.covar[1,1]]])
        return np.sqrt(np.transpose(m - m_predict)@np.linalg.pinv(covar_position)@(m- m_predict))


""" 
Create and initialize a Kalman filter for each start point. 
"""
filters = []
for i in range(0, n):
    init_state = np.array([startPositions[i,0], startPositions[i,1], 0, 0, 0, 0])
    filters.append(KalmanFilter(state=init_state ,covar=CI))


"""  
Loop over all measurements. Each measurment row contains 4 positions, one for each robot. 
"""
timeStep = 1.0
positions = [[] for _ in range(n)]
covariances = [[] for _ in range(n)]
for measures in noiseMeasurements:
    measures = measures.reshape(len(measures)//2,2)
    m1 = measures[0]
    m2 = measures[1]
    m3 = measures[2]
    m4 = measures[3]
    
    """ b) predict all filters. """
    for i in range(0, n):
        filters[i].predict(timeStep)
        """ c) solve correspondence problem: match filters and measurements. 
        Hint: use the function 'getProbability'. """
        closest_measurement = 0
        m_distance = float("inf")
        for j in range(0, 4):
            m = measures[j]
            p = filters[i].getProbability(m)
            if p < m_distance:
                m_distance = p
                closest_measurement = m

        """ d) update all filters. accumulate positions and covariances for plotting. """
        filters[i].update(closest_measurement)
        positions[i].append(filters[i].state)
        covariances[i].append(filters[i].covar)



"""
d) Plot the trajectories together with the measurements.
"""
fig = plt.figure() 
ax = fig.add_subplot(111)

""" plot results (use PlotStates to plot the trajectories and a scatter plot for the measurements) """
for i in range(0, n):
    PlotStates(positions[i], covariances[i], robot_colors[i])

PlotPositionsV2(npMeasures)

ax.set_aspect('equal', adjustable='box')
plt.show()
