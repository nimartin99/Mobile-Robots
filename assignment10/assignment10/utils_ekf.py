import numpy as np
import matplotlib.pyplot as plt
import math
from matplotlib.patches import Ellipse

landmarks = np.array([[-5,2],[1,0.5],[7,4]])

l = 0.5


def ReadCommands(filename):
    f = open(filename, 'r')
    f.readline()
    startPose = np.array([float(val) for val in f.readline().split(';') ])
    f.readline()
    commands = []
    for line in f:
        if (len(line)>2):    
            commands.append(np.array([float(val) for val in line.split(';') ]))
    return commands, startPose
    

def ReadOdometry(filename):
    f = open(filename, 'r')
    odometry = []
    for line in f:
        if (len(line)>2):    
            odometry.append(np.array([float(val) for val in line.split(';') ]))
    return odometry
    
def ReadMeasurements(filename):
    f = open(filename, 'r')
    measurements = []
    for line in f:
        if (len(line)>2):
            tl = line.split(';')
            ml = [[float(tl[i]),float(tl[i+1])] for i in range(1,len(tl)-1,2)]
            measurements.append( [int(float(tl[0])),   ml]  )
    return measurements


def GetMeasurement(measurements,i):
    tm = [[-1,-1],[-1,-1],[-1,-1]]
    for m in measurements:
        if (m[0] == i):
            return 1, m[1]
    return 0,tm

def RobotToWheel(v,l):
    return np.array([ (2*v[0]-l*v[1])/2.0, (2*v[0]+l*v[1])/2 ])


def WheelToRobot(u,l):
    return np.array([ (u[0]+u[1])/2.0, (u[1]-u[0])/l ])


def CalcDTheta(u,l):
    return (u[1]-u[0])/l   
    
    
def CalcRadius(ws,l):
    if ((ws[1]-ws[0]) == 0):
        return np.inf
    return ((ws[0]+ws[1])*l)/((ws[1]-ws[0])*2)

def UpdatePose(pose, command):
    v, omega,t = command
    x,y,theta = pose
    rv = [v,omega]
    wv = RobotToWheel(rv,l)
    r = CalcRadius(wv*t,l)
    if (r == np.inf):
        dp =  np.array([np.cos(theta)*v*t,np.sin(theta)*v*t,0])
        return pose+ dp
    else:
        dp = np.array( [r*(np.sin(theta+ omega*t) - np.sin(theta) ),r*(-np.cos(theta+ omega*t) + np.cos(theta) ), omega*t ])
        return pose+ dp


def UpdatePoseWheel(pose, control):
    x,y,theta = pose
    t=control[2]
    dtheta = CalcDTheta(control,l)
    r = CalcRadius(control,l)
    if (r == np.inf):
        dp =  np.array([np.cos(theta)*control[0]*t,np.sin(theta)*control[0]*t,0])
        return pose+ dp
    else:
        dp = np.array( [r*(np.sin(theta+ dtheta*t) - np.sin(theta) ),r*(-np.cos(theta+ dtheta*t) + np.cos(theta) ), dtheta*t ])
        return pose+ dp


def GetMinMax(traj):
    border = 1
    points = [[t[0],t[1]] for t in traj ]
    p = np.array(points)
    minT = np.amin(p, axis=0)    
    maxT = np.amax(p, axis=0)
    print ("min max : "+ str([minT[0],maxT[0],minT[1],maxT[1]]) )
    return [minT[0]-border,maxT[0]+border,minT[1]-border,maxT[1]+border]
    
def PlotTrajectoryCommands(startPose,commands):
    fig = plt.figure() 
    trajectory = []
    trajectory.append(startPose)
    curPose = startPose
    for c in commands:
        curPose = UpdatePose(curPose,c)
        trajectory.append(curPose)
        
    pTra = [t[:2] for t in trajectory ]
    plt.plot(*zip(*pTra), marker='o', color='r', ls='solid')     
    for t in trajectory:
        plt.plot([t[0],t[0]+np.cos(t[2])],[t[1],t[1]+np.sin(t[2])], color='b', ls='solid')     
    plt.axis(GetMinMax(trajectory))
    plt.plot(*zip(*landmarks), marker='o', color='b', ls='solid')    
    plt.show()
    
    
def PlotTrajectory(trajectory):            
    fig = plt.figure() 
    pTra = [t[:2] for t in trajectory ]
    plt.plot(*zip(*pTra), marker='o', color='r', ls='solid')     
    for t in trajectory:
        plt.plot([t[0],t[0]+np.cos(t[2])],[t[1],t[1]+np.sin(t[2])], color='b', ls='solid')     
    plt.axis(GetMinMax(trajectory))
    plt.plot(*zip(*landmarks), marker='o', color='b', ls='solid')    
    plt.show()
    
def PlotCommandMeasurements(startPose,commands,measurements):
    fig = plt.figure() 
    trajectory = []
    trajectory.append(startPose)
    curPose = startPose
    for c in commands:
        curPose = UpdatePose(curPose,c)
        trajectory.append(curPose)
            
    pTra = [t[:2] for t in trajectory ]
    plt.plot(*zip(*pTra), marker='o', color='r', ls='solid')     
    for i in range(0,len(trajectory)):
        t = trajectory[i]
        plt.plot([t[0],t[0]+np.cos(t[2])],[t[1],t[1]+np.sin(t[2])], color='b', ls='solid') 
        hasM, m = GetMeasurement(measurements,i-1)    
        if (hasM==1):  
            for tm in m:
                plt.plot([t[0],t[0]+np.cos(t[2]+tm[1])*tm[0]],[t[1],t[1]+np.sin(t[2]+tm[1])*tm[0]], color='g', ls='solid') 
        
    plt.axis(GetMinMax(trajectory))
    plt.plot(*zip(*landmarks), marker='o', color='b', ls='solid')
    plt.show()
    
def PlotTrajectoryMeasurements(trajectory,measurements):
    fig = plt.figure() 
    pTra = [t[:2] for t in trajectory ]
    plt.plot(*zip(*pTra), marker='o', color='r', ls='solid')     
    for i in range(0,len(trajectory)):
        t = trajectory[i]
        plt.plot([t[0],t[0]+np.cos(t[2])],[t[1],t[1]+np.sin(t[2])], color='b', ls='solid') 
        hasM, m = GetMeasurement(measurements,i)  
        
        if (hasM==1):  
            for tm in m:
                #print(tm)
                plt.plot([t[0],t[0]+np.cos(t[2]+tm[1])*tm[0]],[t[1],t[1]+np.sin(t[2]+tm[1])*tm[0]], color='g', ls='solid') 
        
    plt.axis(GetMinMax(trajectory))
    plt.plot(*zip(*landmarks), marker='o', color='b', ls='solid')
    plt.show()


def OdomToTrajectory(startPose,controls):
    trajectory = []
    trajectory.append(startPose)
    curPose = startPose
    for c in controls:
        curPose = UpdatePoseWheel(curPose,c)
        trajectory.append(curPose)
    return trajectory

def PlotTrajectoryControl(startPose,controls):
    fig = plt.figure() 
    trajectory = []
    trajectory.append(startPose)
    curPose = startPose
    for c in controls:
        curPose = UpdatePoseWheel(curPose,c)
        trajectory.append(curPose)
        
    pTra = [t[:2] for t in trajectory ]
    plt.plot(*zip(*pTra), marker='o', color='r', ls='solid')     
    for t in trajectory:
        plt.plot([t[0],t[0]+np.cos(t[2])],[t[1],t[1]+np.sin(t[2])], color='b', ls='solid')     
    
    plt.plot(*zip(*landmarks), marker='o', color='b', ls='solid')    
    plt.axis(GetMinMax(trajectory))
    plt.show()

    
def PlotTrajectorySample(startPose,commands,sampleRate):
    fig = plt.figure() 
    trajectoryS = []
    trajectoryS.append(startPose)
    curPoseS = startPose
    for c in commands:
        tcom = c
        for t in range(0,int(c[2]/sampleRate)):
            tcom[2] = sampleRate
            curPoseS = UpdatePose(curPoseS,tcom)
            trajectoryS.append(curPoseS)
    
    pTraS = [t[:2] for t in trajectoryS ]
    
    plt.plot(*zip(*pTraS),color='g', ls='solid') 
    plt.axis(GetMinMax(trajectory))
    plt.show()
    
    
def PlotEllipse(cov, pos, volume=.01, ax=None, fc='none', ec=[0,0,0], a=1, lw=1):
    """
    Plots an ellipse enclosing *volume* based on the specified covariance
    matrix (*cov*) and location (*pos*). Additional keyword arguments are passed on to the 
    ellipse patch artist.

    Parameters
    ----------
        cov : The 2x2 covariance matrix to base the ellipse on
        pos : The location of the center of the ellipse. Expects a 2-element
            sequence of [x0, y0].
        volume : The volume inside the ellipse; defaults to 0.5
        ax : The axis that the ellipse will be plotted on. Defaults to the 
            current axis.
    """
    from scipy.stats import chi2
    
    def eigsorted(cov):
        vals, vecs = np.linalg.eigh(cov)
        order = vals.argsort()[::-1]
        return vals[order], vecs[:,order]

    if ax is None:
        ax = plt.gca()

    vals, vecs = eigsorted(cov)
    theta = np.degrees(np.arctan2(*vecs[:,0][::-1]))

    kwrg = {'facecolor':fc, 'edgecolor':ec, 'alpha':a, 'linewidth':lw}

    # Width and height are "full" widths, not radius
    width, height = 2 * np.sqrt(chi2.ppf(volume,2)) * np.sqrt(vals)
    ellip = Ellipse(xy=pos, width=width, height=height, angle=theta, **kwrg)

    ax.add_artist(ellip)
    
    
    
def PlotStates(states,covariance):
    fig = plt.figure() 
    pTra = [t[:2] for t in states ]
    plt.plot(*zip(*pTra), marker='o', color='r', ls='solid')     
    for i in range(0,len(states)):
        t = states[i]
        cov = covariance[i]
        plt.plot([t[0],t[0]+np.cos(t[2])],[t[1],t[1]+np.sin(t[2])], color='b', ls='solid') 
        PlotEllipse(cov[0:2,0:2],t)
            
    plt.axis(GetMinMax(pTra))
    plt.plot(*zip(*landmarks), marker='o', color='b', ls='solid')    
    plt.show()
    
    
def PlotStatesMeasurementes(states,covariance,measurements):
    fig = plt.figure() 
    pTra = [t[:2] for t in states ]
    plt.plot(*zip(*pTra), marker='o', color='r', ls='solid')     
    for i in range(0,len(states)):
        t = states[i]
        cov = covariance[i]
        plt.plot([t[0],t[0]+np.cos(t[2])],[t[1],t[1]+np.sin(t[2])], color='b', ls='solid') 
        PlotEllipse(cov[0:2,0:2],t)
        hasM, m = GetMeasurement(measurements,i-1)    
        if (hasM==1):  
            for tm in m:
                plt.plot([t[0],t[0]+np.cos(t[2]+tm[1])*tm[0]],[t[1],t[1]+np.sin(t[2]+tm[1])*tm[0]], color='g', ls='solid') 
        
            
    plt.axis(GetMinMax(pTra))
    plt.plot(*zip(*landmarks), marker='o', color='b', ls='solid')    
    plt.show()   
