# -*- coding: utf-8 -*-
"""
Created on Thu May  3 14:01:38 2018

@author: jordan
"""
import numpy

"""
re-edited: 05/2022
"""

import numpy as np
import matplotlib.pyplot as plt
import math
from matplotlib.patches import Ellipse


"""

Utility Functions for Plotting

"""
###############################################################################
def Get2DArray(mean, cov, nsamples = 100, scalef = 1.0):
    result = np.zeros((nsamples, nsamples))
    xmin = mean[0]-cov[0][0]*scalef
    xmax = mean[0]+cov[0][0]*scalef
    ymin = mean[1]-cov[1][1]*scalef
    ymax = mean[1]+cov[1][1]*scalef

    xarr = np.linspace(xmin,xmax,nsamples)   
    yarr = np.linspace(ymin,ymax,nsamples)   

    for i in range(0,len(xarr)):
        for j in range(0,len(yarr)):
            result[j][i] = GetProb([xarr[i],yarr[j]],mean,cov)

    return result    

def Get2DArrayMinMax(mean, cov, minmax, nsamples = 100):
    result = np.zeros((nsamples, nsamples))
    xmin = minmax[0]    
    xmax = minmax[1]
    ymin = minmax[2]
    ymax = minmax[3]

    xarr = np.linspace(xmin,xmax,nsamples)   
    yarr = np.linspace(ymin,ymax,nsamples)   
    maxVal = 0
    for i in range(0,len(xarr)):
        for j in range(0,len(yarr)):
            result[len(yarr)-1-j][i] = GetProb([xarr[i],yarr[j]],mean,cov)
        if (result[len(yarr)-1-j][i] > maxVal): 
            maxVal = result[len(yarr)-1-j][i]

    return result    


def Plot2DGrid(mean, cov, minmax, nsamples = 100):
    img = Get2DArrayMinMax(mean, cov, minmax, nsamples)
    fig, ax = plt.subplots(figsize=(6,6))
    cax = ax.imshow(img, cmap=plt.cm.rainbow, interpolation='none', extent=[minmax[2],minmax[3],minmax[0],minmax[1]], aspect="auto")
    #ax.set_aspect(2) # you may also use am.imshow(..., aspect="auto") to restore the
    #fig.legend()#plt.imshow(image);
    #plt.colorbar(fig)
    fig.colorbar(cax)
    ax.set_aspect('equal')
    return fig, ax

def PlotEllipse(pos, cov,volume=0.5, ax=None, fc='none', ec=[0,0,0], a=1, lw=1):
    """
    Plots an ellipse enclosing *volume* based on the specified covariance
    matrix (*cov*) and location (*pos*). Additional keyword arguments are passed on to the 
    ellipse patch artist.

    Parameters
    ----------
        pos : The location of the center of the ellipse. Expects a 2-element
            sequence of [x0, y0].
        cov : The 2x2 covariance matrix to base the ellipse on
        volume : The volume inside the ellipse; defaults to 0.5
        ax : The axis that the ellipse will be plotted on. Defaults to the 
            current axis.
    """
    #from scipy.stats import chi2

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
    #width, height = 2 * np.sqrt(chi2.ppf(volume,2)) * np.sqrt(vals)
    width, height = 2 *np.sqrt(vals)
    ellip = Ellipse(xy=pos, width=width, height=height, angle=theta, **kwrg)

    ax.add_artist(ellip)

def PlotDistribution(points, minmax):
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.scatter(*zip(*points), color = 'blue', s=3)
    meanReal = GetMean(points)
    covReal = GetCov(points)

    PlotEllipse(meanReal, covReal, ec='r', lw=1)
    ax.scatter(meanReal[0], meanReal[1], color="red", s=2)

    plt.axis([minmax[2],minmax[3],minmax[0],minmax[1]])
    ax.set_aspect('equal')
    return fig, ax

def draw_ring(ax, pos, R):
    ring = plt.Circle(pos, R, color="black", fill=False)
    ax.add_patch(ring)  
###############################################################################





"""

Implement your functions here

"""

"""
Implement a function that returns the mean vector of multiple samples (Nx2)
""" 
def GetMean(values):
    return np.mean(values, axis=0)

"""
Implement a function that returns the covariance matrix of multiple samples (Nx2)
""" 
def GetCov(values):
    mean = GetMean(values)
    var_x = var_y = cov_x_y = 0
    for value in values:
        var_x += pow(value[0] - mean[0], 2)
        var_y += pow(value[1] - mean[1], 2)
        cov_x_y += (value[0] - mean[0]) * (value[1] - mean[1])
    var_x = var_x / len(values)
    var_y = var_y / len(values)
    cov_x_y = cov_x_y / len(values)
    matrix = np.matrix([[var_x, cov_x_y], [cov_x_y, var_y]])
    return matrix

"""
Implement a function that returns a 2x2 rotation matrix for a given angle
""" 
def GetRotationMatrix(angle):
    raise NotImplementedError



"""
Implement a function that returns a 3x3  homogeneous transformation matrix  for a given angle and translation tx,ty
""" 
def GetHomTransformMatrix(angle,tx,ty):
    raise NotImplementedError


"""
Implement a function for transforming all input points with the homogeneous transformation matrix 
""" 
def ApplyTransformToPoints(mat, points):
    raise NotImplementedError

"""
Implement the multivariate probability density function 
""" 
def GetProb(x, mean, cov):
    determinant = np.linalg.det(cov)
    cov_inverse = np.linalg.inv(cov)
    z = pow(2 * math.pi, 3/2) * pow(determinant, 0.5)
    print(f"mean {mean}")
    print(numpy.transpose(mean.T))
    print(cov_inverse)
    print((x - mean))

    exp = -0.5 * (x - mean) * cov_inverse * numpy.transpose(x - mean)
    pdf = 1 / z * numpy.exp(exp)
    return pdf


"""

Global parameters

"""
np.random.seed(1337)

#min and max draw area: [x_min, x_max, y_min, y_max]
minMaxDraw = [-0.5,0.5,-0.5,0.5]
ring_r = 0.5

#number of sample points
numPointSamples = 300
#resolution of grid representation
numGridSamplesPerAxis = 100

# Infrared measurements
IR_meas = np.array([
                       [0.19, 0.38],
                       [0.12, 0.33],
                       [0.15, 0.38],
                       [0.13, 0.36],
                       [0.14, 0.33],
                       [0.18, 0.39],
                   ])

#transformation parameters
angle = np.pi/3
transX = 0.2
transY = -0.3


"""
A)
Implement GetMean and GetCov
"""
print("The IR got several measurements for the R2 robot's position:")
for meas in IR_meas:
    print(meas)
mean = GetMean(IR_meas)
cov = GetCov(IR_meas)
print("R2's mean position in the R1 frame: ")
print(mean)
print("R2's position covariance matrix in the R1 frame:")
print(cov)

"""
B)
Implement the "GetProb(x, mean, cov)" function above. The Plot2DGrid function plots the grid-based representation of the density
"""
# Check if the probability density function is correct
testProb = GetProb(mean, mean, cov)
print("Maximum probability density: " + str(testProb))    
if (np.abs(np.abs(testProb) - 348.5237586060737) > 0.000001):
    print("Error: Probability density should be ~348.5237586060737 but is " + str(testProb) )

print("Distribution plot...")
c, s = np.cos(-angle), np.sin(-angle)
R = np.array(((c, -s), (s, c)))
ring_pos = np.dot(R,np.array([-transX, -transY]))
fig, ax = Plot2DGrid(mean, cov,minMaxDraw, nsamples = numGridSamplesPerAxis);

# Add R1 
plt.plot(0, 0, color="black", marker="^")
plt.text(0.02, 0.02, "R1")

# Show ring limit in plot
draw_ring(ax, ring_pos, ring_r)

plt.show()


"""
B) C)
Implement GetRotationMatrix fucntion and use the affine transform (A,b) on mu and Sigma. The result should be mu_P and Sigma_P  
"""
A = GetRotationMatrix(angle)
b = np.array([transX,transY])

mu_P    = np.dot(A,mean)+b
Sigma_P = np.dot(np.dot(A,cov),np.transpose(A))

print("A: " + str(A))
print("b: " + str(b))
print("uP: " + str(mu_P))
print("SP: " + str(Sigma_P))
print("Transformed distribution plot...")

#Plot the resulting transform
fig, ax= Plot2DGrid(mu_P,Sigma_P,minMaxDraw, nsamples = numGridSamplesPerAxis);

# Add R1 
plt.plot(transX, transY, color="black", marker="^")
plt.text(transX+0.02, transY+0.02, "R1")

# Show ring limit in plot
draw_ring(ax, (0, 0), ring_r)

plt.show()

"""
D)
Implement the GetHomTransformMatrix and ApplyTransformToPoints functions and use them to transform the sampled points
"""

print("Plot of the IR measurements, this can be viewd as a sample-based distribution...")

#plot samples and one sigma ellipse
fig, ax= PlotDistribution(IR_meas, minMaxDraw)
draw_ring(ax, ring_pos, ring_r)
# Add R1 
plt.plot(0, 0, color="black", marker="^")
plt.text(0.02, 0.02, "R1")
plt.show()


#create a homogeneous transformation matrix and use it to transform the sampled points
H = GetHomTransformMatrix(angle,transX,transY)
transPoints = ApplyTransformToPoints(H,IR_meas)

print("H: " + str(H))
print("Transformed IR measurements")


#plot samples and one sigma ellipse
fig, ax= PlotDistribution(transPoints, minMaxDraw)
draw_ring(ax, (0, 0), ring_r)
# Add R1 
plt.plot(transX, transY, color="black", marker="^")
plt.text(transX+0.02, transY+0.02, "R1")
plt.show()


