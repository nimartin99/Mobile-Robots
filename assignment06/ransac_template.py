import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches

plt.close("all")
 

inliner_threshold = 10

def ransac_circle(pts, iterations=500, distance_threshold = 0.1):
    """
    Implement find circle here. The fuction should return a cirlce model: [[CenterPoint], Radius].
    """        
    
    current_best = 0
    best_result = None
    best_inliers = None

    #best_result = [[0,0] , 10]
    #best_inliers = pts[10:20]
    
    for i in range(0, iterations):
        # select three random points to compute circle
        p1 = pts[np.random.randint(0, len(pts))]
        p2 = pts[np.random.randint(0, len(pts))]
        p3 = pts[np.random.randint(0, len(pts))]

        model = fit_circle(np.array([p1, p2, p3]))

        # check how many points lie on this circle (with distance threshold) 
        inliners = validate_model(model, pts, distance_threshold)
        # if more than "inliner_threshold" points are part of circle, recompute it
        if len(inliners) > inliner_threshold:
            model = fit_circle(np.asarray(inliners))
            inliners = validate_model(model, pts, distance_threshold)
            
            # check if this model is better than prev found models
            if len(inliners) > current_best:
                current_best = len(inliners)
                best_result = [[model['xc'], model['yc']], model['R']]
                best_inliers = np.asarray(inliners)
    
    return best_result, best_inliers


def fit_circle(pts):
    # coordinates of the barycenter
    x = pts[:, 0]
    y = pts[:, 1]
    x_m = np.mean(x)
    y_m = np.mean(y)


    # calculation of the reduced coordinates
    u = x - x_m
    v = y - y_m

    # linear system defining the center (uc, vc) in reduced coordinates:
    #    Suu * uc +  Suv * vc = (Suuu + Suvv)/2
    #    Suv * uc +  Svv * vc = (Suuv + Svvv)/2
    Suv  = sum(u*v)
    Suu  = sum(u**2)
    Svv  = sum(v**2)
    Suuv = sum(u**2 * v)
    Suvv = sum(u * v**2)
    Suuu = sum(u**3)
    Svvv = sum(v**3)

    # Solving the linear system
    A = np.array([ [ Suu, Suv ], [Suv, Svv]])
    B = np.array([ Suuu + Suvv, Svvv + Suuv ])/2.0
    uc, vc = np.linalg.solve(A, B)

    xc_1 = x_m + uc
    yc_1 = y_m + vc

    # Calcul des distances au centre (xc_1, yc_1)
    Ri_1     = np.sqrt((x-xc_1)**2 + (y-yc_1)**2)
    R_1      = np.mean(Ri_1)
    
    return({'xc': xc_1, 'yc': yc_1, 'R': R_1})

def validate_model(model, pts, threshold):
    inliner_pts = []

    # calculate distance from point to circle center
    # if abs(dist - radius) <= threshold -> point is inliner
    for point in pts:
        dist = np.linalg.norm(point - np.array([model['xc'], model['yc']]))
        if (abs(dist - model['R']) <= threshold):
            inliner_pts.append(point)

    return inliner_pts


def render_circle(circle, color='red', alpha=0.5, border=0.0):
    """
    Renders a circle using the given color and opacity information
    """    
    patch = patches.Circle(circle[0],circle[1], facecolor=color, lw=1)
    patch.set_alpha(alpha)
    axes.add_patch(patch)
    
#
# main program start
#

# generation
I = 987315449
np.random.seed(I)

path_base = "inputs/points_"
for i in range(1, 10):
    path = path_base + str(i) + ".txt"
    print(path)
    fig = plt.figure()
    axes = fig.add_subplot(111, aspect='equal')
        
    axes.clear()

    with open(path) as f:
        # read points
        content = f.readlines()        
        pts = [np.array(pt) for pt in eval(content[0])]
            
        # plot all points
        xs,ys = zip(*pts)    
        axes.scatter(xs, ys, s=25)
        axes.set_ylabel("y")
        axes.set_xlabel("x")
            
            
        circle, circle_inliers = ransac_circle(pts, iterations=500, distance_threshold=1.5)
        if circle:
            render_circle(circle, 'green')
            axes.scatter(*np.transpose(circle_inliers), color='green', s=50)
            axes.text(0,np.max(ys), "Number of Inliners: " + str(circle_inliers.shape[0]))
            
        plt.savefig("plots/"+str((path.split("/")[-1]).split(".")[0]), format = "pdf")
        plt.show()