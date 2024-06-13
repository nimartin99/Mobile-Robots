import numpy as np
import matplotlib.pyplot as plt
import csv
import matplotlib.patches as patches

# Set random seed for reproducibility
np.random.seed(42)


# TODO: read data
x_data = np.array([])
y_data = np.array([])


# TODO: Plot the data
data = []
# TODO: Plot the data
with open('circle_data.csv', newline = '') as csvfile:
    reader = csv.reader(csvfile, delimiter = ',')
    for row in reader:
        data.append([float(i) for i in row])

data = np.asarray(data)
x_data = data[0]
y_data = data[1]

fig, ax = plt.subplots()

ax.scatter(x_data, y_data)
ax.set_ylabel("y")
ax.set_xlabel("x")
plt.show()



def fit_circle(x, y):
    # TODO
    A = np.vstack([x, y, np.ones(len(x))]).T
    b = x**2 + y**2
    c = np.linalg.pinv(A) @ b

    xc = c[0] / 2
    yc = c[1] / 2
    r = np.sqrt(c[2] + xc** 2 + yc**2)
    return(xc, yc, r)
    



def find_inliners(xc, yc, r, x, y, distance_threshold):
    inliners = []
    # find inliners for this model
    for j in range(0, len(x)):
        dist_c = np.sqrt((xc - x[j])**2 + (yc - y[j])**2)
        if np.abs(dist_c - r) < distance_threshold:
            inliners.append([x[j], y[j]])

    return np.asarray(inliners)


def ransac_circle(x, y, num_iterations, distance_threshold, min_inliers):
    # TODO   
    best_model = {}
    best_inliner_cout = 0
    best_inliners = None

    for i in range(0, num_iterations):
        r1 = np.random.randint(0, np.shape(x)[0])
        r2 = np.random.randint(0, np.shape(x)[0])
        r3 = np.random.randint(0, np.shape(x)[0])

        selected_x = [x[r1], x[r2], x[r3]]
        selected_y = [y[r1], y[r2], y[r3]]
        xc, yc, r = fit_circle(np.asarray(selected_x), np.asarray(selected_y))

        # get inliners
        inliners = find_inliners(xc, yc, r, x, y, distance_threshold)

        # recompute model
        if np.shape(inliners)[0] > min_inliers: 
            xc, yc, r = fit_circle(inliners[:,0], inliners[:,1])
            inliners_new = find_inliners(xc, yc, r, x, y, distance_threshold)

            # check if this is best model so far:
            if np.shape(inliners_new)[0] > best_inliner_cout:
                best_model["xc"] = xc
                best_model["yc"] = yc
                best_model["r"] = r
                best_inliner_cout = np.shape(inliners_new)[0] 
                best_inliners = inliners_new.copy()

    return (best_model["xc"], best_model["yc"], best_model["r"], best_inliner_cout, best_inliners)   

# RANSAC parameters
num_iterations = 100
distance_threshold = 0.5
min_inliers = 30

best_xc, best_yc, best_r, best_inlier_count, inliners = ransac_circle(x_data, y_data, num_iterations, distance_threshold, min_inliers)
print(f"Best circle: center=({best_xc}, {best_yc}), radius={best_r} with {best_inlier_count} inliers")


# TODO: Plot with color codes:
fig, ax = plt.subplots()

# plot all data points
ax.scatter(x_data, y_data)
ax.set_ylabel("y")
ax.set_xlabel("x")

# recolor data points that are inliners
ax.scatter(inliners[:, 0], inliners[:, 1], color = 'green')

# draw circle
patch = patches.Circle([best_xc, best_yc], best_r, facecolor='green', lw=1)
patch.set_alpha(0.3)
ax.add_patch(patch)

plt.show()