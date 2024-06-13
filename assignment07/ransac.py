import numpy as np
import matplotlib.pyplot as plt
import csv

# Set random seed for reproducibility
np.random.seed(42)

# TODO: read data
x_data = np.array([])
y_data = np.array([])

data = []
# TODO: Plot the data
with open('line_data.csv', newline = '') as csvfile:
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

def fit_line(x, y):
    # TODO
    A = np.vstack([x, np.ones(len(x))]).T
    m, b = np.linalg.lstsq(A, y)[0]
    return (m, b)

def find_inliners(m, b, x, y, distance_threshold):
    inliners = []
    # find inliners for this model
    for j in range(0, len(x)):
        if np.abs(y[j] - (m*x[j]+b)) < distance_threshold:
            inliners.append([x[j], y[j]])

    return np.asarray(inliners)

def ransac(x, y, num_iterations = 100, distance_threshold = 1, min_inliers = 30):
    # TODO
    best_model = None
    best_inliner_cout = 0
    best_inliners = None

    for i in range(0, num_iterations):
        r1 = np.random.randint(0, np.shape(x)[0])
        r2 = np.random.randint(0, np.shape(x)[0])

        selected_x = [x[r1], x[r2]]
        selected_y = [y[r1], y[r2]]
        m, b = fit_line(np.asarray(selected_x), np.asarray(selected_y))

        # get inliners
        inliners = find_inliners(m, b, x, y, distance_threshold)

        # recompute model
        if np.shape(inliners)[0] > min_inliers: 
            m_new, b_new = fit_line(inliners[:,0], inliners[:,1])
            inliners_new = find_inliners(m_new, b_new, x, y, distance_threshold)

            # check if this is best model so far:
            if np.shape(inliners_new)[0] > best_inliner_cout:
                best_model = [m_new, b_new].copy()
                best_inliner_cout = np.shape(inliners_new)[0] 
                best_inliners = inliners_new.copy()

    return (best_model[0], best_model[1], best_inliner_cout, best_inliners)

# RANSAC parameters
num_iterations = 100
distance_threshold = 1.0
min_inliers = 30

best_m, best_c, best_inlier_count, inliners = ransac(x_data, y_data, num_iterations, distance_threshold, min_inliers)
print(f"Best line: y = {best_m}x + {best_c} with {best_inlier_count} inliers")


# TODO: Plot with color codes:
fig, ax = plt.subplots()

# plot all data points
ax.scatter(x_data, y_data)
ax.set_ylabel("y")
ax.set_xlabel("x")

# recolor data points that are inliners
ax.scatter(inliners[:, 0], inliners[:, 1], color = 'green')

# draw line
x_dummy = np.linspace(start=np.min(x_data), stop=np.max(x_data), num=100)
y_dummy = best_m * x_dummy + best_c
ax.plot(x_dummy, y_dummy, color = 'red')
plt.show()
