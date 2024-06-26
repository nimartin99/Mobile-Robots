import numpy as np
import matplotlib.pyplot as plt

np.random.seed(31)


#######################################
# set here wether to run a or b
run_b = False
#######################################


A = 1.1
B = 1.0
R = 0.8**2

C = 2.1
Q = 0.8**2


μ_0 = 1
Σ_0 = 3.6**2


# data for b)
us = np.array([2,-4])
zs = np.array([3, 1])

# data for c)
data = np.genfromtxt('./data.csv', delimiter=',')

if not run_b:
    us = data[0]
    zs = data[1]


def kalman_update(mu_t_prev, sigma_t_prev, u_t):
    # TODO
    # prediction step
    mu_t_bar = A * mu_t_prev + B * u_t
    sigma_t_bar = A*sigma_t_prev*A+R

    return (mu_t_bar, sigma_t_bar)


def kalman_correction(mu_t_bar, sigma_t_bar, z_t):
    # TODO
    K = sigma_t_bar * C / (C*sigma_t_bar*C + Q)
    mu_t = mu_t_bar + K*(z_t - C*mu_t_bar)
    sigma_t = (1- K*C)*sigma_t_bar

    if run_b:
        print("Kalmain Gain K:", K)
        print("Mu_t_bar: ", mu_t_bar)
        print("Sigma_t_bar: ", sigma_t_bar)
        print("Mu_t: ", mu_t)
        print("Sigma_t: ", sigma_t)

    return(mu_t, sigma_t)

def plot_with_uncertainty(data_points, error_bars):
    # TODO
    fig, ax = plt.subplots()
    x_data=np.arange(0, np.shape(data_points)[0])
    ax.plot(x_data, data_points, color="blue")
    ax.errorbar(x=x_data, y=data_points, yerr=error_bars, fmt="o", ecolor="red", capsize=4, elinewidth=1.5, capthick=1.5)
    ax.set_xlabel("Iteration")
    ax.set_ylabel(r"$\mu_t$")
    plt.show()

def kalman():
    mu_curr = μ_0
    sigma_curr = Σ_0

    mus = list()
    sigmas = list()

    mus.append(mu_curr)
    sigmas.append(sigma_curr)


    for i in range(0,np.shape(us)[0]):
        if run_b:
            print("Iteration of KF:", i+1)

        mu_t_bar, sigma_t_bar = kalman_update(mu_curr, sigma_curr, us[i])
        if zs[i] != 0:
            mu_curr, sigma_curr = kalman_correction(mu_t_bar, sigma_t_bar, zs[i])
        else:
            mu_curr = mu_t_bar
            sigma_curr = sigma_t_bar

        mus.append(mu_curr)
        sigmas.append(sigma_curr)

    plot_with_uncertainty(mus, sigmas)

if __name__ == '__main__':
    kalman()
