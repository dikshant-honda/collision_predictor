import numpy as np
import matplotlib.pyplot as plt

def multivariate_normal(x, d, mean, covariance):
    """pdf of the multivariate normal distribution."""
    x_m = x - mean
    return (1. / (np.sqrt((2 * np.pi)**d * np.linalg.det(covariance))) * 
            np.exp(-(np.linalg.solve(covariance, x_m).T.dot(x_m)) / 2))

# Plot bivariate distribution
def generate_surface(mean, covariance, d):
    nb_of_x = 50 # grid size
    xs = np.linspace(-50 , 150, num=nb_of_x)
    ys = np.linspace(-5 , 5, num=nb_of_x)
    x, y = np.meshgrid(xs, ys) # Generate grid
    pdf = np.zeros((nb_of_x, nb_of_x))
    
    for i in range(nb_of_x):
        for j in range(nb_of_x):
            pdf[i,j] = multivariate_normal(
                np.matrix([[x[i,j]], [y[i,j]]]), 
                d, mean, covariance)
    return x, y, pdf 
 
fig, ax = plt.subplots()
ax.set_xlabel('x')
ax.set_ylabel('y')
# ax.set_aspect('equal')
ax.axis([-50, 50, -25, 25])

d = 2  

init_pos = np.matrix([[0.], [0.]])
v = 5

for t in range(10):
    bivariate_mean = np.matrix([[v*t], [0.]])
    bivariate_covariance = np.matrix([
        [4., 0.], 
        [0., 2.]]) 
    x1, x2, p = generate_surface(
        bivariate_mean, bivariate_covariance, d)
    # Plot bivariate distribution
    con = ax.contourf(x1, x2, p, 33)

plt.show()
