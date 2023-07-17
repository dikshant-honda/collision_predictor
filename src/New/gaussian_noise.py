import numpy as np
import matplotlib.pyplot as plt

def multivariate_normal(x, d, mean, covariance):
    x_m = x - mean
    return (1. / (np.sqrt((2 * np.pi)**d * np.linalg.det(covariance))) * 
            np.exp(-(np.linalg.solve(covariance, x_m).T.dot(x_m)) / 2))

# Plot bivariate distribution
def generate_surface(mean, covariance, d, shift):
    nb_of_x = 100 # grid size
    xs = np.linspace(-40 + shift , 40 + shift , num=nb_of_x)
    ys = np.linspace(-40 , 40 , num=nb_of_x)
    x, y = np.meshgrid(xs, ys) # Generate grid
    pdf = np.zeros((nb_of_x, nb_of_x))
    
    for i in range(nb_of_x):
        for j in range(nb_of_x):
            pdf[i,j] = multivariate_normal(
                np.matrix([[x[i,j]], [y[i,j]]]), 
                d, mean, covariance)
    return x, y, pdf 
 
mean = np.matrix([[0], [0.]])
cov = np.matrix([
        [4., 0.], 
        [0., 2.]])
d = 2  
x, y, pdf = generate_surface(mean, cov, d, 0)

plt.figure()
contour = plt.contour(x, y, pdf, levels = 100, cmap = "viridis")

plt.title('Contour plot for positions')
plt.xlabel('X')
plt.ylabel('Y')
plt.xlim(-40, 120)
plt.ylim(-20, 20)
# plt.axis('equal')

init_pos = np.matrix([[0.], [0.]])
v = 10

for t in range(1, 10):
    bivariate_mean = np.matrix([[v*t], [0.]])
    bivariate_covariance = np.matrix([
        [4., 0.], 
        [0., 2.]]) 
    x, y, p = generate_surface(
        bivariate_mean, bivariate_covariance, d, v*t)
    
    contour.remove()
    contour = plt.contour(x, y, p, levels = 33, cmap = "viridis")
    
    plt.draw()
    plt.pause(0.5)
 
plt.show()
