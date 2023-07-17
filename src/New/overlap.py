import numpy as np
from scipy.integrate import dblquad

def gaussian(x, y, mu, cov):
    """
    Compute the value of a 2D Gaussian distribution at coordinates (x, y).
    
    Parameters:
        x (float): x-coordinate.
        y (float): y-coordinate.
        mu (array-like): Mean vector of shape (2,).
        cov (array-like): Covariance matrix of shape (2, 2).
    
    Returns:
        float: Value of the Gaussian distribution at (x, y).
    """
    inv_cov = np.linalg.inv(cov)
    delta = np.array([x - mu[0], y - mu[1]])
    exponent = -0.5 * np.dot(np.dot(delta, inv_cov), delta.T)
    norm = 1.0 / (2 * np.pi * np.sqrt(np.linalg.det(cov)))
    return norm * np.exp(exponent)

def compute_overlap(mu1, cov1, mu2, cov2):
    """
    Compute the overlap between two 2D Gaussian distributions using numerical integration.
    
    Parameters:
        mu1 (array-like): Mean vector of the first Gaussian distribution of shape (2,).
        cov1 (array-like): Covariance matrix of the first Gaussian distribution of shape (2, 2).
        mu2 (array-like): Mean vector of the second Gaussian distribution of shape (2,).
        cov2 (array-like): Covariance matrix of the second Gaussian distribution of shape (2, 2).
    
    Returns:
        float: Overlap between the two Gaussian distributions.
    """
    integrand = lambda y, x: gaussian(x, y, mu1, cov1) * gaussian(x, y, mu2, cov2)
    overlap, _ = dblquad(integrand, -np.inf, np.inf, lambda x: -np.inf, lambda x: np.inf)
    return overlap

# Example usage
mu1 = np.array([0, 0])
cov1 = np.array([[1, 0], [0, 1]])

mu2 = np.array([0, 0])
cov2 = np.array([[1, 0], [0, 1]])

overlap = compute_overlap(mu1, cov1, mu2, cov2)
print("Overlap:", overlap)
