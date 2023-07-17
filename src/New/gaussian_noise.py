import numpy as np

def add_gaussian_noise(point, mean, std_dev):
    noise = np.random.normal(mean, std_dev, len(point))
    noisy_point = point + noise
    return noisy_point

# Example usage
point = np.array([1.0, 2.0, 3.0])  # Original point
mean = 0.0  # Mean of the Gaussian distribution
std_dev = 0.1  # Standard deviation of the Gaussian distribution

noisy_point = add_gaussian_noise(point, mean, std_dev)
print("Original point:", point)
print("Noisy point:", noisy_point)
