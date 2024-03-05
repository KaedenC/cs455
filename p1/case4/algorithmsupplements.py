import numpy as np

d = 15
k = 1.2
r = k*d
epsilon = 0.1

def magnitude(i):
    return np.sqrt((i[0]**2) + (i[1]**2))

def n_ij(i, j):
    z = [i.x - j.x, i.y - j.y]
    return np.array(z) / np.sqrt(1 + (epsilon * (magnitude(z)**2)))

def bump_function(z):
    if 0 <= z < 0.2:
        return 1
    elif 0.2 <= z <= 1:
        return (1 + np.cos(np.pi * (z-0.2) / (1-0.2))) / 2
    else:
        return 0
    
def sigma(z):
    return z / np.sqrt(1+(z**2))

def sigma_norm(z):
    if isinstance(z, (np.ndarray, list)):
        return np.sqrt(1 + (epsilon*(magnitude(z)**2)))
    else:  # z is a scalar
        return (np.sqrt(1 + (epsilon*(z**2))) - 1) / epsilon
    
def a_ij(i, j):
    position_i = np.array([i.x, i.y])
    position_j = np.array([j.x, j.y])
    r_alpha = sigma_norm(r)
    distance = np.linalg.norm(position_i - position_j) 
    return bump_function(sigma_norm(distance) / r_alpha)

def phi(z):
    sig1 = sigma(z + (np.abs(5-5))/np.sqrt(4*5*5))
    return (10*sig1) / 2

def phi_alpha(z):
    r_alpha = sigma_norm(r)
    d_alpha = sigma_norm(d)
    return bump_function(z/r_alpha) * phi(z-d_alpha)

