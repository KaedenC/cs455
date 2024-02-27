import numpy as np
#i 
#hate
#this
#shit

epsilon = 0.1
d = 15                      # Desired distance among sensor nodes
k = 1.2                     # Scaling factor
r = k * d                   # Interaction range

def distance(i, j):
    return np.linalg.norm(i.position - j.position)


#so this works fine? cool
def magnitude(i):
    return np.sqrt((i[0]**2) + (i[1]**2))
    

def n_ij(i, j): #also works fine i think. since it returns a vector..
    z = [i.position[0] - j.position[0], i.position[1] - j.position[1]]
    return z / np.sqrt(1 + (epsilon * (magnitude(z)**2)))

def bump_function(z):
    if 0 <= z < 0.2:
        return 1
    elif 0.2 <= z <= 1:
        return (1 + np.cos(np.pi * (z-0.2) / (1-0.2))) / 2
    else:
        return 0
    
def sigma(z):
    return z / np.sqrt(1+(z**2))
#bc adjacency matrix does both array and an int, need to check what we are passing in
def sigma_norm(z):
    if isinstance(z, np.ndarray) or isinstance(z, list): #do the calc for sigma norm between two vector positions... FUCK 
        return np.sqrt(1 + (epsilon*(magnitude(z)**2)))
    else:
        return np.sqrt(1 + (epsilon*(z**2)) - 1) / epsilon
    
def a_ij(i, j):
    r_alpha = sigma_norm(r)
    return bump_function(sigma_norm(i.position - j.position) / r_alpha)
    
#supplement for phi_alpha
def phi(z):
    sig1 = sigma(z + (np.abs(5-5))/np.sqrt(4*5*5)) #uneven sigmoidal function
    return (10*sig1) / 2

def phi_alpha(z):
    r_alpha = sigma_norm(r)
    d_alpha = sigma_norm(d)
    return bump_function(z/r_alpha) * phi(z-d_alpha)

#huh???? maybe use scipy for the integral shit, idk im fucked
def psi_alpha(z):
    np.cumsum()