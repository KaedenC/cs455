import numpy as np
import matplotlib.pyplot as plt

# Parameters
n = 100                     # Number of sensor nodes
m = 2                       # Space dimensions
d = 15                      # Desired distance among sensor nodes
k = 1.2                     # Scaling factor
r = k * d                   # Interaction range
space_dim = 300             # Space dimensions
epsilon = 0.1
delta_t = 0.001
c1_alpha = 30
c2_alpha = 2 * np.sqrt(30)
# Generate random coordinates for sensor nodes
nodes = np.random.rand(n, m) * space_dim

# Function to calculate distance between two points
def distance(p1, p2):
    return np.linalg.norm(p1 - p2)

def link_nodes():
    for i in range(n):
        for j in range(i+1, n):  # Avoid duplicate checks and self-connections
            if distance(nodes[i], nodes[j]) <= r:
                plt.plot([nodes[i][0], nodes[j][0]], [nodes[i][1], nodes[j][1]], 'b-', alpha=0.5)  # Link with a blue line
def n_ij(i, j):
    z = j.position - i.position
    # Calculate the magnitude of vector z
    z_magnitude = np.linalg.norm(z)
    # Normalize z using the given non-linear formula
    normalized_z = z / np.sqrt(1 + (epsilon * (z_magnitude ** 2)))
    return normalized_z


def init():
    plt.figure(figsize=(8, 8))
    plt.scatter(nodes[:, 0], nodes[:, 1], color='blue', marker='>', label='Sensor Nodes')
    
    print(n_ij(nodes[0], nodes[1]))
    
    link_nodes()
    plt.title('Initial Deployment of Mobile Sensor Network')
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.xlim(0, space_dim)  # Set x-axis limits
    plt.ylim(0, space_dim)  # Set y-axis limits
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    init()
