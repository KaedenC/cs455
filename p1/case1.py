import numpy as np
import matplotlib.pyplot as plt
from SensorNode import SensorNode
# Global Parameters
n = 100                     # Number of sensor nodes
m = 2                       # Space dimensions
d = 15                      # Desired distance among sensor nodes
k = 1.2                     # Scaling factor
r = k * d                   # Interaction range
space_dim = 150             # Space dimensions
epsilon = 0.1
delta_t = 0.009
nodes = [SensorNode(np.random.rand(m) * space_dim) for _ in range(n)]

def link_nodes(nodes, r):
    """Link nodes that are within range r of each other."""
    for i in range(len(nodes)):
        for j in range(i+1, len(nodes)):
            nodes[i].link_to(nodes[j], r)

def init(nodes, space_dim):
    # Generate sensor nodes with random positions
    plt.figure(figsize=(8, 8))
    # Plot each node
    for node in nodes:
        plt.scatter(node.position[0], node.position[1], color='red', marker='o')
    # Link neighboring nodes
    link_nodes(nodes, r)
    
    plt.title('Initial Deployment of Mobile Sensor Network')
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.xlim(0, space_dim)
    plt.ylim(0, space_dim)
    plt.grid(True)
    plt.show()

def n_ij(i, j):
    z = j.position - i.position
    return z / np.sqrt(1 + epsilon)
if __name__ == "__main__":
    init(nodes, space_dim)