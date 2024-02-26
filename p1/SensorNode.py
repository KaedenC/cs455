import numpy as np
import matplotlib.pyplot as plt

class SensorNode:
    def __init__(self, position):
        self.position = position  # Position in 2D space
        self.velocity = np.zeros(2)  # Initial velocity set to 0

    def distance_to(self, other_node):
        """Calculate Euclidean distance to another node."""
        return np.linalg.norm(self.position - other_node.position)

    def link_to(self, other_node, r):
        """Visualize a link to another node if within range r."""
        if self.distance_to(other_node) <= r:
            plt.plot([self.position[0], other_node.position[0]], [self.position[1], other_node.position[1]], 'b-', alpha=0.5)
