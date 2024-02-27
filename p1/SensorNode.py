import numpy as np
import matplotlib.pyplot as plt

class SensorNode:
    def __init__(self, position, index):
        self.index = index
        self.position = position  # Position in 2D space
        self.velocity = np.zeros(2)  # Initial velocity set to 0
        self.neighbors = []
        self.prev_pos = []
        self.prev_vel = np.zeros(2)

    def distance_to(self, other_node):
        """Calculate Euclidean distance to another node."""
        return np.linalg.norm(self.position - other_node.position)

    def link_to(self, other_node, r):
        """Visualize a link to another node if within range r."""
        if self.distance_to(other_node) <= r:
            self.neighbors.append(other_node)
            plt.plot([self.position[0], other_node.position[0]], [self.position[1], other_node.position[1]], 'b-', alpha=0.5)

    def update_vel(self, acceleration):
        #add our current velocity to the prev velocity array, then add the new velocities
        self.prev_vel = np.append(self.prev_vel, self.velocity)
        self.velocity += acceleration
        #if we're updating its velocity, i also think its safe to say we're updating the position
        self.update_pos(self.velocity)
        
    def update_pos(self, velocity):
        self.prev_pos.append(self.position)
        self.position += velocity
        
    #vector subtraction for position or velocity
    def __sub__(self, other):
        return [self.position[0] - other.position[0], self.position[1] - other.position[1]]
    
    #is not used for some odd reason :)))))
    def __mul__(self, other):
        print("yah mon")
        if isinstance(other, np.float64) or isinstance(other, int) or isinstance(other, float):
            return [self.position[0] * other, self.position[1] * other]
        
    def __truediv__(self, other):
        if isinstance(other, float) or isinstance(other, int):
            return [self.position[0] / other, self.position[1] / other]
    
