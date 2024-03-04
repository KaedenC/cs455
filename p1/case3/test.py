import math
import matplotlib.pyplot as plt
import numpy as np

class SensorNode:
    def __init__(self, positionx, positiony, index):
        self.index = index
        self.x = positionx
        self.y = positiony
        self.velocity = [0, 0]
        
        self.neighbors = []
        self.prev_pos = [(positionx, positiony)]  # Initialize with starting position
        self.prev_vel = []

    def update_position(self, dx=0, dy=0):
        self.prev_pos.append((self.x, self.y))
        self.x += dx
        self.y += dy
        self.prev_vel.append((dx, dy))  # Store the delta as velocity

    def move_in_sine_wave(self, A, omega, vx, steps, dt):
        for t in range(steps):
            dx = vx * dt
            dy = A * math.sin(omega * t * dt)
            self.update_position(dx, dy)

    @staticmethod
    def center_of_mass(nodes):
        total_x = sum(node.x for node in nodes)
        total_y = sum(node.y for node in nodes)
        num_nodes = len(nodes)
        return total_x / num_nodes, total_y / num_nodes

# Sine wave parameters
A = 10  # Amplitude
T = 20  # Period
omega = (2 * np.pi) / T  # Angular frequency
vx = 2  # Horizontal speed
dt = 1  # Time step
steps = 100  # Number of steps to simulate

# Initialize a node
node = SensorNode(0, 0, 1)

# Move the node in a sine wave trajectory
node.move_in_sine_wave(A, omega, vx, steps, dt)

# Plot the node's trajectory
x_vals, y_vals = zip(*node.prev_pos)
plt.figure(figsize=(10, 5))
plt.plot(x_vals, y_vals, marker='o')
plt.title("Node's Sine Wave Trajectory")
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.grid(True)
plt.show()