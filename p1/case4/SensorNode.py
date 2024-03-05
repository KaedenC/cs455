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

    def vel_diff(self, other):
        xvel = self.velocity[0] - other.velocity[0]
        yvel = self.velocity[1] - other.velocity[1]
        return [xvel, yvel]
    
    def add_vel_to_pos(self):
        #Add a copy of current position to prev_pos
        self.prev_pos.append([self.x, self.y])
        #add vel
        self.x += self.velocity[0]
        self.y += self.velocity[1]
        
    def add_accel_to_vel(self, accel):
        # Add a copy of current velocity to prev_vel
        curvel = self.velocity.copy()
        self.prev_vel.append(curvel)
        
        # Update velocity with acceleration
        self.velocity[0] = accel[0]
        self.velocity[1] = accel[1]
    
    def distance_to(self, other_node):
        dx = self.x - other_node.x
        dy = self.y - other_node.y
        return math.sqrt(dx**2 + dy**2)
    
    def link_node(self, other_node, r):
        if self.distance_to(other_node) <= r:
            self.neighbors.append(other_node) #Add to neighbors
            other_node.neighbors.append(self) #Also need to make the other node know that its a neighbor of this node.
    
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
    def move_in_circular_motion(self, omega, initx, inity, steps):
        for t in range(steps):
            theta = omega * t * 1
            dx = initx + np.cos(theta)*50
            dy = inity + np.sin(theta)*50
            self.update_position(dx, dy)
            
    def __sub__(self, other):
        return [self.x - other.x, self.y - other.y]
    
    def __add__(self, other):
        return [self.x + other.x, self.y + other.y]
    
    #usually multiplying with a scalar
    def __mul__(self, other):
        return [self.x * other, self.y * other]
    
    def __truediv__(self, other):
        return [self.x / other, self.y / other]
    