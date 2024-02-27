import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist
import csv

num_nodes = 100
area_size = 150
k = 1.2
d = 14
interaction_range = k * d

x = np.random.uniform(0, area_size, num_nodes)
y = np.random.uniform(0, area_size, num_nodes)

neighbors = []
for i in range(num_nodes):
    distances = cdist([(x[i], y[i])], np.column_stack((x,y)))[0]
    neighborIndices = np.where(distances > 0)
    nodeNeighbors = [index for index in neighborIndices[0] if distances[index] <= interaction_range]
    neighbors.append(nodeNeighbors)

with open('problem3.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    
    writer.writerow(['Sensor Index', 'Neighbor Indexes'])
    
    for i in range(num_nodes):
        writer.writerow([str(int(x[i])) + " " + str(int(y[i])), neighbors[i]])

