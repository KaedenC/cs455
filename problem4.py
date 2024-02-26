import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist

num_nodes = 100
area_size = 150
k = 1.2
d = 14
interaction_range = k * d

x = np.random.uniform(0, area_size, num_nodes)
y = np.random.uniform(0, area_size, num_nodes)
#find neighbors using cdist
neighbors = []
for i in range(num_nodes):
    distances = cdist([(x[i], y[i])], np.column_stack((x,y)))[0]
    neighborIndices = np.where(distances > 0)
    nodeNeighbors = [id for id in neighborIndices[0] if distances[id] <= interaction_range]
    neighbors.append(nodeNeighbors)
plt.figure(figsize=(5,5))
plt.scatter(x,y,color='blue',label='Sensor Nodes', marker='>')

for i, nodeNeighbors in enumerate(neighbors):
    for neighborIndices in nodeNeighbors:
        plt.plot([x[i], x[neighborIndices]], [y[i], y[neighborIndices]], color='blue')
plt.xlim(0, area_size)
plt.ylim(0, area_size)

plt.grid(True)
plt.show()
