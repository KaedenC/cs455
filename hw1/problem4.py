import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist

def problem4(numNodes, size, k, d):
    interactionRange = k*d

    x = np.random.uniform(0, size, numNodes)
    y = np.random.uniform(0, size, numNodes)

    #find neighbors using cdist
    neighbors = []
    for i in range(numNodes):
        distances = cdist([(x[i], y[i])], np.column_stack((x,y)))[0]
        neighborIndices = np.where(distances > 0)
        nodeNeighbors = [id for id in neighborIndices[0] if distances[id] <= interactionRange]
        neighbors.append(nodeNeighbors)
    
    plt.figure(figsize=(5,5))
    plt.scatter(x,y,color='blue',label='Sensor Nodes')

    for i, nodeNeighbors in enumerate(neighbors):
        for neighborIndices in nodeNeighbors:
            plt.plot([x[i], x[neighborIndices]], [y[i], y[neighborIndices]], color='blue')

    plt.xlim(0, size)
    plt.ylim(0, size)

    plt.grid(True)
    plt.show()
