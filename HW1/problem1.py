import numpy as np
import matplotlib.pyplot as plt

#setup const
num_nodes = 100
area_size = 150
k = 1.2
d = 14
interaction_range = k * d

#random nodes
x = np.random.uniform(0, area_size, num_nodes)
y = np.random.uniform(0, area_size, num_nodes)

# Plot nodes
plt.figure(figsize=(5, 5))
plt.scatter(x, y, color='blue', marker='>')
plt.xlim(0, area_size)    
plt.ylim(0, area_size)
plt.title('Network of 100 Sensor Nodes')
plt.legend()

plt.grid(True)
plt.show()
