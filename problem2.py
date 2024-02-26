import numpy as np
import matplotlib.pyplot as plt

num_nodes = 100
area_size = 150
k = 1.2
d = 14
interaction_range = k * d

x = np.random.uniform(0, area_size, num_nodes)
y = np.random.uniform(0, area_size, num_nodes)

# Plot nodes
plt.figure(figsize=(5, 5))
plt.scatter(x, y, color='blue', marker='>')


for i in range(num_nodes):
    circle = plt.Circle((x[i], y[i]), interaction_range, color='red', fill=False)
    plt.gca().add_patch(circle)

plt.xlim(0, area_size)
plt.ylim(0, area_size)
plt.title('Network of 100 Sensor Nodes')
plt.legend()

plt.grid(True)
plt.show()
