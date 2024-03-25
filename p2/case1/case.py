#project parameters
import numpy as np
import matplotlib.pyplot as plt

num_nodes = 150
m = 2 #space dimension
d = 15 #distance between nodes
k = 1.2 #scaling factor
r = k * d # interaction range

epsilon = 0.1
delta_t = 0.009
t = np.arange(0, 15, delta_t) # simulation time
#generate 6 evenly spaced snapshots
snapshot_t = np.linspace(0, len(t)-1, 6, dtype=int)

c1 = 60
c2 = 2 * np.sqrt(c1)

c1_mt = 10
c2_mt = 2 * np.sqrt(c1_mt)

c1_b = 1000
c2_b = 2 * np.sqrt(c1_b)

#obstacles
obstacles = [((100, 25), 15), ((150, 30), 25), ((200, 25), 30)] #obstacle locations and radii

#target
g_target = [250, 25]

#initialize nodes in a 70x70 grid with num_nodes
nodes = []
for i in range(num_nodes):
    x = np.random.uniform(0, 70)
    y = np.random.uniform(0, 70)
    nodes.append((x, y))

#Initialize array for tracking the previous position and velocities of a node
node_positions = np.zeros((len(t), m))
node_velocities = np.zeros((len(t), m))

#initialize center of mass trajectory of the nodes for each time step
center_of_mass_trajectory = np.zeros((len(t), m))

#initialize neighbor list for the nodes
neighbors = [[] for i in range(num_nodes)]

def plot_nodes():
    plt.figure()
    for node in nodes:
        plt.plot(node[0], node[1], marker='>', color='blue')

    link_sensor_nodes(nodes)

def plot_obstacles():
    for (x, y), radius in obstacles:
        plt.gca().add_patch(plt.Circle((x, y), radius, color='red'))
                            
def plot_target():
    plt.plot(g_target[0], g_target[1], marker='o', color='green')

#Methods used for the simulation
def link_sensor_nodes(nodes):
    #link the nodes that are within the interaction range and plot their link with a lightblue line
    for i in range(num_nodes):
        for j in range(i+1, num_nodes):
            if np.linalg.norm(np.array(nodes[i]) - np.array(nodes[j])) < r:
                neighbors[i].append(j)
                neighbors[j].append(i)
                plt.plot([nodes[i][0], nodes[j][0]], [nodes[i][1], nodes[j][1]], color='blue')

def plot_simulation():
    plot_nodes()
    plot_obstacles()
    plot_target()
    plt.show()

def simulate_flocking():
    for iteration in range(len(t)):
        #store the center of mass of the nodes in the trajectory tuple
        center_of_mass_trajectory[iteration] = np.mean(nodes, axis=0)

        #store old positions and velocities
        node_positions[iteration] = np.array(nodes)
        node_velocities[iteration] = np.array(nodes)

        


        

plot_simulation()
plt.show()


