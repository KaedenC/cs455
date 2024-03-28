#project parameters
import numpy as np
import matplotlib.pyplot as plt

num_nodes = 20
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
    nodes.append([x, y])

#Initialize variable to store the previous position and velocities of all nodes
# node_positions = np.zeros((len(t), num_nodes, m))
node_positions = np.zeros((len(t), num_nodes, m))
node_velocities = np.zeros((len(t), num_nodes))
connectivity = np.zeros((len(t), 1))

#initialize center of mass trajectory of the nodes for each time step
center_of_mass_trajectory = np.zeros((len(t), m))

#initialize neighbor list for the nodes
neighbors = [[] for i in range(num_nodes)]

def plot_nodes():
    plt.figure()
    for node in nodes:
        plt.plot(node[0], node[1], marker='>', color='blue')

    link_sensor_nodes()  

def plot_obstacles():
    for (x, y), radius in obstacles:
        plt.gca().add_patch(plt.Circle((x, y), radius, color='red'))
                            
def plot_target():
    plt.plot(g_target[0], g_target[1], marker='o', color='green')

#Methods used for the simulation
def link_sensor_nodes():
    #link the nodes that are within the interacti on range and plot their link with a lightblue line
    for i in range(num_nodes):
        for j in range(i+1, num_nodes):
            if np.linalg.norm(np.array(nodes[i]) - np.array(nodes[j])) < r:
                neighbors[i].append(j)
                neighbors[j].append(i)
                plt.plot([nodes[i][0], nodes[j][0]], [nodes[i][1], nodes[j][1]], color='blue')

def plot_simulation(iteration):
    plot_nodes()
    plot_obstacles()
    plot_target()
    #save plot as a png file
    plt.savefig(f'flocking_simulation_{iteration}.png')

def plot_trajectories():
    plt.figure()
    for i in range(num_nodes):
        plt.plot(node_positions[:, i-1, 0], node_positions[:, i-1, 1], label=f'Node {i+1}')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Trajectory of Nodes')
    #save plot as a png file
    plt.savefig('flocking_trajectories.png')

def plot_previous_velocity():
    plt.figure()
    for i in range(num_nodes):
        plt.plot(t, node_velocities[:, i-1], label=f'Node {i+1}')
        plt.xlabel('Time')
        plt.ylabel('Velocity')
        plt.title('Previous Velocity of Nodes')
    #save plot as a png file
    plt.savefig('flocking_previous_velocity.png')

def plot_connectivity():
    plt.figure()
    plt.plot(t, connectivity)
    plt.xlabel('Time')
    plt.ylabel('Connectivity')
    plt.title('Connectivity of Nodes')
    #save plot as a png file
    plt.savefig('flocking_connectivity.png')

def adjacency_matrix():
    #initialize adjacency matrix
    adjacency_matrix = np.zeros((num_nodes, num_nodes))
    for i in range(num_nodes):
        for j in range(num_nodes):
            if np.linalg.norm(np.array(nodes[i]) - np.array(nodes[j])) < r:
                adjacency_matrix[i, j] = 1
    return adjacency_matrix

def obstacle_calculations(obstacle_center, obstacle_radius, nodes, velocities, n):
    qk = np.zeros((n,2))
    I = np.eye(n)
    distances = np.zeros((n,1))
    
    #calculate the distance for each node from the obstacle by calculating the euclidean distance between the nodes x and y coordinates and the obstacle center x and y coordinates
    for i in range(n):
        distances[i] = np.linalg.norm(np.array(nodes[i]) - np.array(obstacle_center))
    
    myu = obstacle_radius/distances
    #calculate ak, which contains the difference between all of the x and y coordinates of the nodes and the obstacle center
    ak = np.array(nodes) - np.array(obstacle_center)
    P = I - np.outer(ak, ak)
    pk = P * velocities
    
    for i in range(n):
        pk[i] = myu[i] * pk[i]
        qk[i] = myu[i] * np.array([x[i], y[i]]) + (1 - myu[i]) * obstacle_center
        
    return qk, pk
    
def simulate_flocking():
    for iteration in range(len(t)):
        print(iteration)
        #store the center of mass of the nodes in the trajectory tuple
        center_of_mass_trajectory[iteration] = np.mean(nodes, axis=0)

        #store the previous positions and velocities of the nodes
        node_positions[iteration] = np.array(nodes)
        
        #then, lets check the connectivity of the nodes by ranking the adjacency matrix of the nodes
        #calculate adjacency matrix
        a_ij = adjacency_matrix()
        
        #then calculate the connectivity of the nodes
        connectivity[iteration] = np.linalg.matrix_rank(a_ij)/num_nodes
        
        #the velocities are calculated as the magnitude of the difference between the current and previous positions
        #something like magnitude of ((x - old_x)/delta_t)+(y - old_y)/delta_t)
        for i in range(num_nodes):
            if iteration > 0:
                node_velocities[iteration, i] = np.linalg.norm((np.array(nodes[i]) - node_positions[iteration-1, i])/delta_t)
        
        #lets update the neighbors
        link_sensor_nodes()
        
        #now lets throw in algorithm 3                
        
        #calculate the gradient term
    
        #calculate the conensus term
    
        #then with obstacle avoidance, we need to calculate the total betagradient and total betaconsensus
        #so init beta g and c
        total_beta_g = np.zeros((1, m))
        total_beta_c = np.zeros((1, m))
        #then calculate obstacle avoidance for all obstacles
        for obstacle in obstacles:
            obstacle_center = obstacle[0]
            obstacle_radius = obstacle[1]
            obstacle_calculations(obstacle_center, obstacle_radius, nodes, node_velocities[iteration], num_nodes)
        
                
            
        #at intervals lets create a snapshot of the simulation
        if iteration in snapshot_t:
            plot_simulation(iteration)

if __name__ == "__main__":
    choice = input("debug: 1\nSimulation: 2\n")
    if choice == '2':
        print('testing simulation...')
        simulate_flocking()
        plot_trajectories()
        plot_previous_velocity()
        plot_connectivity()
#plot the trajectory of the nodes by creating a line through their previous positions
