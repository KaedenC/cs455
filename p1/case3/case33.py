import SensorNode as SN
import numpy as np
import matplotlib.pyplot as plt
import random

n = 100
m = 2
d = 15
k = 1.2
r = k*d
epsilon = 0.1
delta_t = 0.009
c1_alpha = 30
c2_alpha = 2 * np.sqrt(c1_alpha)

static_target = []

#region Algorithm Supplements + Algo 1
def magnitude(i):
    return np.sqrt((i[0]**2) + (i[1]**2))

def n_ij(i, j):
    z = [i.x - j.x, i.y - j.y]
    return np.array(z) / np.sqrt(1 + (epsilon * (magnitude(z)**2)))

def bump_function(z):
    if 0 <= z < 0.2:
        return 1
    elif 0.2 <= z <= 1:
        return (1 + np.cos(np.pi * (z-0.2) / (1-0.2))) / 2
    else:
        return 0
    
def sigma(z):
    return z / np.sqrt(1+(z**2))

def sigma_norm(z):
    if isinstance(z, (np.ndarray, list)):  # Check if z is a vector
        return np.sqrt(1 + (epsilon*(magnitude(z)**2)))
    else:  # z is a scalar
        return (np.sqrt(1 + (epsilon*(z**2))) - 1) / epsilon
    
def a_ij(i, j):
    position_i = np.array([i.x, i.y])
    position_j = np.array([j.x, j.y])
    r_alpha = sigma_norm(r)
    distance = np.linalg.norm(position_i - position_j)  # Euclidean distance between i and j
    return bump_function(sigma_norm(distance) / r_alpha)

def phi(z):
    sig1 = sigma(z + (np.abs(5-5))/np.sqrt(4*5*5))  # Assuming this formula is correct
    return (10*sig1) / 2

def phi_alpha(z):
    r_alpha = sigma_norm(r)
    d_alpha = sigma_norm(d)
    return bump_function(z/r_alpha) * phi(z-d_alpha)

def algo_2_update_locations(nodes, dynamic_target):
    accels = []
    for i_agent in nodes:
        g_term = np.zeros(2)
        c_term = np.zeros(2)
        nf_term = np.zeros(2)
        for j_agent in i_agent.neighbors:
            # Adjusted to use the correct method of accessing positions and performing calculations
            nij = n_ij(i_agent, j_agent)  # Assuming n_ij returns a numpy array
            dist_ij = np.array([j_agent.x - i_agent.x, j_agent.y - i_agent.y])
            phi_alpha_val = phi_alpha(sigma_norm(np.linalg.norm(dist_ij)))
            g_term += nij * phi_alpha_val

            adjacencymatrixbump = a_ij(i_agent, j_agent)
            c_term += dist_ij * adjacencymatrixbump
            
        g_term *= c1_alpha
        c_term *= c2_alpha
        
        #navigational feedback term
        nf_term_g = c1_mt * np.array([i_agent.x , i_agent.y ])
        nf_term_c = c2_mt * np.array([i_agent.velocity, j_agent.velocity])
        
        ui = (g_term + c_term - nf_term) * delta_t
        accels.append(ui)
        
    # Apply accelerations to update the nodes' velocities and positions
    for node, accel in zip(nodes, accels):
        node.add_accel_to_vel(accel)
        node.add_vel_to_pos()     
      
def update_neighbors(nodes):
    for node in nodes:
        node.neighbors = []
        for potential_neighbor in nodes:
            if node != potential_neighbor:
                node.link_node(potential_neighbor, r)      

def plot_network(nodes, iteration):
    plt.figure()
    ax = plt.gca()
    for node in nodes:
        # Plot node
        ax.plot(node.x, node.y, 'b>')  # 'bo' for blue circle markers
        # Draw connections to neighbors
        for neighbor in node.neighbors:
            ax.plot([node.x, neighbor.x], [node.y, neighbor.y], 'b-') 
    plt.xlim(-0, 200)
    plt.ylim(-0, 200)
    plt.title(f'Iteration {iteration}')
    # Save the plot
    if iteration == 0 or iteration == 40 or iteration == 80 or iteration == 120 or iteration == 160 or iteration == 200:
        plt.savefig(f'network_iteration_{iteration}.png', bbox_inches='tight')
    plt.close()  # Close the plot to avoid displaying it
    
def plot_velocity_values(nodes):
    plt.figure()
    for node in nodes:
        # Assuming prev_vel is a list of tuples/lists with velocities [(vx1, vy1), (vx2, vy2), ...]
        step_numbers = list(range(len(node.prev_vel)))
        velocities = [np.sqrt(vel[0]**2 + vel[1]**2) for vel in node.prev_vel]  # Calculate magnitude for each step
        plt.plot(step_numbers, velocities, '-o', label=f'Node {node.index}')  # Plot velocity magnitude over time
    plt.title('Velocity Values of Nodes at Each Step')
    plt.xlabel('Step Number')
    plt.ylabel('Velocity Magnitude')
    plt.legend()
    plt.savefig('velocity_values.png')
    plt.close()
    
def plot_prev_positions_with_trajectory(nodes):
    plt.figure()
    for node in nodes:
        # Assuming prev_pos is a list of tuples/lists [(x1, y1), (x2, y2), ...]
        x_positions = [pos[0] for pos in node.prev_pos]
        y_positions = [pos[1] for pos in node.prev_pos]
        plt.plot(x_positions, y_positions, marker='o', markersize=4, label=f'Node {node.index}')  # Connect positions with lines
    plt.title('Trajectories of Nodes')
    plt.xlabel('X position')
    plt.ylabel('Y position')
    plt.legend()
    plt.savefig('prev_positions_trajectory.png')
    plt.close()

def plot_trajectories(nodes):
    previous_positions = {}
    plt.figure()
    ax = plt.gca()
    for node in nodes:
        previous_positions[node.index] = []
        previous_positions[node.index].append((node.x, node.y))
    
    for node_index, positions in previous_positions.items():
        if positions:
            x_values, y_values = zip(*positions)
            ax.plot(x_values, y_values, 'b-')  # Plot trajectory with red dashed line
    plt.xlim(0, 50)
    plt.ylim(0, 50)
    plt.title('Node Trajectories')
    plt.savefig(f'node_trajectories.png', bbox_inches='tight')
    plt.close()

def check_and_plot_connectivity(nodes):
    num_nodes = len(nodes)
    A = np.zeros((num_nodes, num_nodes))  # Initialize adjacency matrix

    # Fill the adjacency matrix based on the presence of links between nodes
    for i, node_i in enumerate(nodes):
        for j, node_j in enumerate(nodes):
            if node_j in node_i.neighbors:
                A[i, j] = 1

    connectivity_values = []
    for t in range(len(nodes[0].prev_pos)):  # Assuming all nodes have the same number of position records
        # Calculate the rank of A at time t (since A does not change over time in this example, rank is constant)
        rank_A = np.linalg.matrix_rank(A)
        connectivity_t = (rank_A / num_nodes)
        connectivity_values.append(connectivity_t)
    
    # Plot connectivity over time
    plt.figure()
    plt.plot(connectivity_values, '-o')
    plt.title('Connectivity of the MSN Over Time')
    plt.xlabel('Time Step')
    plt.ylabel('Connectivity')
    plt.ylim(0, 1.1)  # Ensure y-axis is scaled to show maximum connectivity value of 1
    plt.axhline(y=1, color='r', linestyle='--')
    plt.savefig('connectivity_over_time.png')
    plt.close()

        
nodes = [SN.SensorNode(random.uniform(0, 50), random.uniform(0, 50), i) for i in range(100)]
previous_positions = {}

for iteration in range(201):
    update_neighbors(nodes)
    algo_2_update_locations(nodes) 
    plot_network(nodes, iteration)

plot_velocity_values(nodes)
plot_prev_positions_with_trajectory(nodes)
check_and_plot_connectivity(nodes)
