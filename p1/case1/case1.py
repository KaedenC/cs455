import SensorNode as SN
import numpy as np
import matplotlib.pyplot as plt
import random
import algorithmsupplements as AS

n = 100
m = 2
d = 15
k = 1.2
r = k*d
epsilon = 0.1
delta_t = 0.009
c1_alpha = 30
c2_alpha = 2 * np.sqrt(c1_alpha)

c1_mt = 1.1
c2_mt = 2 * np.sqrt(c1_mt)

def algo_1_update_locations(nodes):
    accels = []
    for i_agent in nodes:
        g_term = np.zeros(2)
        c_term = np.zeros(2)
        for j_agent in i_agent.neighbors:
            dist_ij = np.array([j_agent.x - i_agent.x, j_agent.y - i_agent.y])
            g_term += AS.n_ij(i_agent, j_agent)  * AS.phi_alpha(AS.sigma_norm(np.linalg.norm(dist_ij)))
            c_term += dist_ij * AS.a_ij(i_agent, j_agent)
            
        ui = ((g_term * c1_alpha) + (c_term * c2_alpha)) * delta_t
        accels.append(ui)
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
        ax.plot(node.x, node.y, 'b>')
        for neighbor in node.neighbors:
            ax.plot([node.x, neighbor.x], [node.y, neighbor.y], 'b-') 
    plt.xlim(-100, 100)
    plt.ylim(-100, 100)
    plt.title(f'Iteration {iteration}')
    if iteration == 0 or iteration == 100 or iteration == 200 or iteration == 300 or iteration == 400 or iteration == 500:
        plt.savefig(f'network_iteration_{iteration}.png', bbox_inches='tight')
    plt.close()
    
def plot_velocity_values(nodes):
    plt.figure()
    for node in nodes:
        step_numbers = list(range(len(node.prev_vel)))
        velocities = [np.sqrt(vel[0]**2 + vel[1]**2) for vel in node.prev_vel] 
        plt.plot(step_numbers, velocities, '-o', label=f'Node {node.index}')
    plt.title('Velocity Values of Nodes at Each Step')
    plt.xlabel('Step Number')
    plt.ylabel('Velocity Magnitude')
    plt.savefig('velocity_values.png')
    plt.close()
    
def plot_prev_positions_with_trajectory(nodes):
    plt.figure()
    for node in nodes:
        x_positions = [pos[0] for pos in node.prev_pos]
        y_positions = [pos[1] for pos in node.prev_pos]
        plt.plot(x_positions, y_positions, marker='o', markersize=4, label=f'Node {node.index}')
    plt.title('Trajectories of Nodes')
    plt.xlabel('X position')
    plt.ylabel('Y position')
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
            ax.plot(x_values, y_values, 'b-')
    plt.xlim(0, 50)
    plt.ylim(0, 50)
    plt.title('Node Trajectories')
    plt.savefig(f'node_trajectories.png', bbox_inches='tight')
    plt.close()

def check_and_plot_connectivity(nodes):
    num_nodes = len(nodes)
    A = np.zeros((num_nodes, num_nodes))

    for i, node_i in enumerate(nodes):
        for j, node_j in enumerate(nodes):
            if node_j in node_i.neighbors:
                A[i, j] = 1

    connectivity_values = []
    for t in range(len(nodes[0].prev_pos)):
        rank_A = np.linalg.matrix_rank(A)
        connectivity_t = (rank_A / num_nodes)
        connectivity_values.append(connectivity_t)
    
    plt.figure()
    plt.plot(connectivity_values, '-o')
    plt.title('Connectivity of the MSN Over Time')
    plt.xlabel('Time Step')
    plt.ylabel('Connectivity')
    plt.ylim(0, 1.1)
    plt.axhline(y=1, color='r', linestyle='--')
    plt.savefig('connectivity_over_time.png')
    plt.close()

nodes = [SN.SensorNode(random.uniform(0, 50), random.uniform(0, 50), i) for i in range(100)]

for iteration in range(501):
    update_neighbors(nodes)
    algo_1_update_locations(nodes) 
    plot_network(nodes, iteration)

plot_velocity_values(nodes)
plot_prev_positions_with_trajectory(nodes)
check_and_plot_connectivity(nodes)
