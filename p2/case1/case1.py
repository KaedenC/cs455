import SensorNode as SN
import numpy as np
import matplotlib.pyplot as plt
import math
import random
import algorithmsupplements as AS

#finna catch a case with all these fuckin cases

#glhf algorithm 3 has a similar format to algorithm2 but it has some differences

#uialpha = c1alpha * (sum (sigma_norm(qj - qi))ni,j ) + c2alpha * (sum (aij(q)(pj-pi)))

#uibeta = c1beta * (sum (psipeta * (sigmanorm(q^i,k - qi))n^i,k))

n = 150
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

def algo_2_update_locations(nodes, g_agentx, g_agenty, g_agentvel):
    accels = []
    for i_agent in nodes:
        g_term = np.zeros(2)
        c_term = np.zeros(2)
        nf_term = np.zeros(2)
        for j_agent in i_agent.neighbors:
            dist_ij = np.array([j_agent.x - i_agent.x, j_agent.y - i_agent.y])
            g_term += AS.n_ij(i_agent, j_agent) * AS.phi_alpha(AS.sigma_norm(np.linalg.norm(dist_ij)))

            adjacencymatrixbump = AS.a_ij(i_agent, j_agent)
            c_term += dist_ij * adjacencymatrixbump 
        #navigational feedback term
        nf_term_g = c1_mt * np.array([i_agent.x - g_agentx, i_agent.y - g_agenty])
        nf_term_c = c2_mt * np.array([i_agent.velocity[0] - g_agentvel[0], i_agent.velocity[1] - g_agentvel[1]]) 
        nf_term = nf_term_g - nf_term_c
        
        ui = ((g_term * c1_alpha) + (c_term * c2_alpha) - nf_term) * delta_t
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
    plt.xlim(-0, 400)
    plt.ylim(-0, 300)
    plt.title(f'Iteration {iteration}')
    if iteration == 0 or iteration == 40 or iteration == 80 or iteration == 120 or iteration == 160 or iteration == 200:
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
    plt.legend()
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
    
def center_of_mass(nodes):
        total_x = sum(node.x for node in nodes)
        total_y = sum(node.y for node in nodes)
        num_nodes = len(nodes)
        return [total_x / num_nodes, total_y / num_nodes]
    
def plot_gamma_agent(node):
    x_vals, y_vals = zip(*node.prev_pos)
    plt.figure(figsize=(10, 5))
    plt.plot(x_vals, y_vals, marker='o')
    plt.title("Node's Sine Wave Trajectory")
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.grid(True)
    plt.savefig("Gamma_Agent_Trajectory.png")
    
def plot_COM(COM):
    x_vals, y_vals = zip(*COM)
    plt.figure(figsize=(10,5))
    plt.plot(x_vals, y_vals, marker='o')
    plt.title("MSN's actual sine wave trajectory")
    plt.xlabel('X position')
    plt.ylabel('Y Position') 
    plt.grid(True)
    plt.savefig("Center_of_Mass_Trajectory.png")

def create_obstacles():
    pass


g_pos = []
nodes = [SN.SensorNode(random.uniform(0, 75), random.uniform(0, 75), i) for i in range(n)]
g_pos.append(center_of_mass(nodes))
A = 5
T = 100
omega = (2 * np.pi) / T
vx = 2
steps = 201
g_agent = SN.SensorNode(250, 25, 101)

for iteration in range(201):
    update_neighbors(nodes)
    g_pos.append(center_of_mass(nodes))
    g_agentx = g_agent.prev_pos[iteration][0]
    g_agenty = g_agent.prev_pos[iteration][1]
    g_agentvel = g_agent.prev_vel[iteration]
    algo_2_update_locations(nodes, g_agentx, g_agenty, g_agentvel)
    plot_network(nodes, iteration)

plot_velocity_values(nodes)
plot_prev_positions_with_trajectory(nodes)
check_and_plot_connectivity(nodes)
plot_gamma_agent(g_agent)
plot_COM(g_pos)