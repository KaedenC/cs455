import numpy as np
import matplotlib.pyplot as plt
from SensorNode import SensorNode
import algorithmSupplements as aS
# Global Parameters
global cur_step

n = 100                     # Number of sensor nodes
m = 2                       # Space dimensions
d = 15                      # Desired distance among sensor nodes
k = 1.2                     # Scaling factor
r = k * d                   # Interaction range
space_dim = 50             # Space dimensions
c1_alpha = 30
c2_alpha = 2 * np.sqrt(c1_alpha)
epsilon = 0.1
delta_t = 0.009
conn = np.empty(n)

def link_nodes(nodes):
    """Link nodes that are within range r of each other."""
    for i in range(len(nodes)):
        for j in range(i+1, len(nodes)):
            nodes[i].link_to(nodes[j], r)
            
def update_neighbors(nodes):
    for node in nodes:
        for other_node in nodes:
            if node is not other_node:
                node.link_to(other_node, r)
            else: #cannot add yourself as a neighbor
                continue
        
def algo_1_update_locations(nodes):
    accels = []
    for i_agent in nodes:
        g_term = np.zeros(2)
        c_term = np.zeros(2)
        for j_agent in i_agent.neighbors:
            g_term += aS.n_ij(i_agent, j_agent) * aS.phi_alpha(aS.sigma_norm(i_agent - j_agent))  
            #c_term += (j_agent - i_agent) * aS.a_ij(i_agent, j_agent)
            #b/c this for some reason (*) doesn't want to apply sensor_nodes shit...
            temp = j_agent - i_agent
            adjacencymatrixbump = aS.a_ij(i_agent, j_agent)
            
            c_term += [temp[0] * adjacencymatrixbump, temp[1] * adjacencymatrixbump]
        g_term *= c1_alpha
        c_term = np.array(c_term)
        c_term *= c2_alpha
        
        ui = (g_term + c_term) * delta_t

        accels.append(ui)
        
    #then apply accelerations to the nodes so that we can update our velocity and position 
    for i, (node, accel) in enumerate(zip(nodes, accels)):
        node.update_vel(accel)
        
def update_conn(nodes, cur_step):
    global conn
    
    adj_m = np.zeros((100,100))
    for node in nodes:
        for neighbor in node.neighbors:
            adj_m[node.index][neighbor.index] = 1
               
    if cur_step >= 50:
        conn = np.roll(conn, -1)
        conn[50 - 1] = np.linalg.matrix_rank(adj_m) / 100
        
       
def draw_conn(nodes):
    conns = [[],[]]
    for node in nodes:
        for neighbor in node.neighbors:
            conns.append([[node.position[0], node.position[1]], [neighbor.position[0], neighbor.position[1]]])
            plt.plot(*zip(*conns), color='blue', linewidth=0.2)
  
def draw_paths(nodes):
    for node in nodes:
        # Check if the node has previous positions stored
        if node.prev_pos:
            # Include the current position as the last point in the path
            path_points = node.prev_pos + [node.position]
            # Convert the list of positions into a format suitable for plotting (two lists: one for x and another for y)
            x_vals, y_vals = zip(*path_points)
            plt.plot(x_vals, y_vals, color='blue', linewidth=0.3, alpha=0.5)

def draw_nodes(nodes):
    # Initialize arrays to store the x and y coordinates
    x = np.empty(len(nodes))
    y = np.empty(len(nodes))

    # Populate the arrays with the positions of the nodes
    for i, node in enumerate(nodes):
        x[i] = node.position[0]
        y[i] = node.position[1]

    # Plot the positions of the nodes on the graph as points
    plt.plot(x, y, marker=(3, 1, 30), markersize=8, color="blue", linestyle="None")
    
#should create a seperate graph that isnt displayed and just show a png of the graph at the end...
def draw_velocity_graph(nodes, cur_step, vel_graph):
    for node in nodes:
        # Determine the range of x values (simulation steps) to plot
        if cur_step < n:
            min_x = 0
            size = cur_step + 1
        else:
            min_x = cur_step - n
            size = n

        # Prepare the x values (simulation steps)
        x_values = np.linspace(min_x, cur_step, size)

        # Extract the velocity magnitudes up to the current step or up to the max data points limit
        # Since prev_vel stores full velocity vectors, we calculate the magnitude (speed) for plotting
        velocity_magnitudes = np.linalg.norm(node.prev_vel.reshape(-1, 2), axis=1)[:size]

        # Plot the velocity magnitudes over time for this node
        vel_graph.plot(x_values, velocity_magnitudes)
        
def draw_connectivity_graph(connectivities, graph, cur_step):
    if cur_step < n:
        min_x = 0
        size = cur_step + 1
    else:
        min_x = cur_step - n
        size = n

    graph.plot(np.linspace(min_x, cur_step, size), connectivities[:cur_step + 1])

def main():
    nodes = [SensorNode(np.random.rand(m) * space_dim, _) for _ in range(n)]
    cur_step = 0
    
    #velocity graph setup
    vel_graph = plt.figure(figsize=(5,5))
    vel_graph.title = ("Velocity Graph")
    #simulation graph
    #setup grid stuff for the plots
    plt.ion()
    plt.figure(figsize=(10,10))
    plt.title(f"Simulation Graph: Step {cur_step}")
    plt.xlim(-100, space_dim)
    plt.ylim(-100, space_dim)
    
    #main simulation junk
    while True:
        print(cur_step)
        # update neighbors and sim
        update_neighbors(nodes)
        algo_1_update_locations(nodes)
        update_conn(nodes, cur_step)
        
        #display the state then write on the graphs:
        draw_conn(nodes, figure)
        draw_paths(nodes, figure)
        # draw_nodes(nodes, figure)
        
        # draw_velocity_graph(nodes, vel_section, cur_step)
        # draw_connectivity_graph(conn, con_section, cur_step)
        #prob need to do something with updating the connection
        figure.canvas.draw()
        figure.canvas.flush_events()
        plt.pause(2)  # Adjust the pause time as needed for visualization
        cur_step += 1
        
if __name__ == '__main__':
    #Dont bother showing velocity graphs or connectivity graph until 
    #
    
    main()