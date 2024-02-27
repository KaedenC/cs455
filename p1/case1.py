import numpy as np
import matplotlib.pyplot as plt
from SensorNode import SensorNode
import algorithmSupplements as aS
import sys #only used for exiting the app when closing plt
# Global Parameters
global cur_step

n = 100                     # Number of sensor nodes
m = 2                       # Space dimensions
d = 15                      # Desired distance among sensor nodes
k = 1.2                     # Scaling factor
r = k * d                   # Interaction range
space_dim = 150             # Space dimensions
c1_alpha = 30
c2_alpha = 2 * np.sqrt(c1_alpha)
epsilon = 0.1
delta_t = 0.009

def link_nodes(nodes, graph):
    """Link nodes that are within range r of each other."""
    for i in range(len(nodes)):
        for j in range(i+1, len(nodes)):
            nodes[i].link_to(nodes[j], r, graph)
            
def update_neighbors(nodes, graph):
    for node in nodes:
        for other_node in nodes:
            if node is not other_node:
                node.link_to(other_node, r, graph)
            else: #cannot add yourself as a neighbor
                continue
        
def algo_1_update_locations(nodes):
    accels = []
    for i_agent in nodes:
        g_term = np.zeros(2)
        c_term = np.zeros(2)
        for j_agent in i_agent.neighbors:
            g_term += aS.n_ij(i_agent, j_agent) * aS.phi_alpha(aS.sigma_norm(i_agent - j_agent))
            c_term += (j_agent - i_agent) * aS.a_ij(i_agent, j_agent)
            print(c_term)
        g_term *= c1_alpha
        c_term = np.array(c_term)
        c_term *= c2_alpha
        
        ui = (g_term + c_term) * delta_t
        print(ui)
        accels.append(ui)
        
    #then apply accelerations to the nodes so that we can update our velocity and position 
    for i, (node, accel) in enumerate(zip(nodes, accels)):
        node.update_vel(accel)
        
        
def draw_paths(nodes, plt):
    pass

 
def main():
    nodes = [SensorNode(np.random.rand(m) * space_dim) for _ in range(n)]
    cur_step = 0
    
    #setup grid stuff for the plots
    plt.ion()
    figure = plt.figure(figsize=(10,10))
    grid = figure.add_gridspec(2,3)
    sim_section = figure.add_subplot(grid[:2, 1:])
    vel_section = figure.add_subplot(grid[0,0])
    con_section = figure.add_subplot(grid[1,0])
    figure.canvas.manager.set_window_title("MSN Fragmentation")
    sim_section.set_title(f"Simulation Graph: Step {cur_step}")
    sim_section.set_xlim(0, space_dim)
    sim_section.set_ylim(0, space_dim)
    vel_section.set_title("Velocity Graph")
    vel_section.set_xlabel("Simulation step")
    vel_section.set_ylabel("Velocity")
    con_section.set_title("Connection Graph")
    con_section.set_xlabel("Simulation Step")
    con_section.set_ylabel("Connectivity")
    
    #main simulation junk
    while True:
        cur_step += 1
        sim_section.clear()  # Clear the previous drawings
        sim_section.set_title(f"Simulation Graph: Step {cur_step}")
        sim_section.set_xlim(0, space_dim)
        sim_section.set_ylim(0, space_dim)
        
        # update neighbors and sim
        update_neighbors(nodes, sim_section)
        algo_1_update_locations(nodes)
        #prob need to do something with updating the connection
        figure.canvas.draw()
        figure.canvas.flush_events()
        plt.pause(0.05)  # Adjust the pause time as needed for visualization
   
#using this to check if the math actually adds up b/c im dumb      
def test():
    nodes = [SensorNode(np.random.rand(m) * 100) for _ in range(20)]
    figure = plt.figure(figsize=(10,10))
    grid = figure.add_gridspec(2,3)
    sim_section = figure.add_subplot(grid[:2, 1:])
    vel_section = figure.add_subplot(grid[0,0])
    con_section = figure.add_subplot(grid[1,0])
    figure.canvas.manager.set_window_title("MSN Fragmentation")
    sim_section.set_title("Simulation Graph: Step")
    sim_section.set_xlim(0, 100)
    sim_section.set_ylim(0, 100)
    vel_section.set_title("Velocity Graph")
    vel_section.set_xlabel("Simulation step")
    vel_section.set_ylabel("Velocity")
    con_section.set_title("Connection Graph")
    con_section.set_xlabel("Simulation Step")
    con_section.set_ylabel("Connectivity")
    
    # link_nodes(nodes, sim_section)
    update_neighbors(nodes, sim_section) #this works fine so thats cool.
    algo_1_update_locations(nodes)
    plt.show()

        
if __name__ == "__main__":
    main()