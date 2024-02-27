import matplotlib.pyplot as plt
import numpy
import sys


# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED   = (255, 0, 0)
GREEN = (0, 255, 0)
GRAY  = (40, 40, 40)
BLUE  = (0, 100, 255)
ORANGE = (255, 165, 0)


# Simulation
AGENT_COUNT = 100
ENVIRONMENT_WIDTH = 50
ENVIRONMENT_HEIGHT = 50
EPSILON = 0.1
DELTA_T = 0.001
C_1_ALPHA = 30
C_2_ALPHA = 2 * numpy.sqrt(C_1_ALPHA)
K = 1.2
D = 15
R = K * D


# Graph
MAX_DATA_POINTS = 50
current_simulation_step = 0
connectivity = numpy.empty(MAX_DATA_POINTS)




class Vector:
    """
    Simple Vector class
    """
    def __init__(self, x, y):
        self.x = float(x)
        self.y = float(y)


    @property
    def magnitude(self):
        return numpy.sqrt((self.x**2) + (self.y**2))



    def __repr__(self):
        return f"Vector({self.x:.4}, {self.y:.4})"

    def __add__(self, other):
        if not isinstance(other, Vector): raise ValueError("Can only add 2 vectors")
        return Vector(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        if not isinstance(other, Vector): raise ValueError("Can only subtract 2 vectors")
        return Vector(self.x - other.x, self.y - other.y)

    def __mul__(self, other):
        if isinstance(other, float) or isinstance(other, int):
            return Vector(self.x * other, self.y * other)
        raise ValueError("Can only multiply Vector by a scalar")

    def __truediv__(self, other):
        if isinstance(other, float) or isinstance(other, int):
            return Vector(self.x / other, self.y / other)
        raise ValueError("Can only divide Vector by a scalar")



class Agent:

    def __init__(self, x, y, index):
        self._position = Vector(x, y)
        self.index = index

        self._velocity = Vector(0, 0)

        self.neighbors = []

        self.previous_positions = []
        self.previous_velocities = numpy.zeros(MAX_DATA_POINTS)


    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, new):
        self.previous_positions.append(self._position)
        self._position = new

    @property
    def velocity(self):
        return self._velocity

    @velocity.setter
    def velocity(self, new):
        if current_simulation_step >= MAX_DATA_POINTS:
            self.previous_velocities = numpy.roll(self.previous_velocities, -1)
            self.previous_velocities[MAX_DATA_POINTS - 1] = self._velocity.magnitude
        else:
            self.previous_velocities[current_simulation_step] = self._velocity.magnitude
        self._velocity = new



def distance(v1, v2):
    """
    Distance between 2 vectors
    """
    return (v2-v1).magnitude



def n_ij(i, j):
    z = j.position - i.position
    return z / numpy.sqrt(1 + (EPSILON * (z.magnitude**2)))



def sigma_norm(z):
    if isinstance(z, Vector): return (numpy.sqrt(1 + (EPSILON*(z.magnitude**2))) - 1) / EPSILON

    return (numpy.sqrt(1 + (EPSILON*(z**2))) - 1) / EPSILON



def bump_function(z, h=0.2):
    if 0 <= z < h: return 1

    elif h <= z <= 1: return (1 + numpy.cos(numpy.pi * (z-h) / (1-h))) / 2

    else: return 0



def a_ij(i, j):
    r_alpha = sigma_norm(R)
    v = bump_function(sigma_norm(j.position - i.position) / r_alpha)
    return v



def sigma1(z):
    return z / numpy.sqrt(1+(z**2))



def phi(z, a=5, b=5):
    c = numpy.absolute(a-b) / numpy.sqrt(4 * a * b)
    return (((a + b) * sigma1(z + c)) + (a - b)) / 2



def phi_alpha(z):
    r_alpha = sigma_norm(R)
    d_alpha = sigma_norm(D)
    return bump_function(z/r_alpha) * phi(z-d_alpha)



def update_neighbors(agents):
    """
    Update the neighbors list of the agents
    """
    for current_agent in agents:
        neighbors = []
        for other_agent in agents:
            # Cant be your own neighbor
            if current_agent is other_agent: continue

            if distance(current_agent.position, other_agent.position) < R:
                neighbors.append(other_agent)

        current_agent.neighbors = neighbors



def draw_agent_connections(agents, axis):
    # Draw the connections over the radiuses
    for agent in agents:

        # Draw the connections
        for neighbor in agent.neighbors:
            locations = [(p.x, p.y) for p in [agent.position, neighbor.position]]
            axis.plot(*zip(*locations), color="blue", linewidth=0.3)



def draw_agent_paths(agents, axis):
    for i, a in enumerate(agents):
        locations = [(p.x, p.y) for p in a.previous_positions + [a.position]]
        axis.plot(*zip(*locations), color="fuchsia", linewidth=2)



def draw_agent_bodies(agents, axis):
    x = numpy.empty(AGENT_COUNT)
    y = numpy.empty(AGENT_COUNT)

    for i, a in enumerate(agents):
        x[i] = a.position.x
        y[i] = a.position.y

    axis.plot(x, y, marker=(3, 1, 30), markersize=8, color="black", linestyle="None")



def update_connectivity(agents):
    global connectivity

    # Calculate connectivity
    adjacency_matrix = numpy.zeros((AGENT_COUNT, AGENT_COUNT))
    for i in agents:
        for n in i.neighbors:
            adjacency_matrix[i.index][n.index] = 1


    if current_simulation_step >= MAX_DATA_POINTS:
        connectivity = numpy.roll(connectivity, -1)
        connectivity[MAX_DATA_POINTS - 1] = numpy.linalg.matrix_rank(adjacency_matrix) / AGENT_COUNT
    else:
        connectivity[current_simulation_step] = numpy.linalg.matrix_rank(adjacency_matrix) / AGENT_COUNT



def draw_velocity_graph(agents, axis):
    for agent in agents:
        if current_simulation_step < MAX_DATA_POINTS:
            min_x = 0
            size = current_simulation_step+1
        else:
            min_x = current_simulation_step - MAX_DATA_POINTS
            size = MAX_DATA_POINTS

        axis.plot(numpy.linspace(min_x, current_simulation_step, size), agent.previous_velocities[:current_simulation_step+1])



def draw_connectivity_graph(connectivities, axis):
    if current_simulation_step < MAX_DATA_POINTS:
        min_x = 0
        size = current_simulation_step+1
    else:
        min_x = current_simulation_step - MAX_DATA_POINTS
        size = MAX_DATA_POINTS


    axis.plot(numpy.linspace(min_x, current_simulation_step, size), connectivities[:current_simulation_step+1])



def update_agent_locations(agents, time_step):
    # Calculate accelerations
    accelerations = []
    for i in agents:
        gradient_term = Vector(0, 0)
        consensus_term = Vector(0, 0)

        for j in i.neighbors:
            gradient_term  += n_ij(i, j) * phi_alpha(sigma_norm(j.position - i.position))
            consensus_term += (j.position - i.position) * a_ij(i, j)

        gradient_term *= C_1_ALPHA
        consensus_term *= C_2_ALPHA

        u = (gradient_term + consensus_term) * time_step
        accelerations.append(u)

    # Once all of the accelerations are known, apply them
    for i, (agent, acceleration) in enumerate(zip(agents, accelerations)):
        agent.velocity += acceleration
        agent.position += agent.velocity

    return True



def main():
    global current_simulation_step
    # Random list of agents
    agents = [Agent(numpy.random.random()*ENVIRONMENT_WIDTH, numpy.random.random()*ENVIRONMENT_HEIGHT, i) for i in range(AGENT_COUNT)]



    # Set up figure
    fig = plt.figure(figsize=(15, 9))
    fig.canvas.mpl_connect("close_event", lambda _: sys.exit(0))

    gridspec = fig.add_gridspec(3, 2)

    trajectory = fig.add_subplot(gridspec[:2, :2])
    velocity_graph = fig.add_subplot(gridspec[2, 0])
    connectivity_graph = fig.add_subplot(gridspec[2, 1])


    fig.canvas.manager.set_window_title("Project 1: Case 1")


    while True:
        # Update the simulation
        update_neighbors(agents)
        update_agent_locations(agents, DELTA_T)
        update_connectivity(agents)


        # Prepare graphs for drawing
        trajectory.cla()
        trajectory.set_title(f"Trajectory (Step: {current_simulation_step})")

        velocity_graph.cla()
        velocity_graph.set_title(f"Velocity graph (Step: {current_simulation_step})")
        velocity_graph.set(xlabel="Simulation step", ylabel="||Velocity||")

        connectivity_graph.cla()
        connectivity_graph.set_title(f"Connectivity (Step: {current_simulation_step}) [{connectivity[min(current_simulation_step, MAX_DATA_POINTS-1)]}]")
        connectivity_graph.set(xlabel="Simulation step", ylabel="Connectivity")
        connectivity_graph.set_ylim(bottom=-0.05, top=1.05)

        fig.tight_layout()

        # Display the current state
        draw_agent_connections(agents, trajectory)
        draw_agent_paths(agents, trajectory)
        draw_agent_bodies(agents, trajectory)



        # Draw the graphs
        draw_velocity_graph(agents, velocity_graph)
        draw_connectivity_graph(connectivity, connectivity_graph)



        fig.canvas.draw()
        plt.pause(0.000000000001)
        current_simulation_step += 1





if __name__ == "__main__":
    main()
