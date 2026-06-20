import numpy as np


class Simulator:
    def __init__(self,
                 dt: float,
                 time_horizon: float,
                 neighbour_dist: float):
        
        self.dt = dt
        self.time_horizon = time_horizon
        self.neighbour_dist = neighbour_dist
        self.agents = []

    def add(self,
            agent):
        self.agents.append(agent)
        return agent

    def all_reached(self,
                    tol=0.2):
        return (a.static or np.linalg.norm(a.goal - a.pos) < tol for a in self.agents)

    def step(self):
        pass

