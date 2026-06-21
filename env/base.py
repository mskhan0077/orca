import numpy as np
from .utils import compute_orca_lines


class Simulator:
    def __init__(self,
                 dt: float,
                 time_horizon: float,
                 neighbour_dist: float):
        
        self.dt = dt
        self.time_horizon = time_horizon
        self.neighbour_dist = neighbour_dist
        self.agents = []


    def reset(self):
        pass

    def add(self,
            agent):
        self.agents.append(agent)
        return agent


    def all_reached(self,
                    tol=0.2):
        return (a.static or np.linalg.norm(a.goal - a.pos) < tol for a in self.agents)


    def neighbours(self, a):
        out = []
        for o in self.agents:
            if o is a:
                continue
            if np.linalg.norm(o.pos - a.pos) <= self.neighbour_dist + o.radius:
                out.append(o)
        out.sort(key=lambda o: not o.static)
        return out


    def step(self):
        for a in self.agents:
            if a.static:
                a.new_vel = np.zeros(2)
                continue
            lines, num_static = compute_orca_lines(
                a, self.neighbours(a), self.time_horizon, self.dt
            )

