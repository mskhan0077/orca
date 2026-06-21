import numpy as np
import sys
import os

# getting the name of the directory
# where the this file is present.
current = os.path.dirname(os.path.realpath(__file__))

# Getting the parent directory name
# where the current directory is present.
parent = os.path.dirname(current)

# adding the parent directory to 
# the sys.path.
sys.path.append(parent)
import utils


class Simulator:
    def __init__(self,
                 dt: float,
                 time_horizon: float):
        
        self.dt = dt
        self.time_horizon = time_horizon
        self.neighbour_dist = 10.0
        self.agents = []


    def reset(self):
        pass

    def add(self,
            agent):
        self.agents.append(agent)
        return agent


    def all_reached(self,
                    tol=0.2):
        return all(a.static or np.linalg.norm(a.goal - a.pos) < tol for a in self.agents)


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
            lines, num_static = utils.compute_orca_lines(
                a, self.neighbours(a), self.time_horizon, self.dt
            )
            a.new_vel = utils.compute_new_velocity(a, lines, num_static)

        for a in self.agents:
            a.vel = a.new_vel
            a.pos = a.pos + a.vel * self.dt

